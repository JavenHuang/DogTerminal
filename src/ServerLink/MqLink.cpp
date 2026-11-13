#include "MqLink.h"
#include "thirdparty/RabbitMq/qamqpclient.h"
#include "thirdparty/RabbitMq/qamqpexchange.h"
#include "thirdparty/RabbitMq/qamqpqueue.h"
#include "thirdparty/RabbitMq/qamqpglobal.h"

#include <QDebug>
#include <QAbstractSocket>

MqLink::MqLink(QObject *parent)
    : QObject(parent)
    , _client(nullptr)
    , _exchange(nullptr)
    , _queue(nullptr)
    , _port(5672)
    , _retryMs(5000)
    , _connectRequested(false)
    , _ready(false)
    , _consuming(false)
{
    _retryTimer = new QTimer(this);
    _retryTimer->setSingleShot(true);
    connect(_retryTimer, &QTimer::timeout, this, &MqLink::onRetryTimer);
}

MqLink::~MqLink()
{
    stop();
}

void MqLink::configure(const QString &host, quint16 port, const QString &vhost,
                       const QString &user, const QString &password,
                       const QString &exchangeName, const QString &bindingKey,
                       int retryMs)
{
    _host = host;
    _port = port;
    _vhost = vhost;
    _user = user;
    _password = password;
    _exchangeName = exchangeName;
    _bindingKey = bindingKey;
    _retryMs = retryMs;
}

void MqLink::start()
{
    _connectRequested = true;
    _client = new QAmqpClient(this);
    _client->setHost(_host);
    _client->setPort(_port);
    _client->setVirtualHost(_vhost);
    _client->setUsername(_user);
    _client->setPassword(_password);
    // 启用自动重连（断开后3s重试）
    _client->setAutoReconnect(true, 3000);

    connect(_client, &QAmqpClient::connected, this, &MqLink::onConnected);
    connect(_client, &QAmqpClient::disconnected, this, &MqLink::onDisconnected);
    // error 既是信号又是getter，需static_cast消歧
    connect(_client, static_cast<void (QAmqpClient::*)(QAMQP::Error)>(&QAmqpClient::error),
            this, &MqLink::onClientError);
    connect(_client, static_cast<void (QAmqpClient::*)(QAbstractSocket::SocketError)>(&QAmqpClient::socketError),
            this, &MqLink::onSocketError);

    _client->connectToHost();
    qInfo() << "[MqLink] 连接RabbitMQ:" << _host << ":" << _port
            << "vhost=" << _vhost;
}

void MqLink::stop()
{
    _connectRequested = false;
    _retryTimer->stop();
    if (_client)
    {
        _client->disconnectFromHost();
        _client->deleteLater();
        _client = nullptr;
    }
    _exchange = nullptr;
    _queue = nullptr;
    _consuming = false;
    if (_ready)
    {
        _ready = false;
        emit disconnected();
    }
}

void MqLink::publish(const QString &routingKey, const QByteArray &payload, const QString &mimeType)
{
    if (!_ready || !_exchange)
        return;
    // 必须显式指定 mimeType 以命中 QByteArray 重载：
    // 两参数调用会因 QByteArray→QString 隐式转换命中 QString 重载，
    // 二进制负载被当作文本处理（QDataStream 头部的 0x00 字节导致截断）
    _exchange->publish(payload, routingKey, mimeType);
}

void MqLink::subscribe(const QString &bindingKey)
{
    _bindingKey = bindingKey;
}

void MqLink::addBindingKey(const QString &key)
{
    const QString k = key.trimmed();
    if (k.isEmpty() || k == _bindingKey || _extraKeys.contains(k))
        return;
    _extraKeys << k;
}

void MqLink::onConnected()
{
    qInfo() << "[MqLink] 已连接，声明交换机:" << _exchangeName;
    setupChannels();
}

void MqLink::setupChannels()
{
    teardownChannels();
    if (!_client || !_client->isConnected())
        return;

    // 声明 Topic 交换机（持久化，供多终端/服务器共用）
    _exchange = _client->createExchange(_exchangeName);
    connect(_exchange, &QAmqpExchange::declared, this, &MqLink::onExchangeDeclared,
            Qt::UniqueConnection);
    connect(_exchange, static_cast<void (QAmqpChannel::*)(QAMQP::Error)>(&QAmqpChannel::error),
            this, &MqLink::onClientError, Qt::UniqueConnection);
    _exchange->declare(QAmqpExchange::Topic, QAmqpExchange::Durable);
}

void MqLink::onExchangeDeclared()
{
    if (!_client || !_client->isConnected())
        return;
    // 自动删除匿名队列：断连即删，避免重启后收到积压的过期指令
    _queue = _client->createQueue();
    connect(_queue, &QAmqpQueue::declared, this, &MqLink::onQueueDeclared,
            Qt::UniqueConnection);
    connect(_queue, &QAmqpQueue::bound, this, &MqLink::onQueueBound,
            Qt::UniqueConnection);
    connect(_queue, &QAmqpQueue::messageReceived, this, &MqLink::onMessageReceived,
            Qt::UniqueConnection);
    connect(_queue, static_cast<void (QAmqpChannel::*)(QAMQP::Error)>(&QAmqpChannel::error),
            this, &MqLink::onClientError, Qt::UniqueConnection);
    _queue->declare(QAmqpQueue::AutoDelete);
}

void MqLink::onQueueDeclared()
{
    if (!_queue)
        return;
    _queue->bind(_exchangeName, _bindingKey);
    // 额外绑定（差分定位等旁路主题）：与本终端指令同队列，由 onMqMessage 按路由键分流
    for (const QString &k : _extraKeys)
        _queue->bind(_exchangeName, k);
    _consuming = false;
}

void MqLink::onQueueBound()
{
    if (!_queue)
        return;
    // 每多一条绑定都会触发 bound；只在中止后首次调用 consume，
    // 避免重复 consume 造成同一消息被投递两次
    if (_consuming)
        return;
    _consuming = true;
    _queue->consume(QAmqpQueue::coNoAck);
    if (!_ready)
    {
        _ready = true;
        QStringList keys = QStringList() << _bindingKey << _extraKeys;
        qInfo() << "[MqLink] 链路就绪，绑定路由键:" << keys.join(QStringLiteral(" / "));
        emit connected();
    }
}

void MqLink::onMessageReceived()
{
    if (!_queue)
        return;
    const QAmqpMessage msg = _queue->dequeue();
    emit messageReceived(msg.routingKey(), msg.payload());
}

void MqLink::onDisconnected()
{
    teardownChannels();
    if (_ready)
    {
        _ready = false;
        emit disconnected();
    }
    if (_connectRequested)
    {
        qWarning() << QStringLiteral("[MqLink] 连接断开，%1ms后重试").arg(_retryMs);
        _retryTimer->start(_retryMs);
    }
}

void MqLink::onRetryTimer()
{
    if (!_connectRequested || !_client)
        return;
    if (_client->isConnected())
        return;
    qInfo() << "[MqLink] 重试连接...";
    _client->connectToHost();
}

void MqLink::onClientError()
{
    QString what;
#ifdef QAMQP_DEBUG
    what = _client ? _client->errorString() : QString();
#endif
    if (what.isEmpty())
        what = QStringLiteral("AMQP错误 code=%1").arg(int(_client ? _client->error() : -1));
    emit errorText(what);
    qWarning() << "[MqLink]" << what;
}

void MqLink::onSocketError(QAbstractSocket::SocketError error)
{
    if (!_client)
        return;

    // 按错误类型给出排查方向，避免只打印一句无上下文的错误
    QString hint;
    switch (error)
    {
    case QAbstractSocket::ConnectionRefusedError:
        hint = QStringLiteral("目标端口未监听或防火墙拦截（RabbitMQ 是否已启动？端口是否正确？）");
        break;
    case QAbstractSocket::HostNotFoundError:
        hint = QStringLiteral("主机名无法解析（检查 Host 是否写错）");
        break;
    case QAbstractSocket::SocketTimeoutError:
    case QAbstractSocket::NetworkError:
        hint = QStringLiteral("网络不可达：检查本机与 RabbitMQ 是否同网段、IP 是否正确、路由/防火墙");
        break;
    default:
        hint = _client->errorString();
        break;
    }

    const QString what = QStringLiteral("Socket错误(%1): %2 -> %3:%4，%5")
                             .arg(int(error))
                             .arg(_client->errorString())
                             .arg(_host)
                             .arg(_port)
                             .arg(hint);
    emit errorText(what);
    qWarning().noquote() << "[MqLink]" << what;
}

void MqLink::teardownChannels()
{
    _exchange = nullptr;
    _queue = nullptr;
    _consuming = false;
}
