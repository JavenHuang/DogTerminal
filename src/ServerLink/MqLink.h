#ifndef MQ_LINK_H
#define MQ_LINK_H

#include <QObject>
#include <QTimer>
#include <QString>
#include <QStringList>
#include <QByteArray>
#include <QAbstractSocket>

class QAmqpClient;
class QAmqpExchange;
class QAmqpQueue;

// ============================================================
// RabbitMQ 北向链路（基于项目内 QAMQP 库封装）
//   - 自动重连（断开后按周期重试）
//   - 连接成功后声明 Topic 交换机，创建自动删除队列并绑定路由键
//   - publish 发送上行消息；messageReceived 接收下行消息
// 连接就绪以 queue bound 为准（此时收发均可用）
// ============================================================
class MqLink : public QObject
{
    Q_OBJECT
public:
    explicit MqLink(QObject *parent = nullptr);
    ~MqLink();

    void configure(const QString &host, quint16 port, const QString &vhost,
                   const QString &user, const QString &password,
                   const QString &exchangeName, const QString &bindingKey,
                   int retryMs = 5000);

    void start();   // 发起连接（失败自动重试）
    void stop();

    bool isConnected() const { return _ready; }

    // mimeType 仅作消息属性标注（服务端按字节解析，不依赖它）；
    // 二进制协议消息保持 application/octet-stream，JSON 转发用 application/json
    void publish(const QString &routingKey, const QByteArray &payload,
                 const QString &mimeType = QStringLiteral("application/octet-stream"));
    void subscribe(const QString &bindingKey);
    // 追加一条绑定键（同一队列可绑多个主题，如差分定位的 la）
    void addBindingKey(const QString &key);

signals:
    void connected();     // 链路就绪（可收发）
    void disconnected();
    void messageReceived(const QString &routingKey, const QByteArray &payload);
    void errorText(const QString &what);

private slots:
    void onConnected();
    void onDisconnected();
    void onClientError();
    void onSocketError(QAbstractSocket::SocketError error);
    void onRetryTimer();
    void onExchangeDeclared();
    void onQueueDeclared();
    void onQueueBound();
    void onMessageReceived();

private:
    void setupChannels();  // 连接成功后建立交换机/队列
    void teardownChannels();

    QAmqpClient *_client;
    QAmqpExchange *_exchange;
    QAmqpQueue *_queue;

    QString _host;
    quint16 _port;
    QString _vhost;
    QString _user;
    QString _password;
    QString _exchangeName;
    QString _bindingKey;
    QStringList _extraKeys;   // 附加绑定键（差分定位等旁路数据）
    int _retryMs;

    bool _connectRequested;
    bool _ready;
    bool _consuming;          // 多绑定下 bound 事件会多次触发，只 consume 一次
    qint64 _dropReportMs;     // 未就绪丢包的上次告警时间（防刷屏）

    QTimer *_retryTimer;
};

#endif  // MQ_LINK_H
