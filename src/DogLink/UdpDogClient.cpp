#include "UdpDogClient.h"
#include "ZsibotDefs.h"

#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDateTime>
#include <QDebug>

UdpDogClient::UdpDogClient(QObject *parent)
    : DogClientBase(parent)
    , _sendPort(8081)
    , _recvPort(8080)
    , _heartbeatMs(50)
    , _timeoutMs(5000)
    , _running(false)
    , _connected(false)
    , _everRecv(false)
    , _lastRecvMs(0)
{
    _socket = new QUdpSocket(this);
    connect(_socket, &QUdpSocket::readyRead, this, &UdpDogClient::onReadyRead);

    _heartbeatTimer = new QTimer(this);
    _heartbeatTimer->setInterval(_heartbeatMs);
    connect(_heartbeatTimer, &QTimer::timeout, this, &UdpDogClient::onHeartbeatTimeout);

    _checkTimer = new QTimer(this);
    _checkTimer->setInterval(500);  // 断连检测周期
    connect(_checkTimer, &QTimer::timeout, this, &UdpDogClient::onCheckTimeout);
}

UdpDogClient::~UdpDogClient()
{
    stop();
}

void UdpDogClient::configure(const QString &dogIp, quint16 sendPort, quint16 recvPort,
                             const QString &role, int heartbeatMs, int timeoutMs)
{
    _dogIp = QHostAddress(dogIp);
    _sendPort = sendPort;
    _recvPort = recvPort;
    _role = role;
    _heartbeatMs = heartbeatMs;
    _timeoutMs = timeoutMs;
}

void UdpDogClient::start()
{
    if (_running)
        return;
    // 绑定本机状态接收端口（狗会向该端口推送状态）
    if (!_socket->bind(QHostAddress::AnyIPv4, _recvPort, QUdpSocket::ShareAddress))
    {
        qWarning() << "[DogLink] 绑定接收端口失败:" << _recvPort
                   << "error=" << _socket->errorString();
        return;
    }
    _running = true;
    _heartbeatTimer->start(_heartbeatMs);
    _checkTimer->start();
    qInfo() << "[DogLink] 已启动, 狗地址:" << _dogIp.toString()
            << "控制口:" << _sendPort << "状态口:" << _recvPort;
}

void UdpDogClient::stop()
{
    if (!_running)
        return;
    _running = false;
    _heartbeatTimer->stop();
    _checkTimer->stop();
    _socket->close();
    setConnected(false);
    qInfo() << "[DogLink] 已停止";
}

bool UdpDogClient::isConnected() const
{
    if (!_running || !_everRecv)
        return false;
    return (QDateTime::currentMSecsSinceEpoch() - _lastRecvMs) <= _timeoutMs;
}

void UdpDogClient::setConnected(bool connected)
{
    if (_connected == connected)
        return;
    _connected = connected;
    emit connectedChanged(_connected);
}

void UdpDogClient::onHeartbeatTimeout()
{
    // {"type":"heartbeat","role":"..."}
    // 心跳角色与指令角色分离：心跳不再固定以 sdk 身份周期声明，
    // 避免遥控器操控时与终端反复抢控制权（指令仍用 _role）。
    const QString hbRole = _heartbeatRole.isEmpty() ? _role : _heartbeatRole;
    QJsonObject obj;
    obj.insert(QStringLiteral("type"), QStringLiteral("heartbeat"));
    obj.insert(QStringLiteral("role"), hbRole);
    QByteArray data = QJsonDocument(obj).toJson(QJsonDocument::Compact);
    _socket->writeDatagram(data, _dogIp, _sendPort);
}

void UdpDogClient::onCheckTimeout()
{
    setConnected(isConnected());
}

void UdpDogClient::onReadyRead()
{
    bool gotValidDogMsg = false;
    while (_socket->hasPendingDatagrams())
    {
        QByteArray datagram;
        datagram.resize(int(_socket->pendingDatagramSize()));
        _socket->readDatagram(datagram.data(), datagram.size());
        if (processDatagram(datagram))
        {
            _lastRecvMs = QDateTime::currentMSecsSinceEpoch();
            _everRecv = true;
            gotValidDogMsg = true;
        }
    }
    // 只有收到合法狗消息才标记已连接
    if (_running && gotValidDogMsg)
    {
        setConnected(true);
        emit stateDataChanged();
    }
}

void UdpDogClient::sendCmd(quint32 cmdCode)
{
    // {"type":"cmd","role":"sdk","cmd":122}
    QJsonObject obj;
    obj.insert(QStringLiteral("type"), QStringLiteral("cmd"));
    obj.insert(QStringLiteral("role"), _role);
    obj.insert(QStringLiteral("cmd"), double(cmdCode));
    QByteArray data = QJsonDocument(obj).toJson(QJsonDocument::Compact);
    _socket->writeDatagram(data, _dogIp, _sendPort);
}

void UdpDogClient::sendRemote(float forward, float turn, float shift, float head)
{
    // {"type":"remote","role":"sdk","joystick":[axis0,axis1,axis2,axis3],"button":[14个0]}
    // 本狗实测轴序（与 SDK 文档顺序不一致）：
    //   axis0 = 横向平移，正值向右
    //   axis1 = 前后平移，正值向后（前进需给负值）
    //   axis2 = 原地旋转，正值航向减小（右转），左转需给负值
    //   axis3 = 头部俯仰
    // 故按本函数语义（forward 正=前进、turn 正=左转/航向增大、shift 正=左）换算符号。
    // 横向轴实测"正=右"，与 shift"正=左"相反，需取反（A 左移才能真的向左）。
    QJsonObject obj;
    obj.insert(QStringLiteral("type"), QStringLiteral("remote"));
    obj.insert(QStringLiteral("role"), _role);

    QJsonArray joystick;
    joystick.append(double(-shift));
    joystick.append(double(-forward));
    joystick.append(double(-turn));
    joystick.append(double(head));
    obj.insert(QStringLiteral("joystick"), joystick);

    QJsonArray button;
    for (int i = 0; i < 14; ++i)
        button.append(0.0);
    obj.insert(QStringLiteral("button"), button);

    QByteArray data = QJsonDocument(obj).toJson(QJsonDocument::Compact);
    _socket->writeDatagram(data, _dogIp, _sendPort);
}

void UdpDogClient::sendZeroRemote()
{
    sendRemote(0.0f, 0.0f, 0.0f, 0.0f);
}

void UdpDogClient::applyArray3(const QJsonValue &v, float out[3])
{
    if (!v.isArray())
        return;
    QJsonArray arr = v.toArray();
    for (int i = 0; i < 3 && i < arr.size(); ++i)
        out[i] = float(arr.at(i).toDouble());
}

void UdpDogClient::applyArray4(const QJsonValue &v, float out[4])
{
    if (!v.isArray())
        return;
    QJsonArray arr = v.toArray();
    for (int i = 0; i < 4 && i < arr.size(); ++i)
        out[i] = float(arr.at(i).toDouble());
}

bool UdpDogClient::processDatagram(const QByteArray &datagram)
{
    QJsonParseError err;
    QJsonDocument doc = QJsonDocument::fromJson(datagram, &err);
    if (doc.isNull() || !doc.isObject())
    {
        // 非 JSON 数据包（如 0x15 之类的二进制控制帧）：每 300 包提示一次
        if (++_badPacketCount % 300 == 1)
        {
            const quint8 first = datagram.isEmpty() ? 0 : static_cast<quint8>(datagram.at(0));
            qWarning().noquote()
                << QStringLiteral("[DogLink] 非JSON数据包 长度=%1 首字节=0x%2 累计=%3 解析=%4")
                       .arg(datagram.size())
                       .arg(first, 2, 16, QLatin1Char('0'))
                       .arg(_badPacketCount)
                       .arg(err.errorString());
        }
        return false;
    }
    QJsonObject obj = doc.object();
    const QString type = obj.value(QStringLiteral("type")).toString();

    // 只有收到合法的狗状态消息才标记已连接
    const bool isValidDogMsg = (type == QLatin1String("heartbeat") ||
                                type == QLatin1String("odom_info") ||
                                type == QLatin1String("imu_info") ||
                                type == QLatin1String("feedback"));

    if (type == QLatin1String("imu_info"))
    {
        applyArray4(obj.value(QStringLiteral("quat")), _imu.quat);
        applyArray3(obj.value(QStringLiteral("rpy")), _imu.rpy);
        const QJsonValue gyroVal = obj.value(QStringLiteral("gyro"));
        applyArray3(gyroVal, _imu.gyro);
        // 老固件不发 gyro：记下"字段缺失"，上层据此改用 odom 角速度，
        // 否则零偏估计与陀螺推进恒为 0，航向只能跟随 1Hz raw yaw
        _imu.gyroField = gyroVal.isArray();
        applyArray3(obj.value(QStringLiteral("acc")), _imu.acc);
        _imu.valid = true;
    }
    else if (type == QLatin1String("odom_info"))
    {
        applyArray3(obj.value(QStringLiteral("position")), _odom.position);
        applyArray3(obj.value(QStringLiteral("v_world")), _odom.vWorld);
        applyArray3(obj.value(QStringLiteral("omega_world")), _odom.omegaWorld);
        applyArray3(obj.value(QStringLiteral("v_body")), _odom.vBody);
        applyArray3(obj.value(QStringLiteral("omega_body")), _odom.omegaBody);
        _odom.valid = true;
    }
    else if (type == QLatin1String("feedback"))
    {
        _feedback.functionMode = Zsibot::functionModeFromString(
            obj.value(QStringLiteral("function")).toString());
        _feedback.controlMode = Zsibot::controlModeFromString(
            obj.value(QStringLiteral("control_mode")).toString());
        _feedback.motionMode = Zsibot::motionModeFromString(
            obj.value(QStringLiteral("motion_mode")).toString());
        _feedback.motionType = Zsibot::motionTypeFromString(
            obj.value(QStringLiteral("motion_type")).toString());
        _feedback.valid = true;
    }
    else if (type == QLatin1String("dev_info"))
    {
        // battery 可能为嵌套对象或平铺字段，做兼容解析
        QJsonObject bat = obj.value(QStringLiteral("battery")).toObject();
        if (bat.isEmpty())
            bat = obj;
        _battery.current = float(bat.value(QStringLiteral("current")).toDouble());
        _battery.error = bat.value(QStringLiteral("error")).toInt();
        _battery.power = bat.value(QStringLiteral("power")).toInt();
        _battery.temp = float(bat.value(QStringLiteral("temp")).toDouble());
        float volt = float(bat.value(QStringLiteral("volt")).toDouble());
        // 真狗实测 volt 上报为 mV（41668 → 41.668V），超过 1000 按 mV 折算
        // TODO: verify 与固件确认 volt 单位后固化
        if (volt > 1000.0f)
            volt /= 1000.0f;
        _battery.volt = volt;
        _battery.valid = true;
    }
    else if (type == QLatin1String("dog_state"))
    {
        _basic.power = quint32(obj.value(QStringLiteral("power")).toDouble());
        _basic.temperature = float(obj.value(QStringLiteral("temp")).toDouble());
        _basic.speed = float(obj.value(QStringLiteral("speed")).toDouble());
        _basic.sn = obj.value(QStringLiteral("sn")).toString();
        _basic.devName = obj.value(QStringLiteral("dev_name")).toString();
        _basic.ssid = obj.value(QStringLiteral("ssid")).toString();
        if (obj.contains(QStringLiteral("model")))
            _basic.model = obj.value(QStringLiteral("model")).toInt();
        _basic.valid = true;
    }
    else if (type == QLatin1String("fault_info"))
    {
        _faults.clear();
        const QJsonArray faults = obj.value(QStringLiteral("faults")).toArray();
        for (int i = 0; i < faults.size(); ++i)
        {
            const QJsonObject f = faults.at(i).toObject();
            DogFault fault;
            fault.module = f.value(QStringLiteral("module")).toString();
            fault.submodule = f.value(QStringLiteral("submodule")).toString();
            fault.errorCode = quint32(f.value(QStringLiteral("error_code")).toDouble());
            fault.level = f.value(QStringLiteral("level")).toString();
            fault.info = f.value(QStringLiteral("info")).toString();
            _faults.append(fault);
        }
    }
    else if (type == QLatin1String("speed_set"))
    {
        // 速度档位：low/mid/high 或整数
        const QJsonValue v = obj.value(QStringLiteral("speed_set"));
        if (v.isString())
            _feedback.speedLevel = Zsibot::speedLevelFromString(v.toString());
        else if (v.isDouble())
            _feedback.speedLevel = v.toInt();
    }
    else
    {
        // 未识别类型：每个 type 只提示一次，便于现场发现协议变更
        if (!_unknownTypes.contains(type))
        {
            _unknownTypes.insert(type);
            qInfo().noquote() << "[DogLink] 未识别消息 type=" << type
                              << "，字段=" << QString::fromUtf8(QJsonDocument(obj).toJson(QJsonDocument::Compact));
        }
    }

    return isValidDogMsg;
}
