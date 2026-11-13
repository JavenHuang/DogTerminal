#include "MatrixDogClient.h"

#include <QByteArray>
#include <QDateTime>
#include <QDebug>
#include <QHostAddress>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QThread>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
#include <stdexcept>

using namespace zenoh;

// ============================================================
// 工具：RPY 单位换算
// ============================================================
static constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;

static qint64 nowMs()
{
    return QDateTime::currentMSecsSinceEpoch();
}

// ============================================================
// CDR (XCDR1 little-endian) 通用读取器
// 参考 visualize_lidar_camera_projection.py 的 CdrReader
// ============================================================
namespace {

class CdrReader
{
public:
    CdrReader(const uint8_t *data, size_t size)
        : _data(data), _size(size), _offset(0)
    {
        if (size < 4 || data[0] != 0x00 || data[1] != 0x01) {
            throw std::runtime_error("invalid XCDR1 encapsulation header");
        }
        _offset = 4;  // 跳过 4 字节封装头
    }

    void align(size_t alignment)
    {
        const size_t relative = _offset - 4;
        const size_t aligned = 4 + ((relative + alignment - 1) & ~(alignment - 1));
        if (aligned > _size) throw std::runtime_error("CDR alignment out of range");
        _offset = aligned;
    }

    int32_t i32()   { align(4); auto v = readAt<int32_t>(_offset); _offset += 4; return v; }
    uint32_t u32()  { align(4); auto v = readAt<uint32_t>(_offset); _offset += 4; return v; }
    uint8_t u8()    { if (_offset >= _size) throw std::runtime_error("CDR u8 out of range");
                      auto v = _data[_offset++]; return v; }
    double f64()    { align(8); auto v = readAt<double>(_offset); _offset += 8; return v; }

    // ROS Header: sec(i32) + nanosec(u32) + frame_id(string)
    void skipHeader()
    {
        /*sec =*/ i32();
        /*nano =*/ u32();
        /*frame_id =*/ skipString();
    }

    void skipString()
    {
        uint32_t len = u32();
        align(1);
        if (_offset + len > _size) throw std::runtime_error("CDR string out of range");
        _offset += len;
        align(4);
    }

    template <typename T>
    T readAt(size_t offset)
    {
        if (offset + sizeof(T) > _size) throw std::runtime_error("CDR field out of range");
        T v;
        std::memcpy(&v, _data + offset, sizeof(T));
        return v;
    }

private:
    const uint8_t *_data;
    size_t _size;
    size_t _offset;
};

// ROS Quaternion (x,y,z,w double)
void readQuaternion(CdrReader &r, float out[4])
{
    out[0] = float(r.f64());
    out[1] = float(r.f64());
    out[2] = float(r.f64());
    out[3] = float(r.f64());
}

// ROS Vector3 (x,y,z double)
void readVector3(CdrReader &r, float out[3])
{
    out[0] = float(r.f64());
    out[1] = float(r.f64());
    out[2] = float(r.f64());
}

// 跳过 N 个 double
void skipNf64(CdrReader &r, int n)
{
    for (int i = 0; i < n; ++i) r.f64();
}

// 归一化四元数
void normalizeQuat(float xyzw[4])
{
    const float n = std::sqrt(xyzw[0]*xyzw[0] + xyzw[1]*xyzw[1] + xyzw[2]*xyzw[2] + xyzw[3]*xyzw[3]);
    if (n > 1e-6f) {
        xyzw[0] /= n; xyzw[1] /= n; xyzw[2] /= n; xyzw[3] /= n;
    } else {
        // 默认单位四元数
        xyzw[0] = 0.0f; xyzw[1] = 0.0f; xyzw[2] = 0.0f; xyzw[3] = 1.0f;
    }
}

// 四元数 → RPY（弧度），ZYX 顺序
void quatToRpy(float xyzw[4], float rpy[3])
{
    const float x = xyzw[0], y = xyzw[1], z = xyzw[2], w = xyzw[3];
    // Roll (X)
    float sinr_cosp = 2.0f * (w*x + y*z);
    float cosr_cosp = 1.0f - 2.0f * (x*x + y*y);
    rpy[0] = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (Y)
    float sinp = 2.0f * (w*y - z*x);
    if (std::abs(sinp) >= 1.0f)
        rpy[1] = std::copysign(3.14159265f / 2.0f, sinp);
    else
        rpy[1] = std::asin(sinp);

    // Yaw (Z)
    float siny_cosp = 2.0f * (w*z + x*y);
    float cosy_cosp = 1.0f - 2.0f * (y*y + z*z);
    rpy[2] = std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace

// ============================================================
// MatrixDogClient
// ============================================================

MatrixDogClient::MatrixDogClient(QObject *parent)
    : DogClientBase(parent)
    , _socket(new QUdpSocket(this))
    , _deadmanTimer(new QTimer(this))
    , _checkTimer(new QTimer(this))
{
    _deadmanTimer->setInterval(kDeadmanMs);   // 20Hz
    _checkTimer->setInterval(kCheckMs);

    connect(_deadmanTimer, &QTimer::timeout, this, &MatrixDogClient::onDeadmanTimer);
    connect(_checkTimer, &QTimer::timeout, this, &MatrixDogClient::onCheckTimeout);

    connect(this, &MatrixDogClient::imuReady,
            this, &MatrixDogClient::onImuReady,
            Qt::QueuedConnection);
    connect(this, &MatrixDogClient::odomReady,
            this, &MatrixDogClient::onOdomReady,
            Qt::QueuedConnection);
}

MatrixDogClient::~MatrixDogClient()
{
    stop();
}

// ---- 配置 ----
void MatrixDogClient::configure(const QString &dogIp, quint16 sendPort, quint16 recvPort,
                                const QString &role, int heartbeatMs, int timeoutMs)
{
    Q_UNUSED(role);
    _dogIp = dogIp;
    _sendPort = sendPort;
    _recvPort = recvPort;
    // Zenoh 端点：如果 recvPort 与 MATRiX 默认 7447 不同，更新 endpoint
    // 默认 tcp/127.0.0.1:7447；如果 dogIp 不是 127.0.0.1，改成 dogIp
    const QString host = (_dogIp == QStringLiteral("127.0.0.1") ||
                          _dogIp == QStringLiteral("localhost"))
                             ? QStringLiteral("127.0.0.1")
                             : _dogIp;
    _zenohEndpoint = QStringLiteral("tcp/%1:%2").arg(host).arg(_recvPort);
    Q_UNUSED(heartbeatMs);
    Q_UNUSED(timeoutMs);
}

// ---- 启停 ----
void MatrixDogClient::start()
{
    if (_running) return;

    // 1. UDP socket（Matrix_MC 控制口，纯发送）
    //    Matrix 模式下不需要绑定本地端口（纯 sendto）
    _running = true;

    // 2. Zenoh 订阅（在后台线程 open）
    startZenoh();

    // 3. 启动定时器
    _deadmanTimer->start(kDeadmanMs);
    _checkTimer->start();

    // 【修复】UDP 控制链路独立于 Zenoh 状态链：
    // 只要 UDP socket 能发出去，就认为"控制链路已就绪"，
    // 不等待 Zenoh 收到数据才标记 connected。
    // Zenoh 只影响状态上报，不挡控制指令。
    _connected.store(true);
    emit connectedChanged(true);

    qInfo() << "[MatrixDog] 已启动，控制目标:" << _dogIp << ":" << _sendPort
            << "Zenoh endpoint:" << _zenohEndpoint;
}

void MatrixDogClient::stop()
{
    if (!_running) return;
    _running = false;

    _deadmanTimer->stop();
    _checkTimer->stop();

    // 先发 stand 确保狗停下，再被动模式
    sendAction(QStringLiteral("stand"));
    sendAction(QStringLiteral("passive"));

    // 停 Zenoh
    stopZenoh();

    // 重置状态
    _connected.store(false);
    _everGotImu.store(false);
    _everGotOdom.store(false);
    _lastRecvMs.store(0);
    emit connectedChanged(false);

    qInfo() << "[MatrixDog] 已停止";
}

bool MatrixDogClient::isRunning() const { return _running; }
bool MatrixDogClient::isConnected() const { return _connected.load(); }

// ---- Zenoh 启停 ----
void MatrixDogClient::startZenoh()
{
    // zenoh::open() 是阻塞的，在后台线程执行
    _zenohThread = QThread::create([this]() {
        try {
            // zenoh-cpp 1.x API：insert_json5 需要 JSON5 格式字符串（值加引号）
            Config config = Config::create_default();
            // mode 需要是 JSON5 字符串格式："\"client\""
            config.insert_json5("mode", "\"" + _zenohMode.toStdString() + "\"");
            // endpoints 是数组格式
            std::string endpoints_json = "[\"" + _zenohEndpoint.toStdString() + "\"]";
            config.insert_json5("connect/endpoints", endpoints_json);
            // scouting 关闭
            config.insert_json5("scouting/multicast/enabled", "false");

            // Session::open 直接返回 Session（不是 variant）
            auto session = Session::open(std::move(config));
            _session = std::make_shared<Session>(std::move(session));

            const QString imuKey = _topicPrefix + _imuTopic;
            const QString odomKey = _topicPrefix + _odomTopic;

            // 订阅 IMU（1.x API：直接传 lambda，不用 closures::Data）
            auto imuSub = new auto(_session->declare_subscriber(
                KeyExpr(imuKey.toStdString()),
                [this](const Sample &sample) {
                    // 1.x API：payload 用 as_vector()
                    const auto &payload = sample.get_payload();
                    std::vector<uint8_t> data = payload.as_vector();
                    RawImu raw;
                    if (parseImuCdr(QByteArray::fromRawData(
                            reinterpret_cast<const char *>(data.data()), data.size()), raw)) {
                        std::memcpy(_imu.quat, raw.quatXyzw, sizeof(_imu.quat));
                        quatToRpy(raw.quatXyzw, _imu.rpy);
                        std::memcpy(_imu.gyro, raw.gyroBody, sizeof(_imu.gyro));
                        std::memcpy(_imu.acc, raw.accBody, sizeof(_imu.acc));
                        _imu.valid = true;
                        _imu.gyroField = true;   // CDR 帧内固定含 gyro_body
                        _everGotImu.store(true);
                        _lastRecvMs.store(nowMs());
                        QMetaObject::invokeMethod(this, "onImuReady", Qt::QueuedConnection);
                    }
                },
                closures::none));
            _imuSub = imuSub;

            // 订阅 Odom（用 new auto 绕过模板类型）
            auto odomSub = new auto(_session->declare_subscriber(
                KeyExpr(odomKey.toStdString()),
                [this](const Sample &sample) {
                    const auto &payload = sample.get_payload();
                    std::vector<uint8_t> data = payload.as_vector();
                    RawPose raw;
                    if (parseOdomCdr(QByteArray::fromRawData(
                            reinterpret_cast<const char *>(data.data()), data.size()), raw)) {
                        // 【修复】部分 MATRiX 版本 rt/odom 的 twist 恒为 0，
                        // 狗在动但速度全 0，会导致 Navigator 速度闭环没有反馈。
                        // 这里用位置差分兜底测速。
                        const qint64 t = nowMs();
                        const bool twistEmpty = (raw.vWorld[0] == 0.0f
                                                 && raw.vWorld[1] == 0.0f
                                                 && raw.vWorld[2] == 0.0f);
                        if (twistEmpty && _haveLastPos && t > _lastOdomMs) {
                            const double dt = static_cast<double>(t - _lastOdomMs) / 1000.0;
                            if (dt > 1e-3 && dt < 1.0) {
                                // 一阶低通，抑制位置量化噪声。
                                // 只对水平两轴测速：z 恒定时差分噪声会被放大成假向速度
                                const float k = 0.35f;
                                for (int i = 0; i < 2; ++i) {
                                    const float v = static_cast<float>(
                                        (raw.position[i] - _lastPos[i]) / dt);
                                    raw.vWorld[i] = raw.vWorld[i] * (1.0f - k) + v * k;
                                }
                            }
                        }
                        _lastPos[0] = raw.position[0];
                        _lastPos[1] = raw.position[1];
                        _lastPos[2] = raw.position[2];
                        _lastOdomMs = t;
                        _haveLastPos = true;

                        std::memcpy(_odom.position, raw.position, sizeof(_odom.position));
                        std::memcpy(_odom.vWorld, raw.vWorld, sizeof(_odom.vWorld));
                        std::memcpy(_odom.omegaBody, raw.omegaBody, sizeof(_odom.omegaBody));

                        // 【修复】vBody 之前直接拷贝 vWorld 是错的：
                        // 世界速度需按 yaw 反旋转得到机体系分量
                        float odomRpy[3];
                        quatToRpy(raw.quatXyzw, odomRpy);
                        const float cy = std::cos(odomRpy[2]);
                        const float sy = std::sin(odomRpy[2]);
                        _odom.vBody[0] =  cy * raw.vWorld[0] + sy * raw.vWorld[1];
                        _odom.vBody[1] = -sy * raw.vWorld[0] + cy * raw.vWorld[1];
                        _odom.vBody[2] = raw.vWorld[2];

                        _odom.valid = true;
                        _everGotOdom.store(true);
                        _lastRecvMs.store(nowMs());
                        QMetaObject::invokeMethod(this, "onOdomReady", Qt::QueuedConnection);
                    }
                },
                closures::none));
            _odomSub = odomSub;

            qInfo() << "[MatrixDog] Zenoh 已连接:" << _zenohEndpoint
                    << "IMU:" << imuKey << "Odom:" << odomKey;
        } catch (const std::exception &e) {
            qWarning() << "[MatrixDog] Zenoh 启动失败:" << e.what()
                       << "endpoint:" << _zenohEndpoint;
        }
    });
    _zenohThread->start();
}

void MatrixDogClient::stopZenoh()
{
    // 先释放 subscriber（void* 置空），再释放 session，最后 join 线程
    // Subscriber 是 new auto 创建的，但不知道具体模板类型，这里不 delete
    // 让 Session 析构时自动清理 subscriber
    _imuSub = nullptr;
    _odomSub = nullptr;
    _session.reset();

    if (_zenohThread && _zenohThread->isRunning()) {
        _zenohThread->quit();
        _zenohThread->wait(2000);
        delete _zenohThread;
        _zenohThread = nullptr;
    }
}

// ---- CDR 解析 ----
bool MatrixDogClient::parseImuCdr(const QByteArray &data, RawImu &out)
{
    try {
        CdrReader r(reinterpret_cast<const uint8_t *>(data.constData()), data.size());
        r.skipHeader();              // ROS Header

        // sensor_msgs/msg/Imu
        // orientation (x,y,z,w)
        readQuaternion(r, out.quatXyzw);
        normalizeQuat(out.quatXyzw);
        // orientation_covariance [9]
        skipNf64(r, 9);
        // angular_velocity (x,y,z)
        readVector3(r, out.gyroBody);
        // angular_velocity_covariance [9]
        skipNf64(r, 9);
        // linear_acceleration (x,y,z)
        readVector3(r, out.accBody);
        // linear_acceleration_covariance [9]
        skipNf64(r, 9);
        return true;
    } catch (const std::exception &e) {
        qWarning() << "[MatrixDog] IMU CDR 解析失败:" << e.what();
        return false;
    }
}

bool MatrixDogClient::parseOdomCdr(const QByteArray &data, RawPose &out)
{
    try {
        CdrReader r(reinterpret_cast<const uint8_t *>(data.constData()), data.size());
        r.skipHeader();              // ROS Header
        r.skipString();              // child_frame_id

        // nav_msgs/msg/Odometry
        // pose.pose.position (x,y,z)
        readVector3(r, out.position);
        // pose.pose.orientation (x,y,z,w)
        readQuaternion(r, out.quatXyzw);
        normalizeQuat(out.quatXyzw);
        // pose.covariance [36]
        skipNf64(r, 36);
        // twist.twist.linear (x,y,z)
        readVector3(r, out.vWorld);
        // twist.twist.angular (x,y,z)
        readVector3(r, out.omegaBody);
        // twist.covariance [36]
        skipNf64(r, 36);
        return true;
    } catch (const std::exception &e) {
        qWarning() << "[MatrixDog] Odom CDR 解析失败:" << e.what();
        return false;
    }
}

void MatrixDogClient::quatToRpy(float xyzw[4], float rpy[3])
{
    ::quatToRpy(xyzw, rpy);
}

// ---- 定时器槽 ----
void MatrixDogClient::onImuReady()
{
    // 主线程处理：发状态变更信号
    emit stateDataChanged();
}

void MatrixDogClient::onOdomReady()
{
    emit stateDataChanged();
}

void MatrixDogClient::onDeadmanTimer()
{
    // 30Hz 持续发送当前动作（action 协议要求输入持续刷新）
    // 无速度指令时保持 sendCmd 设置的动作（stand/jointlock/passive），
    // 不能笼统发 neutral，否则会把 walk/stand 覆盖掉
    if (_hasMoveCmd) {
        sendAction(_currentAction, _currentForward, _currentLateral, _currentYaw);
    } else {
        sendAction(_currentAction);
    }
}

void MatrixDogClient::onCheckTimeout()
{
    const bool haveData = _everGotImu.load() || _everGotOdom.load();
    const qint64 sinceLast = nowMs() - _lastRecvMs.load();

    // 【修复】状态链判定：Zenoh 收到过数据 && 末包在超时内
    // 注意：_connected 现在由 UDP socket 就绪决定，Zenoh 只影响状态链提示
    const bool stateLinkOk = haveData && (sinceLast <= kTimeoutMs);
    if (stateLinkOk != _stateLinkOk) {
        _stateLinkOk = stateLinkOk;
        if (stateLinkOk) {
            qInfo() << "[MatrixDog] Zenoh 状态链路已连通";
            // 首次连通时，填充 DogBasic 的 model/sn（MATRiX 无这些字段，填模拟值）
            if (!_basic.valid) {
                _basic.model = 1;
                _basic.sn = QStringLiteral("ZSL1-MATRiX-SIM");
                _basic.devName = QStringLiteral("ZSL1-SIM");
                _basic.temperature = 25.0f;
                _basic.power = 100;
                _basic.valid = true;
            }
            // feedback：默认 SDK 模式 + 移动模式
            if (!_feedback.valid) {
                _feedback.functionMode = 3;  // sdk
                _feedback.controlMode = 1;   // stand_up
                _feedback.motionMode = 2;    // rest
                _feedback.speedLevel = 2;    // normal
                _feedback.valid = true;
            }
        } else {
            qWarning() << "[MatrixDog] Zenoh 状态链路断开（末包距今" << sinceLast << "ms）";
        }
    }
}

// ---- 指令下发 ----
void MatrixDogClient::sendCmd(quint32 cmdCode)
{
    // L1 命令码 → MatrixMotionControl 动作
    // 参考: Tools/send_gamepad_udp.py 的 ACTIONS
    QString action;
    switch (cmdCode) {
    case 0x7A:  // 起身站立
    case 0x9A:  // 平衡站立
        action = QStringLiteral("stand");
        break;
    case 0x6A:  // 趴下/关节保持
    case 0x9B:  // 锁定
        action = QStringLiteral("jointlock");
        break;
    case 0x5A:  // 被动/急停
        action = QStringLiteral("passive");
        break;
    case 0x8A:  // 移动模式
    case 0xAA:  // 遥控模式
        action = QStringLiteral("walk");
        break;
    case 0xB3:  // CMD_SDK → MATRiX inside_mc=true 时自动 SDK 模式
        qInfo() << "[MatrixDog] L1 cmd=0xB3 (CMD_SDK) → MATRiX 自动 SDK 模式，忽略";
        return;
    default:
        qWarning() << "[MatrixDog] 未知 L1 命令码:" << Qt::hex << cmdCode;
        return;
    }

    qInfo() << "[MatrixDog] L1 cmd=0x" << Qt::hex << cmdCode << "→ action" << action;
    _currentAction = action;
    _hasMoveCmd = false;
    sendAction(action);
}

void MatrixDogClient::sendRemote(float forward, float turn, float shift, float head)
{
    Q_UNUSED(head);  // MATRiX ZSL1 无头部，忽略

    // 【实测定标】action 协议下 forward/yaw 与 rt/odom 同属 ROS 右手系，直传：
    //   forward 正 = odom +x；yaw 正 = odom yaw 增大。
    // 有速度指令即切 walk（stand/jointlock 状态下摇杆分量不生效）。
    const bool hasSpeed = (std::abs(forward) > 0.01f || std::abs(turn) > 0.01f || std::abs(shift) > 0.01f);
    if (hasSpeed) {
        _currentAction = QStringLiteral("walk");
    }

    _currentForward = std::clamp(forward, -1.0f, 1.0f);
    _currentLateral = std::clamp(shift, -1.0f, 1.0f);
    _currentYaw     = std::clamp(turn, -1.0f, 1.0f);
    _hasMoveCmd = true;

    // 立即发一帧，避免等到下一个 30Hz 周期
    sendAction(_currentAction, _currentForward, _currentLateral, _currentYaw);
}

void MatrixDogClient::sendZeroRemote()
{
    _currentForward = 0.0f;
    _currentLateral = 0.0f;
    _currentYaw = 0.0f;
    _hasMoveCmd = false;
    // 保持 walk action，只把速度置零。
    // 实测切 stand/jointlock 会触发 MATRiX 的站姿切换动画（约 1~2 秒），
    // 造成"到达一个点停一会再往下走"。walk forward=0 一帧内静止，无过渡。
    // 下一个导航任务的 walk forward=xxx 直接覆盖，没有 action 切换延迟。
    _currentAction = QStringLiteral("walk");
    sendAction(_currentAction);
}

// ---- UDP 发送（MatrixMotionControl 协议）----
// 参考: Tools/send_gamepad_udp.py（官方 30Hz 持续发送）
// 格式: {"version":1,"action":"walk","forward":0.3,"lateral":0.0,"yaw":0.0}
void MatrixDogClient::sendAction(const QString &action, double forward, double lateral, double yaw)
{
    // 按官方 send_gamepad_udp.py 的原始字段顺序手工拼接：
    // QJsonObject 会把键按字母序重排（action/forward/lateral/version/yaw），
    // 与官方的 version/action/forward/lateral/yaw 不一致，这里保持原序。
    const QString payload = QStringLiteral(
        "{\"version\":1,\"action\":\"%1\",\"forward\":%2,\"lateral\":%3,\"yaw\":%4}")
        .arg(action)
        .arg(QString::number(forward, 'g', 10))
        .arg(QString::number(lateral, 'g', 10))
        .arg(QString::number(yaw, 'g', 10));
    const QByteArray data = payload.toUtf8();
    _socket->writeDatagram(data, QHostAddress(_dogIp), _sendPort);

    // 30Hz 全量打印会产出上万行日志，这里限流到 500ms 一条
    if (action != QStringLiteral("stand")) {
        const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
        if (nowMs - _lastCmdLogMs >= 500) {
            _lastCmdLogMs = nowMs;
            qDebug() << "[MatrixDog] sendAction:" << data;
        }
    }
}
