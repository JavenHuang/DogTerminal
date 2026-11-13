#include "SdkDogClient.h"

#include <QDebug>
#include <QList>

#include <array>
#include <cmath>
#include <exception>

using zsibot::float32_t;

namespace
{
constexpr int kPollMs = 50;   // 20Hz 轮询，与 UDP 版状态刷新节奏接近
}

SdkDogClient::SdkDogClient(QObject *parent)
    : DogClientBase(parent)
    , _pollTimer(nullptr)
    , _sendPort(8081)
    , _recvPort(8080)
    , _role(QStringLiteral("sdk"))
    , _heartbeatMs(50)
    , _timeoutMs(5000)
    , _running(false)
    , _connected(false)
{
}

SdkDogClient::~SdkDogClient()
{
    stop();
}

void SdkDogClient::configure(const QString &dogIp, quint16 sendPort, quint16 recvPort,
                             const QString &role, int heartbeatMs, int timeoutMs)
{
    _ip = dogIp;
    _sendPort = sendPort;
    _recvPort = recvPort;
    _role = role;
    // 记录用：SDK 库内部固定 50ms 心跳 / 5s 超时，无法通过参数调整
    _heartbeatMs = heartbeatMs;
    _timeoutMs = timeoutMs;
}

void SdkDogClient::start()
{
    if (_running)
        return;

    const zsibot::Role role = _role.contains(QStringLiteral("sdk"), Qt::CaseInsensitive)
                                  ? zsibot::Role::ROLE_SDK
                                  : zsibot::Role::ROLE_REMOTE;
    try
    {
        // SDK 构造即启动内部三线程；该类不可拷贝/移动赋值，用 unique_ptr 持有
        _exec = std::make_unique<zsibot::ZsibotExecutor>(
            role, _ip.toStdString(),
            static_cast<uint32_t>(_sendPort), static_cast<uint32_t>(_recvPort));
    }
    catch (const std::exception &e)
    {
        qWarning() << "[DogLink][SDK] 初始化失败:" << e.what();
        _exec.reset();
        return;
    }

    if (!_pollTimer)
    {
        _pollTimer = new QTimer(this);
        _pollTimer->setInterval(kPollMs);
        connect(_pollTimer, &QTimer::timeout, this, &SdkDogClient::onPollTimer);
    }
    _running = true;
    _pollTimer->start(kPollMs);

    qInfo() << "[DogLink] 已启动(SDK), 狗地址:" << _ip
            << "控制口:" << _sendPort << "状态口:" << _recvPort
            << "(心跳/超时由 SDK 内部固定 50ms/5s)";
}

void SdkDogClient::stop()
{
    if (!_running)
        return;
    _running = false;

    if (_pollTimer)
        _pollTimer->stop();

    if (_exec)
    {
        // 退出前零速，避免狗保持最后一次遥控量
        _exec->SetRemote(std::array<float32_t, 4>{0.0f, 0.0f, 0.0f, 0.0f},
                         std::array<float32_t, 14>{0.0f});
        _exec.reset();   // 析构时 SDK 内部线程随之结束
    }
    if (_connected)
    {
        _connected = false;
        emit connectedChanged(false);
    }
    qInfo() << "[DogLink] 已停止(SDK)";
}

void SdkDogClient::sendCmd(quint32 cmdCode)
{
    if (!_exec)
        return;
    _exec->SetCmd(static_cast<zsibot::CmdCode>(cmdCode));
}

void SdkDogClient::sendRemote(float forward, float turn, float shift, float head)
{
    if (!_exec)
        return;
    _exec->SetRemote(std::array<float32_t, 4>{forward, turn, shift, head},
                     std::array<float32_t, 14>{0.0f});
}

void SdkDogClient::sendZeroRemote()
{
    sendRemote(0.0f, 0.0f, 0.0f, 0.0f);
}

void SdkDogClient::onPollTimer()
{
    refresh();
}

void SdkDogClient::refresh()
{
    if (!_exec)
        return;

    const bool conn = _exec->IsConnected();

    // ---- IMU ----
    const std::array<float32_t, 3> rpy = _exec->GetRPY();        // rad
    const std::array<float32_t, 3> gyro = _exec->GetBodyGyro();  // rad/s
    const std::array<float32_t, 3> acc = _exec->GetBodyAcc();    // m/s2
    const std::array<float32_t, 4> quat = _exec->GetQuaternion();
    for (int i = 0; i < 3; ++i)
    {
        _imu.rpy[i] = rpy[i];
        _imu.gyro[i] = gyro[i];
        _imu.acc[i] = acc[i];
    }
    _imu.gyroField = true;   // SDK 驱动直接取 GetBodyGyro，必带角速度
    for (int i = 0; i < 4; ++i)
        _imu.quat[i] = quat[i];
    _imu.valid = conn;

    // ---- 里程计 ----
    const std::array<float32_t, 3> pos = _exec->GetPosition();
    const std::array<float32_t, 3> vBody = _exec->GetBodyVelocity();
    const std::array<float32_t, 3> vWorld = _exec->GetWorldVelocity();
    for (int i = 0; i < 3; ++i)
    {
        _odom.position[i] = pos[i];
        _odom.vBody[i] = vBody[i];
        _odom.vWorld[i] = vWorld[i];
        _odom.omegaBody[i] = gyro[i];
        // TODO: verify SDK 未提供世界系角速度，暂以机体角速度近似
        _odom.omegaWorld[i] = gyro[i];
    }
    _odom.valid = conn;

    // ---- 模式反馈 ----
    _feedback.functionMode = static_cast<int>(_exec->GetFunctionMode());
    _feedback.controlMode = static_cast<int>(_exec->GetControlMode());
    _feedback.motionMode = static_cast<int>(_exec->GetMotionMode());
    _feedback.motionType = static_cast<int>(_exec->GetMotionType());
    _feedback.speedLevel = static_cast<int>(_exec->GetSpeedLevel());
    _feedback.valid = conn;

    // ---- 电池 ----
    const zsibot::BatteryInfo bat = _exec->GetBatteryInfo();
    _battery.current = bat.current;
    _battery.error = static_cast<int>(bat.error);
    _battery.power = static_cast<int>(bat.power);
    _battery.temp = bat.temp;
    _battery.volt = bat.volt;
    if (_battery.volt > 1000.0f)   // TODO: verify 真狗上报 mV（41668 -> 41.668V）
        _battery.volt /= 1000.0f;
    _battery.valid = conn;

    // ---- 基础信息 ----
    const zsibot::SpeedInfo speed = _exec->GetSpeed();
    _basic.power = _exec->GetPower();
    _basic.temperature = _exec->GetTemperature();
    _basic.speed = speed.speed;
    _basic.sn = QString::fromStdString(_exec->GetSn());
    _basic.devName = QString::fromStdString(_exec->GetDevName());
    _basic.ssid = QString::fromStdString(_exec->GetWifiSsid());
    _basic.model = static_cast<int>(_exec->GetModel());
    _basic.valid = conn;

    // ---- 故障 ----
    _faults.clear();
    const std::vector<zsibot::FaultInfo> rawFaults = _exec->GetFaultInfo();
    for (const zsibot::FaultInfo &f : rawFaults)
    {
        DogFault d;
        d.module = QString::fromStdString(f.module);
        d.submodule = QString::fromStdString(f.submodule);
        d.errorCode = f.error_code;
        d.level = QString::fromStdString(f.level);
        d.info = QString::fromStdString(f.info);
        _faults.append(d);
    }

    if (_connected != conn)
    {
        _connected = conn;
        emit connectedChanged(conn);
    }
    emit stateDataChanged();
}
