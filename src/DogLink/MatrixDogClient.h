#ifndef MATRIX_DOG_CLIENT_H
#define MATRIX_DOG_CLIENT_H

#include "DogClientBase.h"
#include "DogTypes.h"

#include <QObject>
#include <QJsonObject>
#include <QUdpSocket>
#include <QTimer>
#include <QString>
#include <atomic>
#include <memory>
#include <thread>
#include <functional>

// zenoh-cxx（Zenoh C++ 客户端 v1.x）
// 安装：sudo apt install libzenoh-cxx-dev  或  从源码构建
#include <zenoh.hxx>

// ============================================================
// MATRiX 仿真驱动（ZSL1/ZSL2）
// 下行：UDP 7447，摇杆 JSON + 按键脉冲
// 上行：Zenoh rt/**，CDR 格式（ROS 2 XCDR1）
//
// 线程模型：
//   - 主线程 (Qt)：QUdpSocket 发送、QTimer 死手保护、缓存更新、发 stateDataChanged
//   - Zenoh 线程池：订阅回调在 Zenoh 内部线程触发
//     → 用 QMetaObject::invokeMethod(..., Qt::QueuedConnection) 跨线程回到主线程
// ============================================================
class MatrixDogClient : public DogClientBase
{
    Q_OBJECT
public:
    explicit MatrixDogClient(QObject *parent = nullptr);
    ~MatrixDogClient() override;

    // ---- DogClientBase 接口实现 ----
    void configure(const QString &dogIp, quint16 sendPort, quint16 recvPort,
                   const QString &role, int heartbeatMs, int timeoutMs) override;
    void start() override;
    void stop() override;
    bool isRunning() const override;
    bool isConnected() const override;

    const DogImu &imu() const override { return _imu; }
    const DogOdom &odom() const override { return _odom; }
    const DogFeedback &feedback() const override { return _feedback; }
    const DogBattery &battery() const override { return _battery; }
    const DogBasic &basic() const override { return _basic; }
    QList<DogFault> faults() const override { return _faults; }

    void sendCmd(quint32 cmdCode) override;
    void sendRemote(float forward, float turn, float shift, float head) override;
    void sendZeroRemote() override;

    // ---- 额外配置（configure 之后调用，MainController 里工厂负责设置）----
    void setZenohEndpoint(const QString &ep) { _zenohEndpoint = ep; }
    void setZenohMode(const QString &mode)   { _zenohMode = mode; }
    void setTopicPrefix(const QString &p)    { _topicPrefix = p; }
    void setImuTopic(const QString &t)       { _imuTopic = t; }
    void setOdomTopic(const QString &t)      { _odomTopic = t; }

signals:
    // 从 Zenoh 线程转发到 Qt 线程的原始信号
    void imuReady();
    void odomReady();
    void heartbeatReceived();

private slots:
    void onImuReady();
    void onOdomReady();
    void onDeadmanTimer();    // 持续发送 neutral / 摇杆（Matrix_MC 要求 20Hz 持续输入）
    void onCheckTimeout();    // 断连检测（Zenoh 超时）

private:
    // ---- Zenoh ----
    void startZenoh();
    void stopZenoh();
    void onZenohImu(const zenoh::Sample &sample);
    void onZenohOdom(const zenoh::Sample &sample);

    // ---- UDP 下行（MatrixMotionControl 协议，官方 send_gamepad_udp.py）----
    // 格式: {"version":1,"action":"walk","forward":0.3,"lateral":0.0,"yaw":0.0}
    // action: passive/stand/walk/ampwalk/labwalk/fastterrain/jointpd/jointlock/jointfree/jump/front_jump
    // 官方以 30Hz 持续发送；实测该协议是唯一被 MATRiX 执行的输入方式
    // （17 字段手柄协议实测零响应，狗位姿与 yaw 完全不变）。
    void sendAction(const QString &action, double forward = 0.0, double lateral = 0.0, double yaw = 0.0);

    // ---- CDR 解析 ----
    struct RawPose {
        float position[3];
        float quatXyzw[4];
        float vWorld[3];
        float omegaBody[3];
    };
    struct RawImu {
        float quatXyzw[4];
        float gyroBody[3];
        float accBody[3];   // IMU 里加速度，ZSL1 无单独 acc 接口时复用
    };
    static bool parseImuCdr(const QByteArray &data, RawImu &out);
    static bool parseOdomCdr(const QByteArray &data, RawPose &out);
    static void quatToRpy(float xyzw[4], float rpy[3]);

    // ---- 成员 ----
    QUdpSocket *_socket;
    QTimer *_deadmanTimer;   // 20Hz 控制持续发送
    QTimer *_checkTimer;     // 断连检测 500ms
    QThread *_zenohThread;   // 运行 Zenoh open() 和 subscriber 的线程

    // Zenoh 资源（用 void* 绕过 Subscriber 模板类型，stopZenoh 里 static_cast）
    std::shared_ptr<zenoh::Session> _session;
    void *_imuSub = nullptr;   // 实际是 zenoh::Subscriber<lambda_type>
    void *_odomSub = nullptr;  // 实际是 zenoh::Subscriber<lambda_type>

    // 配置
    QString _dogIp;
    quint16 _sendPort = 7447;
    quint16 _recvPort = 7447;   // Zenoh 监听端口（TCP）
    QString _zenohEndpoint = QStringLiteral("tcp/127.0.0.1:7447");
    QString _zenohMode = QStringLiteral("client");
    QString _topicPrefix = QStringLiteral("rt");
    QString _imuTopic = QStringLiteral("/imu");
    QString _odomTopic = QStringLiteral("/odom");

    // 运行状态
    bool _running = false;
    std::atomic<bool> _connected{false};   // UDP 控制链路就绪（start() 即 true）
    std::atomic<bool> _everGotImu{false};
    std::atomic<bool> _everGotOdom{false};
    std::atomic<qint64> _lastRecvMs{0};
    bool _stateLinkOk = false;             // Zenoh 状态链连通（不影响控制指令）

    // 当前动作状态（供死手定时器 30Hz 持续发送）
    QString _currentAction = QStringLiteral("stand");
    float _currentForward = 0.0f;
    float _currentLateral = 0.0f;
    float _currentYaw = 0.0f;
    bool _hasMoveCmd = false;
    qint64 _lastCmdLogMs = 0;   // 指令日志节流（30Hz 全量打印会让日志爆炸）

    // 位置差分测速：部分 MATRiX 版本 rt/odom 的 twist 恒为 0，
    // 只能用位置差分兜底，否则 Navigator 速度闭环反馈失效
    float _lastPos[3] = {0.0f, 0.0f, 0.0f};
    qint64 _lastOdomMs = 0;
    bool _haveLastPos = false;

    // 状态缓存
    DogImu _imu;
    DogOdom _odom;
    DogFeedback _feedback;
    DogBattery _battery;
    DogBasic _basic;
    QList<DogFault> _faults;

    static constexpr int kDeadmanMs = 33;        // 30Hz，对齐官方 send_gamepad_udp.py
    static constexpr int kCheckMs = 500;
    static constexpr int kTimeoutMs = 5000;
};

#endif  // MATRIX_DOG_CLIENT_H
