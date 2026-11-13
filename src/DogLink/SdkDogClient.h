#ifndef SDK_DOG_CLIENT_H
#define SDK_DOG_CLIENT_H

#include <memory>
#include <QTimer>
#include <QString>

#include "DogClientBase.h"
#include "zsibot_sdk/zsibot_api.h"

// ============================================================
// 机器狗官方 SDK 客户端（南向链路，仅 Linux）
// 依赖 genisom_L1_sdk-main 的 lib/<arch>/libzsibot.a
//   - SDK 构造即启动内部三线程（发送 / 接收 / 心跳 50ms）
//   - SDK 只提供拉取式 getter，故用 20Hz 定时器刷新缓存后发 stateDataChanged()
//   - 心跳/连接超时由库内固定（50ms / 5s），configure 的对应参数仅作记录
// 与 UdpDogClient 实现同一 DogClientBase 接口，上层无感切换。
// ============================================================
class SdkDogClient : public DogClientBase
{
    Q_OBJECT
public:
    explicit SdkDogClient(QObject *parent = nullptr);
    ~SdkDogClient() override;

    void configure(const QString &dogIp, quint16 sendPort, quint16 recvPort,
                   const QString &role, int heartbeatMs, int timeoutMs) override;

    void start() override;
    void stop() override;

    bool isRunning() const override { return _running; }
    bool isConnected() const override { return _connected; }

    const DogImu &imu() const override { return _imu; }
    const DogOdom &odom() const override { return _odom; }
    const DogFeedback &feedback() const override { return _feedback; }
    const DogBattery &battery() const override { return _battery; }
    const DogBasic &basic() const override { return _basic; }
    QList<DogFault> faults() const override { return _faults; }

    void sendCmd(quint32 cmdCode) override;
    void sendRemote(float forward, float turn, float shift, float head) override;
    void sendZeroRemote() override;

private slots:
    void onPollTimer();   // 20Hz 轮询 SDK getter，刷新缓存

private:
    void refresh();

    std::unique_ptr<zsibot::ZsibotExecutor> _exec;
    QTimer *_pollTimer;

    QString _ip;
    quint16 _sendPort;
    quint16 _recvPort;
    QString _role;
    int _heartbeatMs;
    int _timeoutMs;

    bool _running;
    bool _connected;

    DogImu _imu;
    DogOdom _odom;
    DogFeedback _feedback;
    DogBattery _battery;
    DogBasic _basic;
    QList<DogFault> _faults;
};

#endif  // SDK_DOG_CLIENT_H
