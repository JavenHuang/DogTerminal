#ifndef DOG_CLIENT_BASE_H
#define DOG_CLIENT_BASE_H

#include <QObject>
#include "DogTypes.h"

// ============================================================
// 南向链路抽象接口
// 实现类：
//   UdpDogClient —— 自实现 UDP+JSON（Windows / Linux 均可用）
//   SdkDogClient —— 官方 SDK libzsibot（仅 Linux）
// 上层 MainController 只依赖本接口，具体实现由 INI 的 [Dog] Driver 选择。
// ============================================================
class DogClientBase : public QObject
{
    Q_OBJECT
public:
    explicit DogClientBase(QObject *parent = nullptr) : QObject(parent) {}
    ~DogClientBase() override {}

    // 配置网络参数与心跳/超时周期（SDK 实现中心跳/超时为库内固定值）
    virtual void configure(const QString &dogIp, quint16 sendPort, quint16 recvPort,
                           const QString &role, int heartbeatMs, int timeoutMs) = 0;

    // 心跳里声明的角色（与下发指令的角色分开）。空串表示沿用 configure 的 role。
    // 用于避免周期心跳以 sdk 身份持续抢占控制权。
    virtual void setHeartbeatRole(const QString &role) { Q_UNUSED(role); }

    virtual void start() = 0;
    virtual void stop() = 0;

    virtual bool isRunning() const = 0;
    virtual bool isConnected() const = 0;

    virtual const DogImu &imu() const = 0;
    virtual const DogOdom &odom() const = 0;
    virtual const DogFeedback &feedback() const = 0;
    virtual const DogBattery &battery() const = 0;
    virtual const DogBasic &basic() const = 0;
    virtual QList<DogFault> faults() const = 0;

    // ---- 指令下发 ----
    virtual void sendCmd(quint32 cmdCode) = 0;                 // 离散命令码
    virtual void sendRemote(float forward, float turn,
                            float shift, float head) = 0;      // 归一化遥控量 [-1,1]
    virtual void sendZeroRemote() = 0;                         // 零速遥控（停车）

signals:
    void connectedChanged(bool connected);  // 连接状态变化
    void stateDataChanged();                // 任一状态数据更新
};

#endif  // DOG_CLIENT_BASE_H
