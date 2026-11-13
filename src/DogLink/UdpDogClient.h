#ifndef UDP_DOG_CLIENT_H
#define UDP_DOG_CLIENT_H

#include <QObject>
#include <QTimer>
#include <QUdpSocket>
#include <QHostAddress>
#include <QList>
#include <QSet>
#include <QString>

#include "DogClientBase.h"

// ============================================================
// 机器狗 UDP+JSON 客户端（南向链路）
// 等价于官方 SDK ZsibotExecutor 的 Qt 实现，无第三方依赖：
//   - 50ms 心跳维持连接（可配置）
//   - SetCmd / SetRemote 指令下发
//   - 状态接收解析（imu/odom/feedback/battery/fault/dog_state）
//   - 5s 无回包判定断连（可配置）
// ============================================================
class UdpDogClient : public DogClientBase
{
    Q_OBJECT
public:
    explicit UdpDogClient(QObject *parent = nullptr);
    ~UdpDogClient() override;

    // 配置网络参数与心跳/超时周期
    void configure(const QString &dogIp, quint16 sendPort, quint16 recvPort,
                   const QString &role, int heartbeatMs, int timeoutMs) override;

    void setHeartbeatRole(const QString &role) override { _heartbeatRole = role; }

    void start() override;   // 绑定接收端口并启动心跳
    void stop() override;

    bool isRunning() const override { return _running; }
    // 连接条件：收到过至少一个数据包 且 最近心跳在超时时间内
    bool isConnected() const override;

    const DogImu &imu() const override { return _imu; }
    const DogOdom &odom() const override { return _odom; }
    const DogFeedback &feedback() const override { return _feedback; }
    const DogBattery &battery() const override { return _battery; }
    const DogBasic &basic() const override { return _basic; }
    QList<DogFault> faults() const override { return _faults; }

    // ---- 指令下发 ----
    void sendCmd(quint32 cmdCode) override;  // 离散命令码
    // 遥控速度指令：forward 前后(+前)、turn 旋转(+左转)、shift 左右(+左)、head 狗头角度
    // 取值均归一化到 [-1,1]
    void sendRemote(float forward, float turn, float shift, float head) override;
    void sendZeroRemote() override;          // 零速遥控（停车）

private slots:
    void onHeartbeatTimeout();
    void onCheckTimeout();
    void onReadyRead();

private:
    bool processDatagram(const QByteArray &datagram);  // 返回 true=合法狗消息
    void applyArray3(const QJsonValue &v, float out[3]);
    void applyArray4(const QJsonValue &v, float out[4]);
    void setConnected(bool connected);

    QUdpSocket *_socket;
    QTimer *_heartbeatTimer;
    QTimer *_checkTimer;

    QHostAddress _dogIp;
    quint16 _sendPort;
    quint16 _recvPort;
    QString _role;
    QString _heartbeatRole;   // 空串表示沿用 _role
    int _heartbeatMs;
    int _timeoutMs;

    bool _running;
    bool _connected;
    bool _everRecv;
    qint64 _lastRecvMs;

    DogImu _imu;
    DogOdom _odom;
    DogFeedback _feedback;
    DogBattery _battery;
    DogBasic _basic;
    QList<DogFault> _faults;

    // 协议诊断：非 JSON 包计数与已提示过的未知消息类型
    quint64 _badPacketCount = 0;
    QSet<QString> _unknownTypes;
};

#endif  // UDP_DOG_CLIENT_H
