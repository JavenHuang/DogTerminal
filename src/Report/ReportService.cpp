// ============================================================
// 上行上报服务 —— 实现
//
// 三条上行链路各自组装报文：状态、心跳、任务反馈。只有状态报文依赖位置/朝向，
// 另两条只带状态机字段，因此不强行共用打包代码。
// 位置来源统一为 YawStabilizer::yaw() 与 RtkPoseTracker 的融合结果，不直接读 imu.rpy。
// ============================================================
#include "Report/ReportService.h"

#include <QDataStream>
#include <QIODevice>
#include <QDebug>

#include "Common/DogText.h"
#include "Config/TerminalConfig.h"
#include "Control/Navigator.h"
#include "Control/YawStabilizer.h"
#include "Convert/MapTransform.h"
#include "Convert/RtkPoseTracker.h"
#include "DogLink/DogClientBase.h"
#include "ServerLink/MqLink.h"
#include "Protocol/DogProtocol.h"

ReportService::ReportService(const TerminalConfig &cfg, MqLink *&mq, MapTransform &map,
                             YawStabilizer &yaw, RtkPoseTracker &rtk,
                             DogProtocol::TerminalStatus &status, DogClientBase *&dog,
                             Navigator &navigator, bool &controlAcquired, bool &velCtrlActive)
    : _cfg(cfg), _mq(mq), _map(map), _yaw(yaw), _rtk(rtk), _status(status),
      _dog(dog), _navigator(navigator), _controlAcquired(controlAcquired),
      _velCtrlActive(velCtrlActive)
{
}

// 终端状态枚举 → 可读文本（日志用）
static QString terminalStatusText(int s)
{
    switch (s)
    {
    case DogProtocol::TS_INIT:           return QStringLiteral("初始化");
    case DogProtocol::TS_CONNECTING_DOG: return QStringLiteral("连接机器狗中");
    case DogProtocol::TS_DOG_CONNECTED:  return QStringLiteral("已连接机器狗(未取得控制权)");
    case DogProtocol::TS_READY:          return QStringLiteral("就绪");
    case DogProtocol::TS_EXECUTING:      return QStringLiteral("任务执行中");
    case DogProtocol::TS_FAULT:          return QStringLiteral("故障");
    default:                             return QStringLiteral("未知");
    }
}

void ReportService::publishState()
{
    update();
    DogProtocol::StateReport report;
    report.state = buildDogState();

    QByteArray body;
    QDataStream out(&body, QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_5_12);
    out << report;

    DogProtocol::MessageHeader header(DogProtocol::MSG_STATE_REPORT, _cfg.terminalId);
    _mq->publish(routingKeyState(_cfg.terminalId), DogProtocol::packMessage(header, body));
}

void ReportService::publishHeartbeat()
{
    DogProtocol::HeartbeatMsg hb;
    hb.status = _status;

    QByteArray body;
    QDataStream out(&body, QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_5_12);
    out << hb;

    DogProtocol::MessageHeader header(DogProtocol::MSG_HEARTBEAT, _cfg.terminalId);
    _mq->publish(routingKeyHeartbeat(_cfg.terminalId), DogProtocol::packMessage(header, body));
}

void ReportService::publishFeedback(quint32 taskId, DogProtocol::MoveResult result,
                                     const QString &message)
{
    DogProtocol::TaskFeedback fb;
    fb.taskId = taskId;
    fb.result = result;
    fb.message = message;
    DogProtocol::DogState st = buildDogState();
    fb.currentX = st.x;
    fb.currentY = st.y;

    QByteArray body;
    QDataStream out(&body, QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_5_12);
    out << fb;

    DogProtocol::MessageHeader header(DogProtocol::MSG_TASK_FEEDBACK, _cfg.terminalId);
    _mq->publish(routingKeyFeedback(_cfg.terminalId), DogProtocol::packMessage(header, body));
}

DogProtocol::DogState ReportService::buildDogState() const
{
    DogProtocol::DogState st;
    st.dogConnected = _dog->isConnected();
    st.controlAcquired = _controlAcquired;
    st.status = _status;
    // 地图坐标启用且已建立参考时上报地图坐标，否则上报原始局部坐标
    st.calibrated = _map.isActive();

    if (st.dogConnected)
    {
        const DogOdom &odom = _dog->odom();
        const DogImu &imu = _dog->imu();
        const DogFeedback &fb = _dog->feedback();
        const DogBattery &bat = _dog->battery();
        const DogBasic &basic = _dog->basic();

        if (st.calibrated)
        {
            // 上报坐标与朝向同样取融合位姿（差分锚点 + 里程计增量）；
            // 融合尚未建立时退回纯里程计推出的地图位姿
            double fmx = 0.0;
            double fmy = 0.0;
            double fmyaw = 0.0;
            int fsrc = RtkFusion::SrcNone;
            if (_rtk.fusedMapPose(&fmx, &fmy, &fmyaw, &fsrc))
            {
                st.x = fmx;
                st.y = fmy;
                st.yaw = fmyaw;
            }
            else
            {
                _map.toMap(odom.position[0], odom.position[1], _yaw.yaw(),
                           &st.x, &st.y, &st.yaw);
            }
            st.z = odom.position[2];
        }
        else
        {
            st.x = odom.position[0];
            st.y = odom.position[1];
            st.z = odom.position[2];
            st.yaw = _yaw.yaw();
        }
        // 原始局部坐标永远上报（服务端用于地图/局部叠加对比）。
        // 已建立参考时上报"相对参考点的局部位移"，与地图 O 点对齐，
        // 服务端两套轨迹才能同框叠加；未建立参考时上报原始值。
        if (_map.isActive())
        {
            st.localX = odom.position[0] - _map.refLocalX();
            st.localY = odom.position[1] - _map.refLocalY();
        }
        else
        {
            st.localX = odom.position[0];
            st.localY = odom.position[1];
        }
        st.localYaw = _yaw.yaw();
        st.roll = imu.rpy[0];
        st.pitch = imu.rpy[1];

        // 字段名沿用既有协议，数值为狗原始局部参考系速度
        st.vWorldX = odom.vWorld[0];
        st.vWorldY = odom.vWorld[1];
        st.vWorldZ = odom.vWorld[2];
        st.yawRate = odom.omegaWorld[2];
        st.bodyVx = odom.vBody[0];
        st.bodyVy = odom.vBody[1];
        st.bodyVz = odom.vBody[2];

        if (bat.valid)
        {
            st.batteryPercent = bat.power;
            st.batteryVolt = bat.volt;
        }
        else if (basic.valid)
        {
            st.batteryPercent = static_cast<qint32>(basic.power);
        }
        st.temperature = basic.temperature;

        st.functionMode = static_cast<quint8>(qMax(0, fb.functionMode));
        st.controlMode = static_cast<quint8>(qMax(0, fb.controlMode));
        st.speedLevel = static_cast<quint8>(qMax(0, fb.speedLevel));
        st.faultCount = static_cast<quint8>(qMin(255, _dog->faults().size()));
    }
    return st;
}

void ReportService::update()
{
    DogProtocol::TerminalStatus s;
    if (!_dog->isConnected())
        s = DogProtocol::TS_CONNECTING_DOG;
    else if (!_controlAcquired)
        s = DogProtocol::TS_DOG_CONNECTED;
    else if (_navigator.isActive() || _velCtrlActive)
        s = DogProtocol::TS_EXECUTING;
    else
        s = DogProtocol::TS_READY;

    if (s != _status)
    {
        _status = s;
        qInfo() << "[Main] 终端状态:" << terminalStatusText(_status);
    }
}

void ReportService::publishTaskResult(quint32 taskId, DogProtocol::MoveResult result,
                                       double remainingDist, const QString &message)
{
    DogProtocol::TaskFeedback fb;
    fb.taskId = taskId;
    fb.result = result;
    fb.remainingDist = remainingDist;
    fb.message = message;
    DogProtocol::DogState st = buildDogState();
    fb.currentX = st.x;
    fb.currentY = st.y;
    publishState();  // 顺带刷新最新位置

    QByteArray body;
    QDataStream out(&body, QIODevice::WriteOnly);
    out.setVersion(QDataStream::Qt_5_12);
    out << fb;
    DogProtocol::MessageHeader header(DogProtocol::MSG_TASK_FEEDBACK, _cfg.terminalId);
    _mq->publish(routingKeyFeedback(_cfg.terminalId), DogProtocol::packMessage(header, body));

    update();
}
