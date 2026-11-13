// ============================================================
// MainController 核心：生命周期、MQ 收发、状态/心跳/看门狗
// 同一类的实现按职责分在多个编译单元：
//   MainController.cpp        装配入口、生命周期、北向收发、消息分发
//   MainControllerSetup.cpp   装配四段（链路/旁路通道/信号/运行时对象）
//   MainControllerNav.cpp     导航槽（纯转调 NavBridge / CommandRouter 逻辑）
//   MainControllerPose.cpp    南向位姿：状态缓存、地图参考、跳变补偿
// 声明见 MainController.h（只放对外接口、槽与少量状态）。
// ============================================================

#include "MainController.h"

#include <QDateTime>
#include <QDataStream>
#include <QDebug>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QJsonParseError>
#include <QSet>
#include <QStringList>
#include <QSettings>
#include <QCoreApplication>
#include <cmath>
#include "Common/DogText.h"
#include "DogLink/ZsibotDefs.h"
#include "DogLink/UdpDogClient.h"
#ifdef DOGTERMINAL_HAS_SDK
#include "DogLink/SdkDogClient.h"
#endif
#ifdef DOGTERMINAL_HAS_MATRIX
#include "DogLink/MatrixDogClient.h"
#endif
#include "ServerLink/DetectionLink.h"
#include "ServerLink/CalibPanel.h"

MainController::MainController(QObject *parent)
    : QObject(parent)
    , _mq(nullptr)
    , _det(nullptr)
    , _detForwarded(0)
    , _detMqDropped(0)
    , _panel(nullptr)
    , _dog(nullptr)
    , _navigator(nullptr)
    , _attitude(nullptr)
    , _yaw(nullptr)
    , _rtk(nullptr)
    , _report(nullptr)
    , _printer(nullptr)
    , _cmd(nullptr)
    , _nav(nullptr)
    , _lastServerMsgMs(0)
    , _serverHeartbeatSeen(false)
    , _running(false)
    , _controlAcquired(false)
    , _velCtrlActive(false)
    , _status(DogProtocol::TS_INIT)
{
}

MainController::~MainController()
{
    delete _yaw;
    delete _report;
    delete _printer;
    delete _nav;
    delete _cmd;
    delete _rtk;   // 非 QObject，需显式释放（其余子对象由 parent 链回收）
}

bool MainController::init(const TerminalConfig &cfg, QString *errMsg)
{
    if (!cfg.validate(errMsg))
        return false;
    _cfg = cfg;

    // 装配分四段：链路 → 旁路通道 → 信号 → 运行时对象（见 MainControllerSetup.cpp）
    initLinks();
    initChannels();
    initSignals();
    initEngines();
    return true;
}

void MainController::start()
{
    if (_running)
        return;
    _running = true;
    _serverHeartbeatSeen = false;
    _lastServerMsgMs = nowMs();

    _dog->start();
    _mq->start();

    // 目标检测 UDP 接收（与 MQ 独立：MQ 未就绪时仍继续收，转发端丢弃并计数）
    if (_det)
    {
        QString derr;
        if (_det->start(&derr))
            qInfo().noquote() << QStringLiteral("[Det] UDP 监听已开启，转发路由键 %1").arg(_detRoutingKey);
        else
            qWarning().noquote() << "[Det] 启动失败（检测转发不可用，其余功能不受影响）:" << derr;
    }

    _stateTimer->start();
    _hbTimer->start();
    _watchdogTimer->start();
    _ctrlTimer->start();

    qInfo() << "[Main] 终端启动 terminalId =" << _cfg.terminalId;
    // 构建标识：现场核对运行的是否为最新二进制
    // V2 = odom 系与 IMU 同轴，地图角 = refYaw - 局部角（无 90° 偏置）
    // YAW-STAB=2 = 陀螺连续传播 + 一致性校验 + 位移方向校核 + 静止锁
    // CTRL-20HZ = 导航推进/遥控下发改由独立控制节拍驱动（与状态包解耦）
    // RTK-FUSE = 差分绝对位姿做锚点、里程计做增量，融合位姿供导航与上报
    // DET-UDP  = YOLO26 检测结果 UDP 接收 + 分片重组 + MQ 原样转发
    qInfo().noquote() << "[Main] 构建标识: MAP-XFORM=V2(odom同轴) 局部坐标=相对参考位移 "
                         "YAW-STAB=2 CTRL-20HZ=1 RTK-FUSE=1 标定分档=精确/粗+mapcheck DET-UDP=1";
}

void MainController::stop()
{
    if (!_running)
        return;
    _running = false;

    _stateTimer->stop();
    _hbTimer->stop();
    _watchdogTimer->stop();
    _velocityTimer->stop();
    _acquireTimer->stop();
    _ctrlTimer->stop();

    // 退出前尽量让狗停下
    exitVelocityMode(true);
    if (_navigator->isActive())
        _navigator->cancel();
    _dog->sendZeroRemote();

    _mq->stop();
    if (_det)
        _det->stop();
    _dog->stop();
    qInfo() << "[Main] 终端停止";
}

// ============================================================
// 北向：RabbitMQ
// ============================================================

void MainController::onMqConnected()
{
    qInfo() << "[Main] 服务器链路就绪";
}

void MainController::onMqDisconnected()
{
    qWarning() << "[Main] 服务器链路断开";
    // 失联保护：取消任务并让狗零速停车
    if (_navigator->isActive())
        cancelActiveTask(DogProtocol::MR_FAILED, QStringLiteral("服务器链路断开"));
    exitVelocityMode(true);
    _report->update();
}

void MainController::onMqError(const QString &what)
{
    qWarning() << "[Main] MQ错误:" << what;
}

// 目标检测帧（DetectionLink 已完成分片重组与校验）→ 原样转发到 MQ
//
// 为什么原样转发、不套 DogProtocol 头：
//   服务端未做改动，它按 45_YOLO26 文档的 robotdog.detection 字段消费；
//   再套一层二进制头反而要服务端改解析。终端归属由路由键 detection.<Id> 表达。

void MainController::onDetectionFrame(const QByteArray &payload, int detectionCount, const QString &peer)
{
    Q_UNUSED(peer)

    if (!_mq->isConnected())
    {
        ++_detMqDropped;
        // 不积压：链路断了就丢，恢复后从新帧继续（检测是实时流，补发无意义）
        if (_detMqDropped == 1 || (_detMqDropped % 200) == 0)
            qWarning().noquote()
                << QStringLiteral("[Det] 服务器链路未就绪，累计丢弃 %1 帧检测结果（链路恢复后自动继续）")
                       .arg(_detMqDropped);
        return;
    }

    _mq->publish(_detRoutingKey, payload, QStringLiteral("application/json"));
    ++_detForwarded;

    if (_detForwarded == 1 || (_detForwarded % 500) == 0)
        qInfo().noquote()
            << QStringLiteral("[Det] 已转发 %1 帧到 MQ（%2，本帧目标 %3 个，%4B）")
                   .arg(_detForwarded).arg(_detRoutingKey).arg(detectionCount).arg(payload.size());
}

void MainController::onMqMessage(const QString &routingKey, const QByteArray &payload)
{
    // 差分定位通道：独立路由键，载荷是定位服务的原始报文（JSON/自有字段），
    // 不带本终端协议头，也不能计入服务器指令看门狗
    if (_cfg.rtkEnabled && !_cfg.rtkRoutingKey.isEmpty()
        && routingKey == _cfg.rtkRoutingKey)
    {
        _rtk->onFrame(payload);
        return;
    }

    _lastServerMsgMs = nowMs();

    DogProtocol::MessageHeader header;
    QByteArray body;
    if (!DogProtocol::unpackMessage(payload, header, body))
    {
        qWarning() << "[Main] 消息解包失败，长度:" << payload.size();
        return;
    }

    // 服务器下行心跳：启用看门狗（仅执行任务期间生效）
    if (header.msgType == DogProtocol::MSG_HEARTBEAT)
        _serverHeartbeatSeen = true;

    dispatchMessage(header.msgType, body);
}

void MainController::dispatchMessage(quint16 msgType, const QByteArray &body)
{
    switch (msgType)
    {
    // 下行心跳在 onMqMessage 中已记录（用于失联看门狗），此处无需再处理
    case DogProtocol::MSG_HEARTBEAT:    break;
    case DogProtocol::MSG_ATTITUDE_CMD:  _cmd->handleAttitudeCmd(body);  break;
    case DogProtocol::MSG_MOVE_CMD:       _cmd->handleMoveCmd(body);            break;
    case DogProtocol::MSG_ROTATE_REL_CMD: _cmd->handleRotateCmd(body, false);   break;
    case DogProtocol::MSG_ROTATE_ABS_CMD: _cmd->handleRotateCmd(body, true);    break;
    case DogProtocol::MSG_MAP_CALIB_CMD:  _cmd->handleMapCalibCmd(body);        break;
    case DogProtocol::MSG_VELOCITY_CMD:   _cmd->handleVelocityCmd(body);        break;
    case DogProtocol::MSG_STOP_CMD:       _cmd->handleStopCmd(body);            break;
    case DogProtocol::MSG_PARAM_CMD:      _cmd->handleParamCmd(body);           break;
    case DogProtocol::MSG_ABS_POSE_CMD:   _rtk->onFrame(body);                break;

    // 保留 ID：旧服务端仍可能下发坐标标定命令，终端已不再处理该语义。
    // 只提示一次，避免每条命令都刷成"未知消息类型"。
    case DogProtocol::MSG_CALIBRATION:
    {
        static bool s_calibWarned = false;
        if (!s_calibWarned)
        {
            s_calibWarned = true;
            qInfo() << "[Main] 收到保留消息 CALIBRATION(0x15)，终端已不再处理坐标标定，后续同类消息静默忽略";
        }
        break;
    }

    default:
    {
        // 未定义类型：每种只告警一次，避免现场日志被刷屏，同时保留发现协议变更的线索
        static QSet<quint16> s_unknownTypes;
        if (!s_unknownTypes.contains(msgType))
        {
            s_unknownTypes.insert(msgType);
            qWarning() << "[Main] 未知消息类型: 0x" << QString::number(msgType, 16);
        }
        break;
    }
    }
}

void MainController::onServerWatchdog()
{
    if (!_running)
        return;
    // 仅当服务器发送过下行心跳时才启用消息级看门狗，
    // 避免对不支持心跳的服务器误判（链路级断连已有 onMqDisconnected 保护）
    if (!_serverHeartbeatSeen)
        return;
    if (!_navigator->isActive() && !_velCtrlActive)
        return;
    if (!_mq->isConnected())
        return;  // 断连场景由 onMqDisconnected 处理

    const qint64 elapsed = nowMs() - _lastServerMsgMs;
    if (elapsed > qint64(_cfg.serverLinkTimeoutSec) * 1000)
    {
        qWarning() << "[Main] 服务器消息超时" << elapsed << "ms，执行失联保护";
        if (_navigator->isActive())
            cancelActiveTask(DogProtocol::MR_FAILED, QStringLiteral("服务器消息超时"));
        exitVelocityMode(true);
        _report->update();
    }
}

// ============================================================
// 消息分发
// ============================================================

void MainController::onStateReportTick()
{
    _report->publishState();
    // 标定面板保活：没有新差分帧时也要刷新"断流多久"，否则服务端会停在上一块面板上
    if (_panel)
        _panel->pollStale(nowMs());
}

void MainController::onHeartbeatTick()
{
    _report->publishHeartbeat();
}
