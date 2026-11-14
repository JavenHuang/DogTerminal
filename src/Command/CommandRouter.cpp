// ============================================================
// 下行指令处理器 —— 实现
//
// 七类指令共用同一条骨架：解包 → 校验链路/控制权/任务冲突 → 驱动执行器 → 回执。
// 校验顺序刻意保持一致：先"狗是否在线"，再"控制权"，最后"是否与正在执行的任务冲突"，
// 这样回执里的拒绝原因总是最贴近现场的那一条。
// 坐标约定：服务器在地图模式下发地图坐标，进入执行器前统一反变换为狗局部坐标。
// ============================================================
#include "Command/CommandRouter.h"

#include <cmath>
#include <QDataStream>
#include <QDebug>

#include "Common/DogText.h"
#include "Config/TerminalConfig.h"
#include "Control/AttitudeExecutor.h"
#include "Control/Navigator.h"
#include "Control/YawStabilizer.h"
#include "Convert/MapTransform.h"
#include "DogLink/DogClientBase.h"
#include "DogLink/ZsibotDefs.h"
#include "Report/ReportService.h"

CommandRouter::CommandRouter(const TerminalConfig &cfg, Navigator &navigator,
                             AttitudeExecutor &attitude, MapTransform &map, YawStabilizer &yaw,
                             DogClientBase *&dog, bool &controlAcquired, bool &velCtrlActive,
                             ReportService &report, double &velFwd, double &velTurn,
                             double &velShift, CommandHost &host)
    : _cfg(cfg), _navigator(navigator), _attitude(attitude), _map(map), _yaw(yaw),
      _dog(dog), _controlAcquired(controlAcquired), _velCtrlActive(velCtrlActive),
      _report(report), _velFwd(velFwd), _velTurn(velTurn), _velShift(velShift), _host(host)
{
}

void CommandRouter::handleAttitudeCmd(const QByteArray &body)
{
    QDataStream in(body);
    in.setVersion(QDataStream::Qt_5_12);
    DogProtocol::AttitudeCmd cmd;
    in >> cmd;
    if (in.status() != QDataStream::Ok)
    {
        qWarning() << "[Main] AttitudeCmd 解析失败";
        return;
    }

    const DogProtocol::AttitudeAction action =
        static_cast<DogProtocol::AttitudeAction>(cmd.action);
    qInfo() << "[Main] 收到姿态命令 action=" << cmd.action;

    // 申请控制权走专用流程
    if (action == DogProtocol::AA_REQUEST_CONTROL)
    {
        _host.beginControlAcquire();
        return;
    }

    // 急停任何时刻都允许
    if (action == DogProtocol::AA_EMERGENCY_STOP)
    {
        if (_navigator.isActive())
            _host.cancelActiveTask(DogProtocol::MR_FAILED, QStringLiteral("急停"));
        _host.exitVelocityMode(false);
        _dog->sendCmd(Zsibot::CMD_EMERGENCY_STOP);
        _report.update();
        return;
    }

    // 任务执行中忽略其余姿态命令，避免相互干扰
    if (_navigator.isActive() || _velCtrlActive)
    {
        const QString reason = QStringLiteral("任务执行中，忽略姿态命令 %1").arg(action);
        qWarning() << "[Main]" << reason;
        _report.publishFeedback(0, DogProtocol::MR_REJECTED, reason);
        return;
    }

    _attitude.execute(action);
}

void CommandRouter::handleMoveCmd(const QByteArray &body)
{
    QDataStream in(body);
    in.setVersion(QDataStream::Qt_5_12);
    DogProtocol::MoveCmd cmd;
    in >> cmd;
    if (in.status() != QDataStream::Ok)
    {
        qWarning() << "[Main] MoveCmd 解析失败";
        return;
    }

    if (!_dog->isConnected())
    {
        _report.publishFeedback(cmd.taskId, DogProtocol::MR_REJECTED, QStringLiteral("机器狗未连接"));
        return;
    }
    if (!_controlAcquired)
    {
        _report.publishFeedback(cmd.taskId, DogProtocol::MR_REJECTED, QStringLiteral("未取得SDK控制权"));
        return;
    }
    double maxSpeed = cmd.maxSpeed;
    if (maxSpeed <= 0)
        maxSpeed = _cfg.navParams.maxLinearSpeed;
    maxSpeed = qMin(maxSpeed, _cfg.navParams.maxLinearSpeed);

    // 到达半径：服务器下发值优先，但终端配置的默认值作为最终兜底——
    // 服务器下发了比默认值更小的值（例如 0.3）时也被提到默认值，
    // 避免因为远端配置滞后导致到达判定过严（狗冲到脸上还不触发）。
    // 若服务器明确下发比默认值更大的半径，仍听从服务器。
    double arriveRadius = qMax(cmd.arriveRadius, _navigator.params().arriveRadiusDefault);

    // 停掉速度直控，切导航
    _host.exitVelocityMode(true);

    // 【无缝切目标】Navigator::start() 只重设 phase/target/params，
    // 不 emit stopRequested/finished，不触发停车。
    // 狗 update 下一拍就按新目标算 headingErr → 输出新方向 → 无缝变向继续走。
    // 这样服务器可以在"快到 A 点时"提前下发任务 B，无需等到达判定 + feedback 往返。
    // 旧任务由下面的 MR_CANCELLED 单独结束，不继承 ARRIVED/FAILED 语义。
    // wasActive/taskId/softArrived 必须在 start() 之前取，否则拿到的是新任务。
    // 地图坐标启用时服务器下发的是地图坐标，需反变换为狗局部坐标
    double targetX = cmd.x;
    double targetY = cmd.y;
    if (_map.isActive())
        _map.toLocal(cmd.x, cmd.y, 0.0, &targetX, &targetY, nullptr);

    const bool wasActive = _navigator.isActive();
    const quint32 prevTaskId = _navigator.taskId();
    const bool prevSoftArrived = _navigator.isSoftArrived();
    _navigator.start(cmd.taskId, targetX, targetY, maxSpeed, arriveRadius, cmd.timeoutMs);

    // 被抢占的旧任务补一条结束事件：狗不停车，所以这条只表示"旧任务不再执行"。
    // 软到达（已发过 MR_ARRIVED）的旧任务不再补发，避免同一任务两次结束。
    if (wasActive && prevTaskId != 0 && prevTaskId != cmd.taskId && !prevSoftArrived)
    {
        _report.publishFeedback(prevTaskId, DogProtocol::MR_CANCELLED,
                                QStringLiteral("被 task %1 抢占").arg(cmd.taskId));
    }

    _report.publishFeedback(cmd.taskId, DogProtocol::MR_ACCEPTED,
                    QStringLiteral("%1目标(%2, %3)")
                        .arg(_map.isActive() ? QStringLiteral("地图") : QStringLiteral("局部"))
                        .arg(cmd.x, 0, 'f', 2)
                        .arg(cmd.y, 0, 'f', 2));
    _report.update();
    qInfo().noquote()
        << QStringLiteral("[Main] 导航启动 task %1%2 %3目标(%4, %5) → 局部(%6, %7)")
               .arg(cmd.taskId)
               .arg(wasActive ? QStringLiteral("(抢占正在执行的任务)") : QString())
               .arg(_map.isActive() ? QStringLiteral("地图") : QStringLiteral("局部"))
               .arg(cmd.x, 0, 'f', 2)
               .arg(cmd.y, 0, 'f', 2)
               .arg(targetX, 0, 'f', 2)
               .arg(targetY, 0, 'f', 2);
}

void CommandRouter::handleRotateCmd(const QByteArray &body, bool absolute)
{
    QDataStream in(body);
    in.setVersion(QDataStream::Qt_5_12);
    DogProtocol::RotateCmd cmd;
    in >> cmd;
    if (in.status() != QDataStream::Ok)
    {
        qWarning() << "[Main] RotateCmd 解析失败";
        return;
    }
    if (!_dog->isConnected())
    {
        _report.publishFeedback(cmd.taskId, DogProtocol::MR_REJECTED, QStringLiteral("机器狗未连接"));
        return;
    }
    if (!_controlAcquired)
    {
        _report.publishFeedback(cmd.taskId, DogProtocol::MR_REJECTED, QStringLiteral("未取得SDK控制权"));
        return;
    }
    const DogImu &imu = _dog->imu();
    if (!imu.valid)
    {
        _report.publishFeedback(cmd.taskId, DogProtocol::MR_REJECTED, QStringLiteral("IMU尚未生效"));
        return;
    }

    // 绝对旋转：地图坐标启用时 angleRad 为地图 yaw，需转为局部 yaw；
    // 相对旋转与坐标系无关，直接透传。
    const double targetAngle = (absolute && _map.isActive())
                                   ? _map.mapYawToLocalYaw(cmd.angleRad)
                                   : cmd.angleRad;

    _host.exitVelocityMode(true);
    const bool wasActive = _navigator.isActive();
    const quint32 prevTaskId = _navigator.taskId();
    const bool prevSoftArrived = _navigator.isSoftArrived();
    _navigator.startRotate(cmd.taskId, _yaw.yaw(), targetAngle, absolute,
                            cmd.maxTurnRate, cmd.toleranceRad, cmd.timeoutMs);

    // 同 handleMoveCmd：被抢占的旧任务补一条 MR_CANCELLED，软到达过的不补发。
    if (wasActive && prevTaskId != 0 && prevTaskId != cmd.taskId && !prevSoftArrived)
    {
        _report.publishFeedback(prevTaskId, DogProtocol::MR_CANCELLED,
                                QStringLiteral("被 task %1 抢占").arg(cmd.taskId));
    }

    const double angleDeg = targetAngle * kRadToDeg;
    _report.publishFeedback(cmd.taskId, DogProtocol::MR_ACCEPTED,
                    QStringLiteral("%1，角度=%2°")
                        .arg(absolute ? (QStringLiteral("绝对") + (_map.isActive()
                                                                     ? QStringLiteral("地图yaw")
                                                                     : QStringLiteral("局部yaw")))
                                      : QStringLiteral("相对旋转"))
                        .arg(angleDeg, 0, 'f', 2));
    _report.update();
    qInfo().noquote()
        << QStringLiteral("[Main] 旋转启动 task %1%2 %3%4° → 局部%5°")
               .arg(cmd.taskId)
               .arg(wasActive ? QStringLiteral("(抢占正在执行的任务)") : QString())
               .arg(absolute ? (_map.isActive() ? QStringLiteral("地图yaw=")
                                                : QStringLiteral("绝对局部yaw="))
                             : QStringLiteral("相对增量="))
               .arg(cmd.angleRad * kRadToDeg, 0, 'f', 2)
               .arg(angleDeg, 0, 'f', 2);
}

void CommandRouter::handleMapCalibCmd(const QByteArray &body)
{
    QDataStream in(body);
    in.setVersion(QDataStream::Qt_5_12);
    DogProtocol::MapCalibCmd cmd;
    in >> cmd;
    if (in.status() != QDataStream::Ok)
    {
        qWarning() << "[Main] MapCalibCmd 解析失败";
        return;
    }

    const DogOdom &odom = _dog->odom();
    const DogImu &imu = _dog->imu();

    switch (cmd.action)
    {
    case DogProtocol::MC_RESET:
    {
        if (!_cfg.mapParams.enabled)
        {
            qWarning().noquote() << "[Map] 参考重建被拒绝: 地图坐标未启用（[Map] Enabled）";
            _report.publishFeedback(cmd.taskId, DogProtocol::MR_REJECTED,
                            QStringLiteral("地图坐标未启用（[Map] Enabled）"));
            return;
        }
        if (!_dog->isConnected() || !odom.valid || !imu.valid)
        {
            qWarning().noquote() << "[Map] 参考重建被拒绝: 位姿尚未就绪"
                                 << "connected=" << _dog->isConnected()
                                 << "odomValid=" << odom.valid
                                 << "imuValid=" << imu.valid;
            _report.publishFeedback(cmd.taskId, DogProtocol::MR_REJECTED,
                            QStringLiteral("位姿尚未就绪，无法建立地图参考"));
            return;
        }
        if (_navigator.isActive() || _velCtrlActive)
        {
            qWarning().noquote() << "[Map] 参考重建被拒绝: 任务执行中";
            _report.publishFeedback(cmd.taskId, DogProtocol::MR_REJECTED,
                            QStringLiteral("任务执行中，无法重建地图参考"));
            return;
        }
        _host.resetMapReference(odom.position[0], odom.position[1], _yaw.yaw());
        qInfo().noquote() << "[Map] 参考已重建（当前点=O点 朝向=0°正东）:" << _map.describe();
        // 参考一建好就有一份可用标定：k=尺度、α=refYaw。之后靠近距离 mapline 定方向即可。
        _host.applyMapCalibToFusion();
        _report.publishFeedback(cmd.taskId, DogProtocol::MR_ACCEPTED,
                        QStringLiteral("地图参考已建立: %1").arg(_map.describe()));
        break;
    }
    case DogProtocol::MC_END_LINE:
    case DogProtocol::MC_CHECK_LINE:
    {
        const bool checkOnly = (cmd.action == DogProtocol::MC_CHECK_LINE);
        if (!_dog->isConnected() || !odom.valid)
        {
            qWarning().noquote() << "[Map] 直线标定被拒绝: 里程计尚未就绪"
                                 << "connected=" << _dog->isConnected()
                                 << "odomValid=" << odom.valid;
            _report.publishFeedback(cmd.taskId, DogProtocol::MR_REJECTED,
                            QStringLiteral("里程计尚未就绪，无法直线标定"));
            return;
        }
        if (!checkOnly && (_navigator.isActive() || _velCtrlActive))
        {
            qWarning().noquote() << "[Map] 直线标定被拒绝: 任务执行中";
            _report.publishFeedback(cmd.taskId, DogProtocol::MR_REJECTED,
                            QStringLiteral("任务执行中，无法直线标定"));
            return;
        }
        if (!_map.isActive())
        {
            qWarning().noquote() << "[Map] 直线标定被拒绝: 地图坐标未启用或参考未建立";
            _report.publishFeedback(cmd.taskId, DogProtocol::MR_REJECTED,
                            QStringLiteral("地图坐标未启用或参考未建立"));
            return;
        }
        QString msg;
        MapTransform::CalibGrade grade = MapTransform::CalibRejected;
        const bool ok = _map.calibrateLineEx(odom.position[0], odom.position[1],
                                             cmd.targetX, cmd.targetY,
                                             !checkOnly, &grade, &msg);
        if (ok)
        {
            if (checkOnly)
            {
                qInfo().noquote() << "[Map] 标定预检:" << msg;
                _report.publishFeedback(cmd.taskId, DogProtocol::MR_ACCEPTED, msg);
            }
            else
            {
                qInfo().noquote() << "[Map] 直线标定成功:" << msg;
                qInfo().noquote() << "[Map] 当前参数:" << _map.describe();
                if (grade == MapTransform::CalibExact)
                    _yaw.saveCalibScale(_map.scale());
                // 方向已由"向东 N 米 + 终点坐标"定死：直接回填融合，不再走停点解算
                _host.applyMapCalibToFusion();
                _report.publishFeedback(cmd.taskId, DogProtocol::MR_ARRIVED, msg);
            }
        }
        else
        {
            qWarning().noquote() << "[Map] 直线标定失败:" << msg;
            _report.publishFeedback(cmd.taskId, DogProtocol::MR_FAILED, msg);
        }
        break;
    }
    case DogProtocol::MC_DISABLE:
        _map.clearReference();
        qInfo() << "[Map] 地图坐标变换已关闭，回到原始局部坐标";
        _report.publishFeedback(cmd.taskId, DogProtocol::MR_ACCEPTED,
                        QStringLiteral("地图坐标变换已关闭"));
        break;
    default:
        qWarning() << "[Main] 未知地图标定动作:" << cmd.action;
        _report.publishFeedback(cmd.taskId, DogProtocol::MR_REJECTED,
                        QStringLiteral("未知地图标定动作"));
        break;
    }
}

void CommandRouter::handleVelocityCmd(const QByteArray &body)
{
    QDataStream in(body);
    in.setVersion(QDataStream::Qt_5_12);
    DogProtocol::VelocityCmd cmd;
    in >> cmd;
    if (in.status() != QDataStream::Ok)
    {
        qWarning() << "[Main] VelocityCmd 解析失败";
        return;
    }

    // 导航执行中不接受速度直控
    if (_navigator.isActive())
    {
        const QString reason = QStringLiteral("导航执行中，忽略速度直控");
        qWarning() << "[Main]" << reason;
        _report.publishFeedback(0, DogProtocol::MR_REJECTED, reason);
        return;
    }
    if (!_dog->isConnected() || !_controlAcquired)
    {
        const QString reason = QStringLiteral("狗未就绪，忽略速度直控");
        qWarning() << "[Main]" << reason;
        _report.publishFeedback(0, DogProtocol::MR_REJECTED, reason);
        return;
    }

    // 机体系速度 → 归一化遥控量
    double fwd = cmd.vx / _cfg.navParams.joyLinearScale;
    double turn = cmd.yawRate / _cfg.navParams.joyTurnScale;
    double shift = cmd.vy / _cfg.navParams.joyLinearScale;
    fwd = qBound(-1.0, fwd, 1.0);
    turn = qBound(-1.0, turn, 1.0);
    shift = qBound(-1.0, shift, 1.0);

    // 缓存最近一次直控量：由 20Hz 控制节拍续发，避免服务器低频下发时摇杆量断流
    _velFwd   = fwd;
    _velTurn  = turn;
    _velShift = shift;

    _host.enterVelocityMode();
    _dog->sendRemote(static_cast<float>(fwd), static_cast<float>(turn),
                     static_cast<float>(shift), 0.0f);
}

void CommandRouter::handleStopCmd(const QByteArray &body)
{
    QDataStream in(body);
    in.setVersion(QDataStream::Qt_5_12);
    DogProtocol::StopCmd cmd;
    in >> cmd;
    if (in.status() != QDataStream::Ok)
    {
        qWarning() << "[Main] StopCmd 解析失败";
        return;
    }

    if (_navigator.isActive())
        _host.cancelActiveTask(DogProtocol::MR_FAILED, QStringLiteral("服务器停止指令"));

    if (cmd.emergency)
    {
        _host.exitVelocityMode(false);
        _dog->sendCmd(Zsibot::CMD_EMERGENCY_STOP);
    }
    else
    {
        _host.exitVelocityMode(true);
    }
    _report.update();
}

void CommandRouter::handleParamCmd(const QByteArray &body)
{
    QDataStream in(body);
    in.setVersion(QDataStream::Qt_5_12);
    DogProtocol::ParamCmd cmd;
    in >> cmd;
    if (in.status() != QDataStream::Ok)
    {
        qWarning() << "[Main] ParamCmd 解析失败";
        return;
    }

    quint32 code = Zsibot::CMD_NULL;
    switch (cmd.speedLevel)
    {
    case DogProtocol::SL_SLOW:   code = Zsibot::CMD_SLOW_SPEED;   break;
    case DogProtocol::SL_NORMAL: code = Zsibot::CMD_NORMAL_SPEED; break;
    case DogProtocol::SL_FAST:   code = Zsibot::CMD_FAST_SPEED;   break;
    default:
        qWarning() << "[Main] 无效速度档位:" << cmd.speedLevel;
        return;
    }
    _dog->sendCmd(code);
    qInfo() << "[Main] 速度档位切换:" << cmd.speedLevel;
}
