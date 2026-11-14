// ============================================================
// MainController 的驱动侧槽函数
//
// 本文件只剩"信号 → 转调 NavBridge"这一层：所有会改变驱动行为的逻辑
// （控制权申请、节拍推进、直控续发、任务取消）都在 Control/NavBridge，
// 这里保留槽是因为信号连接必须挂在 QObject 上，而不是因为逻辑该放这。
// ============================================================

#include "MainController.h"

#include <QDataStream>
#include <QDateTime>
#include <QDebug>
#include <cmath>

#include "Common/DogText.h"
#include "DogLink/ZsibotDefs.h"

void MainController::onDogConnectedChanged(bool connected)
{
    if (connected)
    {
        qInfo() << "[Main] 机器狗连接状态: 已连接";
        _controlAcquired = false;
        if (_cfg.autoAcquireControl)
            _nav->beginAcquire();
    }
    else
    {
        qWarning().noquote()
            << QStringLiteral("[Main] 机器狗连接状态: 断开（超过 %1ms 未收到狗状态包；"
                              "常见原因：遥控器接管期间狗停发状态 / 网络中断 / HeartbeatRole 身份冲突）")
                   .arg(_cfg.dogTimeoutMs);
        _acquireTimer->stop();
        _controlAcquired = false;
        if (_navigator->isActive())
            cancelActiveTask(DogProtocol::MR_FAILED, QStringLiteral("机器狗断连"));
        _nav->exitVelocityMode(false);
    }
    _report->update();
}

void MainController::onControlAcquireTick()
{
    _nav->acquireTick();
}

void MainController::beginControlAcquire()
{
    _nav->beginAcquire();
}

void MainController::onNavRemote(double forwardJoy, double turnJoy)
{
    _nav->remote(forwardJoy, turnJoy);
}

void MainController::onNavStop()
{
    _dog->sendZeroRemote();
}

void MainController::onNavFinished(quint32 taskId, DogProtocol::MoveResult result,
                                   double remainingDist, const QString &message)
{
    qInfo() << "[Main] 导航结束 task" << taskId << "result =" << result
            << "剩余距离" << remainingDist << message;

    // ---- 到点复核（[Nav] ArriveVerify）----
    // 只对"到达"生效：到达那一下 Navigator 已经发过 stopRequested、狗在停，
    // 正好是等一个静置锚点把位姿钉准的时机。钉准后再看"地图目标 vs 实际位姿"
    // 的残差，超过门限就补一小段（最多 ArriveMaxFix 段），避免把 0.4m 的落点
    // 误差当成"已到达"上报。差分格点 0.61×0.42m、单点 ±0.2~0.3m，
    // 只靠到达半径 0.2/0.3m 判定是判不准的。
    if (_cfg.navArriveVerify && result == DogProtocol::MR_ARRIVED
        && !_avHold && (_avFixLeft > 0 || _avTaskId != taskId))
    {
        _avHold   = true;
        _avTaskId = taskId;
        _avResult = result;
        _avRemain = remainingDist;
        _avMsg    = message;
        _avFixLeft = _cfg.navArriveMaxFix;
        _avAnchorSeq = _anchorSeq;
        _avDeadlineMs = nowMs() + _cfg.navArriveWaitMs;
        qWarning().noquote()
            << QStringLiteral("[Arrive] 到点复核开始 task%1：等静置锚点（≤%2ms）后按残差决定是否补段")
                   .arg(taskId).arg(_cfg.navArriveWaitMs);
        return;
    }

    // 任务结束报文多带一个剩余距离，与普通 feedback 不同，单独走一条
    _report->publishTaskResult(taskId, result, remainingDist, message);
}

// 到点复核的判定拍：锚点已到 或 等超时 → 算残差 → 补段 / 发布
void MainController::maybeFinishArriveVerify(qint64 nowMs)
{
    if (!_avHold)
        return;

    // 服务器已经下发新任务在跑：手上的旧结果立刻发布，不再占用复核窗口
    if (_navigator->isActive() && _navigator->taskId() != _avTaskId)
    {
        publishHeldArrive();
        return;
    }

    const bool newAnchor = (_anchorSeq != _avAnchorSeq);
    const bool timedOut  = (nowMs >= _avDeadlineMs);
    if (!newAnchor && !timedOut)
        return;   // 既没新锚点也没超时：继续等（狗刚停稳，锚点要 3.2s 才出）

    double wantMx = 0.0;
    double wantMy = 0.0;
    if (_map.isActive())
        _map.toMap(_navigator->targetLocalX(), _navigator->targetLocalY(), 0.0,
                   &wantMx, &wantMy, nullptr);
    else
    {
        wantMx = _navigator->targetLocalX();
        wantMy = _navigator->targetLocalY();
    }

    const double resX = _st.x - wantMx;
    const double resY = _st.y - wantMy;
    const double residual = std::sqrt(resX * resX + resY * resY);

    // 补一段的条件：残差超门限、还能补、且残差在可修正范围内。
    // >1.5m 说明不是"到点差一点"，而是地图/方向出了问题——补段救不了，
    // 直接如实上报，让现场看到真实残差。
    if (newAnchor && residual >= _cfg.navArriveResidualM && residual <= 1.5
        && _avFixLeft > 0 && !_navigator->isActive())
    {
        --_avFixLeft;
        // 目标是"同一任务的地图目标"，重新换算成局部坐标后补一小段
        double tx = wantMx;
        double ty = wantMy;
        if (_map.isActive())
            _map.toLocal(wantMx, wantMy, 0.0, &tx, &ty, nullptr);
        _avHold = false;
        qWarning().noquote()
            << QStringLiteral("[Arrive] 复核残差 %1m ≥ %2m，补第 %3 段修正"
                              "（到达半径 %4m，剩余可补 %5 段）")
                   .arg(residual, 0, 'f', 2)
                   .arg(_cfg.navArriveResidualM, 0, 'f', 2)
                   .arg(_cfg.navArriveMaxFix - _avFixLeft)
                   .arg(_cfg.navArriveFixRadiusM, 0, 'f', 2)
                   .arg(_avFixLeft);
        // 修正段用小速度：到达半径只有 0.1m，按巡航 0.8m/s 冲进去必然过冲、
        // 再反向找点，来回蹭几次就把复核窗口耗掉。0.35m/s 时单帧位移 1.7cm。
        const double fixSpeed = qMin(_navigator->params().maxLinearSpeed, 0.35);
        _navigator->start(_avTaskId, tx, ty, fixSpeed,
                          _cfg.navArriveFixRadiusM, 30000);
        return;
    }

    qWarning().noquote()
        << QStringLiteral("[Arrive] 复核结束 task%1：残差 %2m（门限 %3m）%4")
               .arg(_avTaskId)
               .arg(residual, 0, 'f', 2)
               .arg(_cfg.navArriveResidualM, 0, 'f', 2)
               .arg(newAnchor ? QStringLiteral("· 锚点已生效")
                              : QStringLiteral("· 等锚点超时（狗可能没停稳或差分断流）"));
    _avRemain = residual;   // 上报真实残差，让服务器/现场看到落点偏差
    _avMsg = _avMsg + QStringLiteral("；到点复核残差 %1m（差分锚点）")
                          .arg(residual, 0, 'f', 2);
    publishHeldArrive();
}

void MainController::publishHeldArrive()
{
    if (!_avHold)
        return;
    _avHold = false;
    _avFixLeft = 0;
    _report->publishTaskResult(_avTaskId, _avResult, _avRemain, _avMsg);
}

// 方向是否已闭环：只在"差分可用 + 位置走死推算"时有意义，
// 其它组合恒 true（该判据不适用，不能因此拦任务）。
bool MainController::mapDirectionAligned() const
{
    if (!_cfg.rtkEnabled || !_cfg.posePosSourceDr || !_cfg.rtkAnchorRest)
        return true;
    return _rkRotApplied > 0;
}

void MainController::onVelocityTimeout()
{
    if (!_velCtrlActive)
        return;
    qWarning() << "[Main] 速度直控超时（死手保护），自动零速";
    _nav->exitVelocityMode(true);
    _report->update();
}

void MainController::onControlTick()
{
    _nav->tick();
}

void MainController::updateNavigatorFromCache()
{
    _nav->stepFromCache();
}

void MainController::enterVelocityMode()
{
    _nav->enterVelocityMode();
}

void MainController::exitVelocityMode(bool sendZero)
{
    _nav->exitVelocityMode(sendZero);
}

void MainController::cancelActiveTask(DogProtocol::MoveResult result, const QString &reason)
{
    const quint32 taskId = _navigator->taskId();
    _navigator->cancel();
    _dog->sendZeroRemote();
    _report->publishFeedback(taskId, result, reason);
}
