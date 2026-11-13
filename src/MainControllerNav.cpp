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
    // 任务结束报文多带一个剩余距离，与普通 feedback 不同，单独走一条
    _report->publishTaskResult(taskId, result, remainingDist, message);
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
