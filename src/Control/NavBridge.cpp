// ============================================================
// 机器狗驱动节拍与运行模式 —— 实现
//
// 三个入口的分工：
//   acquireTick()  只关心"控制权拿到了没有"，成功后切移动模式并停表；
//   tick()         20Hz 节拍，先保直控、再判断位姿是否过期、最后才推进导航；
//   stepFromCache() 单拍推进，状态包路径与节拍路径共用，保证算法只有一套。
// 顺序不能调换：先判过期再推进，才能避免用几秒前的位姿继续驱动狗。
// ============================================================
#include "Control/NavBridge.h"

#include <cmath>
#include <QDateTime>
#include <QDebug>
#include <QTimer>

#include "Common/DogText.h"
#include "Config/TerminalConfig.h"
#include "Control/MotionState.h"
#include "Control/Navigator.h"
#include "Convert/RtkFusion.h"
#include "Convert/RtkPoseTracker.h"
#include "DogLink/DogClientBase.h"
#include "DogLink/ZsibotDefs.h"
#include "Report/ReportService.h"

NavBridge::NavBridge(const TerminalConfig &cfg, Navigator &navigator, RtkPoseTracker &rtk,
                     DogClientBase *&dog, MotionState &st, ReportService &report,
                     QTimer *acquireTimer, QTimer *velocityTimer,
                     bool &running, bool &controlAcquired, bool &velCtrlActive)
    : _cfg(cfg), _navigator(navigator), _rtk(rtk), _dog(dog), _st(st), _report(report),
      _acquireTimer(acquireTimer), _velocityTimer(velocityTimer),
      _running(running), _controlAcquired(controlAcquired), _velCtrlActive(velCtrlActive)
{
}

void NavBridge::acquireTick()
{
    if (!_dog->isConnected() || _controlAcquired)
    {
        _acquireTimer->stop();
        return;
    }

    // 控制权获取成功判定：狗反馈 functionMode == sdk
    if (_dog->feedback().valid && _dog->feedback().functionMode == Zsibot::FM_SDK)
    {
        _controlAcquired = true;
        _acquireTimer->stop();
        _dog->sendCmd(Zsibot::CMD_MOVE_MODE);  // 进入移动模式
        qInfo() << "[Main] 已取得SDK控制权，进入移动模式";
        _report.update();
        return;
    }

    if (_acquireAttempts >= _cfg.acquireRetryCount)
    {
        if (!_acquireRetryDone)
        {
            _acquireRetryDone = true;
            qInfo() << "[Main] SDK控制权未确认，已按配置停止重复申请"
                    << "(AcquireRetryCount=" << _cfg.acquireRetryCount << ")，继续等待狗端反馈";
        }
        return;
    }

    ++_acquireAttempts;
    _dog->sendCmd(Zsibot::CMD_SDK_CONTROL_RIGHT);
}

void NavBridge::beginAcquire()
{
    // 服务端可能在终端尚未连上狗时下发申请（如开机自动申请关闭的场景），
    // 此时直接下发无效，明确提示并放弃，避免重试定时器空转
    if (!_dog->isConnected())
    {
        qWarning() << "[Main] 申请SDK控制权被忽略：机器狗尚未连接";
        return;
    }

    qInfo() << "[Main] 申请SDK控制权（重试次数="
            << _cfg.acquireRetryCount << "，0=仅本次）...";
    _acquireAttempts = 0;
    _acquireRetryDone = false;
    _dog->sendCmd(Zsibot::CMD_SDK_CONTROL_RIGHT);
    _acquireTimer->start();
}

void NavBridge::remote(double forwardJoy, double turnJoy)
{
    if (_velCtrlActive)
    {
        qWarning() << "[Nav] 速度直控模式激活中，导航输出被忽略";
        return;
    }

    // 1Hz 打印导航闭环输出，便于确认 Navigator 是否真的在算
    const qint64 nowMs = QDateTime::currentMSecsSinceEpoch();
    if (nowMs - _lastNavOutMs >= 1000)
    {
        _lastNavOutMs = nowMs;
        qInfo() << "[Nav] 输出 forwardJoy=" << forwardJoy << "turnJoy=" << turnJoy;
    }

    _dog->sendRemote(static_cast<float>(forwardJoy), static_cast<float>(turnJoy), 0.0f, 0.0f);
}

void NavBridge::stepFromCache()
{
    if (!_st.valid)
        return;
    // 位姿来源：差分新鲜时用"差分锚点 + 里程计增量"，否则退回纯里程计。
    // 统一换回局部坐标再交给 Navigator，导航算法与目标坐标口径完全不变。
    double nx = _st.x;
    double ny = _st.y;
    double nyaw = _st.yaw;
    int src = RtkFusion::SrcNone;
    if (_rtk.fusedLocalPose(&nx, &ny, &nyaw, &src))
        _rtk.setPoseSrc(src);
    else
        _rtk.setPoseSrc(RtkFusion::SrcNone);
    // 位置/朝向取融合结果；速度量仍取里程计（仅作增益自适应与转向阻尼）
    _navigator.update(nx, ny, nyaw, _st.vx, _st.yawRate);
    // 任务边界同步：导航开始前/结束后各让差分吸附一次，段内不再跟踪（见 Rtk/SyncOnlyOnTask）
    _rtk.setNavActive(_navigator.isActive());
}

void NavBridge::tick()
{
    // 差分断流观测（判断与打印都在跟踪器内部，这里只触发）
    _rtk.checkStale();

    // 链路自愈必须跑在所有 return 之前：掉线时下面的分支全都直接返回，
    // 没人会再去握手，只能人工拿遥控器点一下——这正是现场反复"断线"的观感。
    recoverTick();

    if (!_running || !_dog->isConnected() || !_controlAcquired)
        return;

    // 速度直控优先（导航执行中不允许直控，二者不会同时激活）：
    // 按节拍续发最近一次直控量，使低频服务器指令也能被狗端持续接收
    if (_velCtrlActive)
    {
        _dog->sendRemote(static_cast<float>(_velFwd), static_cast<float>(_velTurn),
                         static_cast<float>(_velShift), 0.0f);
        return;
    }

    if (!_navigator.isActive())
        return;

    const qint64 now = nowMs();
    if (_st.valid && _cfg.navCtrlStaleStopMs > 0
        && (now - _st.ms) > static_cast<qint64>(_cfg.navCtrlStaleStopMs))
    {
        if (!_st.staleStopped)
        {
            _st.staleStopped = true;
            qWarning() << "[Main] 状态包中断" << (now - _st.ms)
                       << "ms，暂停导航输出（等待新状态恢复）";
            _dog->sendZeroRemote();
        }
        return;
    }

    // 用最近一次状态推进一拍：Navigator 内部按本拍时间做速度平滑与超时判定
    stepFromCache();
}

void NavBridge::recoverTick()
{
    // 关闭态直接返回：0 表示沿用"人工处理"的老行为（与遥控器共存时更安全）
    if (_cfg.dogAutoRecoverMs <= 0 || !_dog->isRunning())
        return;

    if (_dog->isConnected())
    {
        _recoverTries = 0;
        _lastRecoverMs = 0;
        _linkWasOk = true;
        // 链路回来了但控制权还没拿到：自动补一次申请。
        // 原来只有服务器显式下发 control 才会走 beginAcquire，掉线后就卡死在这里。
        if (!_controlAcquired && !_acquireTimer->isActive())
            beginAcquire();
        return;
    }

    const qint64 now = nowMs();
    if (_linkWasOk)
    {
        _linkWasOk = false;
        _lastRecoverMs = 0;   // 掉线边沿：不等周期，立刻发一次
        qWarning().noquote()
            << QStringLiteral("[Main] 状态包停推，链路自愈启动：每 %1ms 重发申请SDK控制权"
                              "（狗端只向会话持有者推状态，遥控器接管会打断该会话）")
                   .arg(_cfg.dogAutoRecoverMs);
    }

    if (_cfg.dogAutoRecoverMax > 0 && _recoverTries >= _cfg.dogAutoRecoverMax)
        return;   // 到上限后静默，不再无意义刷屏

    if (_lastRecoverMs != 0 && (now - _lastRecoverMs) < _cfg.dogAutoRecoverMs)
        return;

    _lastRecoverMs = now;
    ++_recoverTries;
    _dog->sendCmd(Zsibot::CMD_SDK_CONTROL_RIGHT);
    qWarning().noquote()
        << QStringLiteral("[Main] 链路自愈 %1/%2：已重发申请SDK控制权")
               .arg(_recoverTries)
               .arg(_cfg.dogAutoRecoverMax > 0 ? QString::number(_cfg.dogAutoRecoverMax)
                                               : QStringLiteral("不限"));
}

void NavBridge::enterVelocityMode()
{
    _velCtrlActive = true;
    _velocityTimer->start();  // 刷新死手定时器
    _report.update();
}

void NavBridge::exitVelocityMode(bool sendZero)
{
    if (_velCtrlActive)
    {
        _velCtrlActive = false;
        _velocityTimer->stop();
        // 清空续发缓存：避免退出直控后控制节拍把旧量再发出去
        _velFwd = 0.0;
        _velTurn = 0.0;
        _velShift = 0.0;
    }
    if (sendZero && _dog->isConnected() && _controlAcquired)
        _dog->sendZeroRemote();
}
