#ifndef NAV_BRIDGE_H
#define NAV_BRIDGE_H

#include <QtGlobal>

class QTimer;
class TerminalConfig;
class Navigator;
class RtkPoseTracker;
class DogClientBase;
struct MotionState;
class ReportService;

// ============================================================
// 机器狗驱动节拍与运行模式
//
// 只负责"车怎么动"这一类事：控制权申请状态机、速度直控的进入/退出与续发、
// 以及按固定节拍用最近一次缓存位姿推进导航。不解析报文、不打包上报、
// 不持有链路，定时器与运行标志都由主控制器持有，这里只借用。
//
// 抽出来的直接收益：20Hz 续发缓存（直控量）与"状态包中断后暂停盲发"
// 这两处状态原本散在主控制器里，谁都可能读写；现在只有本类写、外部只读。
// ============================================================
class NavBridge
{
public:
    NavBridge(const TerminalConfig &cfg, Navigator &navigator, RtkPoseTracker &rtk,
              DogClientBase *&dog, MotionState &st, ReportService &report,
              QTimer *acquireTimer, QTimer *velocityTimer,
              bool &running, bool &controlAcquired, bool &velCtrlActive);

    // 控制权申请：重置重试计数并立即下发一次，随后交给定时器按拍重试
    void beginAcquire();
    // 控制权确认：狗端 feedback.functionMode == sdk 时进入移动模式
    void acquireTick();

    // 导航遥控量下发（含 1Hz 输出打印节流）
    void remote(double forwardJoy, double turnJoy);
    // 用最近一次缓存位姿推进一拍导航；状态包路径也走这里，避免两套算法分叉
    void stepFromCache();
    // 控制节拍：直控续发 → 过期保护 → 推进导航
    void tick();

    // 链路自愈：状态包停推（狗端会话失效）时自动重新握手，替代"人工拿遥控器点一下"
    void recoverTick();

    void enterVelocityMode();
    void exitVelocityMode(bool sendZero);

    // 续发缓存：下行指令写入，控制节拍读取；对外只允许拿引用，不允许复制
    double &velFwdRef() { return _velFwd; }
    double &velTurnRef() { return _velTurn; }
    double &velShiftRef() { return _velShift; }

private:
    const TerminalConfig &_cfg;
    Navigator &_navigator;
    RtkPoseTracker &_rtk;
    DogClientBase *&_dog;
    MotionState &_st;
    ReportService &_report;
    QTimer *_acquireTimer;
    QTimer *_velocityTimer;
    bool &_running;
    bool &_controlAcquired;
    bool &_velCtrlActive;

    int _acquireAttempts = 0;
    bool _acquireRetryDone = false;
    qint64 _lastNavOutMs = 0;

    // 链路自愈状态
    int _recoverTries = 0;
    qint64 _lastRecoverMs = 0;
    bool _linkWasOk = true;   // 上一次节拍里链路是否健康，用于识别"掉线边沿"

    // 速度直控最近一次下发量：控制节拍按 20Hz 续发，替代依赖服务器重发
    double _velFwd = 0.0;
    double _velTurn = 0.0;
    double _velShift = 0.0;
};

#endif  // NAV_BRIDGE_H
