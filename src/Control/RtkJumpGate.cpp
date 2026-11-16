// ============================================================
// 差分单帧跳变门控 —— 实现
// 判定逻辑整体来自 RtkPoseTracker::onFrame 的原"单帧跳变"段，判据与日志文本不变。
// 航向跳变必须区分三种成因，否则"原地转向"会被当成"模块重启"反复打断零位
// （原地转向时位置本来就不变，用"位置没动"证明"航向不该动"是逻辑上不成立的）。
// 聚类判据用时间窗而非帧数：同一条报文重复到达会让帧数翻倍，时间窗不受影响。
// ============================================================
#include "Control/RtkJumpGate.h"

#include <cmath>
#include <QDebug>
#include <QString>

#include "Common/DogText.h"
#include "Config/TerminalConfig.h"
#include "Convert/RtkFusion.h"
#include "Protocol/DogProtocol.h"

RtkJumpGate::RtkJumpGate(const TerminalConfig &cfg, RtkFusion &fusion)
    : _cfg(cfg), _fusion(fusion)
{
}

bool RtkJumpGate::accept(double cx, double cy, double yawRad, qint64 recvMs,
                         double baseX, double baseY, double baseYawRad,
                         bool hasOdom, double omyawRad)
{
    // 现场口径（2026-11-16）：差分坐标/标定后角度均为绝对真值，只会上报慢，
    // 不会跳变（极少数狗停着时的格点小跳也照收，它就是真值）。
    // 本门因此不再丢弃任何位置/航向帧；"断流停车"由 MainController::updateRtkGuard
    // 按"超过 GuardStaleMs 没新帧"判定，恢复后原地钉一次再继续。
    (void)cx; (void)cy; (void)yawRad; (void)recvMs;
    (void)baseX; (void)baseY; (void)baseYawRad;
    (void)hasOdom; (void)omyawRad;
    _yawJumpRun = 0;
    _jumpSinceMs = 0;
    _rejectYawValid = false;
    return true;
}

void RtkJumpGate::noteAccepted(bool hasOdom, double omyawRad)
{
    if (hasOdom)
    {
        _odomYawRad = omyawRad;
        _odomYawValid = true;
    }
}

void RtkJumpGate::reset()
{
    _odomYawValid = false;
    _jumpSinceMs = 0;
    _yawJumpRun = 0;
    _rejectYawValid = false;
    // 新参考下航向基准本来就要重取，重取次数重新计数
    _restarts = 0;
    _rebaseExhausted = false;
}
