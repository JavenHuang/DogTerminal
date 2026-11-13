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
    const double dx = cx - baseX;
    const double dy = cy - baseY;
    const double dist = std::sqrt(dx * dx + dy * dy);
    const double dRtkDeg = DogProtocol::radStd(yawRad - baseYawRad) * kRadToDeg;
    const double dyawDeg = std::fabs(dRtkDeg);

    if (dist > _cfg.rtkMaxJumpM)
    {
        // 纯位置跳变：只按野值丢弃，不参与重启判定
        _yawJumpRun = 0;
        _jumpSinceMs = 0;
        _rejectYawValid = false;
        ++_badJumps;
        qWarning().noquote() << QStringLiteral("[Rtk] 位置跳变丢弃 Δ=%1m Δyaw=%2° 累计%3帧")
            .arg(dist, 0, 'f', 2).arg(dyawDeg, 0, 'f', 1).arg(_badJumps);
        return false;
    }

    if (dyawDeg > _cfg.rtkMaxJumpDeg)
    {
        const qint64 kRestartHoldMs = 600;
        const double dOdomDeg = (hasOdom && _odomYawValid)
                                    ? DogProtocol::radStd(omyawRad - _odomYawRad) * kRadToDeg
                                    : 0.0;
        const bool odomTurned = (std::fabs(dOdomDeg) >= 5.0);
        const bool sameSense = (dRtkDeg * dOdomDeg > 0.0);
        const bool sameAmount = (std::fabs(dyawDeg - std::fabs(dOdomDeg)) <= 60.0);

        if (odomTurned && sameSense && sameAmount)
        {
            // A：真转向。接受本帧，绝不清零位——清零位等于把"地图东"搬到当前机头方向，
            //    每清一次就丢一个等于当时转角的航向偏置。
            _yawJumpRun = 0;
            _jumpSinceMs = 0;
            _rejectYawValid = false;
            qInfo().noquote() << QStringLiteral(
                "[Rtk] 航向台阶跟随里程计（差分 %1° / 里程计 %2°），按真转向接受")
                .arg(dRtkDeg, 0, 'f', 1).arg(dOdomDeg, 0, 'f', 1);
        }
        else if (!odomTurned)
        {
            // B：狗没转、只有差分航向跳。同一角度簇稳住够久才算模块重启。
            const bool sameCluster =
                _rejectYawValid
                && std::fabs(DogProtocol::radStd(yawRad - _rejectYawRad)) * kRadToDeg < 5.0;
            if (!sameCluster || _jumpSinceMs == 0)
                _jumpSinceMs = recvMs;
            _rejectYawRad = yawRad;
            _rejectYawValid = true;
            ++_yawJumpRun;
            const qint64 heldMs = recvMs - _jumpSinceMs;
            if (heldMs >= kRestartHoldMs)
            {
                _yawJumpRun = 0;
                _jumpSinceMs = 0;
                _rejectYawValid = false;
                ++_restarts;
                qWarning().noquote() << QStringLiteral(
                    "[Rtk] 狗未转动时差分航向偏移 %1° 并稳定 %2ms（累计判定 %3 次）→ "
                    "判为差分模块重启，接受本帧并按当前地图朝向重取零位")
                    .arg(dyawDeg, 0, 'f', 1).arg(heldMs).arg(_restarts);
                _fusion.requestYawZero(QStringLiteral("差分模块重启"), omyawRad);
            }
            else
            {
                ++_badJumps;
                qWarning().noquote() << QStringLiteral(
                    "[Rtk] 单帧航向跳变丢弃 Δyaw=%1°（里程计同期 %2°，累计%3帧）")
                    .arg(dyawDeg, 0, 'f', 1).arg(dOdomDeg, 0, 'f', 1).arg(_badJumps);
                return false;
            }
        }
        else
        {
            // C：狗转了，但差分航向跟错方向或跟丢
            _yawJumpRun = 0;
            _jumpSinceMs = 0;
            _rejectYawValid = false;
            ++_badJumps;
            qWarning().noquote() << QStringLiteral(
                "[Rtk] 航向与里程计不符（差分 %1° / 里程计 %2°），丢弃该帧："
                "先核对航向符号与轴向（东0°/南90°/西180°/北270°）")
                .arg(dRtkDeg, 0, 'f', 1).arg(dOdomDeg, 0, 'f', 1);
            return false;
        }
    }
    else
    {
        _yawJumpRun = 0;
        _jumpSinceMs = 0;
        _rejectYawValid = false;
    }

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
}
