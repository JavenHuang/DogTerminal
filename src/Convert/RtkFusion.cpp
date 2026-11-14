#include "RtkFusion.h"

#include <QDebug>
#include <QSettings>
#include <QtMath>
#include <cmath>

namespace {

const double kPi = 3.14159265358979323846;

inline double radStdLocal(double a)
{
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}

inline double deg0To360(double rad)
{
    double d = rad * 180.0 / kPi;
    d = std::fmod(d, 360.0);
    if (d < 0.0) d += 360.0;
    return d;
}

}  // namespace

void RtkFusion::clearSamples()
{
    _samples.clear();
    _n = 0;
    _spx = _spy = _sqx = _sqy = 0.0;
    _spxqx = _spyqy = _spxqy = _spyqx = 0.0;
    _sqq = 0.0;
    _sdyaw = 0.0;
    _path = 0.0;
    _hasPrev = false;
    _prevPx = 0.0;
    _prevPy = 0.0;
    _lastSolveMs = 0;
    _bandRejectWarned = false;   // 换了参考点/重标后允许再报一次越界
}

void RtkFusion::reset()
{
    _offInit = false;
    _offX = 0.0;
    _offY = 0.0;
    _offYaw = 0.0;
    _offMs = 0;
    _lastFrameMs = 0;
    _lastSrc = SrcNone;
    _bigWarned = false;
    _offFrozen = false;
    _lastSpeed = 0.0;
    _driftSum = 0.0;
    _driftCount = 0;
    _driftWarned = false;
    _hasPrevOdom = false;
    _prevOdomX = 0.0;
    _prevOdomY = 0.0;
    _prevOdomMs = 0;
    _lastRejectLogMs = 0;
    _staticSinceMs = 0;
    _staticStill = false;
    _staticPointTaken = false;
    // 零位 b 属于"旧参考的那一姿态"，参考点一换就立即作废（由调用方随后重取）
    _zeroYawRad = 0.0;
    _zeroValid = false;
    _zeroPending = false;
    _zeroSinceMs = 0;
    _zeroN = 0;
    _zeroSin = 0.0;
    _zeroCos = 0.0;
    _zeroFirstYaw = 0.0;
    _zeroMaxDev = 0.0;
    _zeroSpreadDeg = 0.0;
    _zeroMs = 0;
    _zeroFromLineSet = false;
    _zeroTargetYawRad = 0.0;
    // 直线观测器：参考点换了，之前的线段全部作废
    _lzActive = false;
    _lzHasSeg = false;
    _lzSegPx = 0.0;
    _lzSegPy = 0.0;
    _lzSegYaw = 0.0;
    _lzLen = 0.0;
    _lzObs.clear();
    _lzWarned = false;
    clearSamples();
}

void RtkFusion::requestYawZero(const QString &reason, double targetMapYawRad)
{
    if (!_p.autoZeroYaw)
        return;
    _zeroTargetYawRad = targetMapYawRad;
    // 不清样本：k/α 是硬件常数，与参考点无关
    _zeroValid = false;
    _zeroYawRad = 0.0;
    _zeroPending = true;
    _zeroSinceMs = 0;
    _zeroN = 0;
    _zeroSin = 0.0;
    _zeroCos = 0.0;
    _zeroFirstYaw = 0.0;
    _zeroMaxDev = 0.0;
    _zeroSpreadDeg = 0.0;
    _zeroFromLineSet = false;
    // 启用直线观测：先给一个"建参考姿态"的初值，随后由直线段反算升级为
    // "地图东"口径。两者可以并存，也可以只走其中之一（ZeroFromLine=false 就只取初值）。
    _lzHasSeg = false;
    _lzSegPx = 0.0;
    _lzSegPy = 0.0;
    _lzSegYaw = 0.0;
    _lzLen = 0.0;
    _lzObs.clear();
    _lzWarned = false;
    _lzActive = _p.zeroFromLine && _p.useRtkYaw;
    qInfo().noquote() << QStringLiteral("[Rtk] 请求重取差分航向零位：%1（等静止窗口 %2ms / %3 帧）")
        .arg(reason).arg(_p.zeroHoldMs).arg(_p.zeroMinFrames);
    if (_lzActive)
        qInfo().noquote() << QStringLiteral(
            "[Rtk] 直线反算零位已启用：每段 ≥%1m、累计 ≥%2m、各段散布 ≤%3° 后自动把 b 归到地图东")
            .arg(_p.zeroLineSegM, 0, 'f', 1).arg(_p.zeroLineMinM, 0, 'f', 1)
            .arg(_p.zeroLineSpreadDeg, 0, 'f', 1);
}

void RtkFusion::addSample(const Sample &s, bool isStatic)
{
    // 里程计坐标系跳变（遥控接管后 odom 归零等）会让相邻样本相距很远。
    // 静态锚点是"走到了另一个位置再停"的稀疏采样，间距本来就大，只要不是
    // 坐标系级跳变（>50m）就照常计入里程；动态样本仍用 1.5m 门限。
    if (_hasPrev)
    {
        const double d = std::sqrt((s.qx - _prevPx) * (s.qx - _prevPx)
                                   + (s.qy - _prevPy) * (s.qy - _prevPy));
        if ((!isStatic && d > 1.5) || d > 50.0)
        {
            _prevPx = s.qx;
            _prevPy = s.qy;
            return;   // 丢弃该帧，不累积
        }
        _path += d;
    }
    else
    {
        _hasPrev = true;
    }
    _prevPx = s.qx;
    _prevPy = s.qy;

    if (_samples.size() < 4000)
        _samples.append(s);

    ++_n;
    _spx += s.px;
    _spy += s.py;
    _sqx += s.qx;
    _sqy += s.qy;
    _spxqx += s.px * s.qx;
    _spyqy += s.py * s.qy;
    _spxqy += s.px * s.qy;
    _spyqx += s.py * s.qx;
    _sqq += s.qx * s.qx + s.qy * s.qy;
    _sdyaw += s.dyaw;
}

void RtkFusion::onRtkFrame(double rtkX, double rtkY, double rtkYaw,
                           double odomX, double odomY, double odomYaw, qint64 tMs,
                           double speedMps, double yawRateRad)
{
    _lastFrameMs = tMs;
    _lastSpeed = speedMps;
    _lastRtkX = rtkX;
    _lastRtkY = rtkY;
    _lastRtkYaw = rtkYaw;   // 先存原值，取到零位后再在下方换成归一化值
    _lastOdomX = odomX;
    _lastOdomY = odomY;
    _lastOdomYaw = odomYaw;
    _hasLastFrame = true;

    // ---- 里程计坐标系重锚检测：标定样本必须全部来自同一套里程计系 ----
    // 遥控接管、狗端 SLAM 重定位都会让 odom 在一个帧间隔里整块平移（实测 3~5m，
    // 狗本身不可能在 0.2s 内走那么多）。这之后采的锚点与之前的不在同一坐标系，
    // 混进同一次相似变换拟合必然发散：RMS 远超上限就被静默拒绝，
    // 面板还会错报"里程不足 0.00m"，把人引向"再多走点"，走多少都没用。
    // 所以这里把"坐标系变了"这件事当场识别出来：清掉旧样本、保留 k/α 与零位 b。
    if (_hasPrevOdom)
    {
        const double odStep = std::sqrt((odomX - _prevOdomX) * (odomX - _prevOdomX)
                                        + (odomY - _prevOdomY) * (odomY - _prevOdomY));
        const double dtSec = static_cast<double>(tMs - _prevOdomMs) / 1000.0;
        // 这一段里"狗真的走了这么远"的上限：按当前速度外推再留 0.5m 余量。
        // 断链后差分帧可能隔好几秒才来，固定 2m 门限会把正常行走误判成重锚，
        // 所以门限必须跟着时间间隔长。
        const double legit = _lastSpeed * dtSec + 0.5;
        if (odStep > qMax(2.0, legit))
        {
            qWarning().noquote() << QStringLiteral(
                "[Rtk] 里程计坐标系重锚 %1m/%2ms：旧标定样本作废（k/α 与零位 b 保留），"
                "从当前位置重新收锚点")
                .arg(odStep, 0, 'f', 2).arg(tMs - _prevOdomMs);
            clearSamples();             // _path/_n/_samples 清零；_bandRejectWarned 重臂
            _staticPointTaken = false;  // 当前位置立刻允许再采一个锚点
            _snapNext = true;           // 里程计整块平移过，offset 必须重新吸附
        }
    }
    _prevOdomX = odomX;
    _prevOdomY = odomY;
    _prevOdomMs = tMs;
    _hasPrevOdom = true;
    // 里程计 -> 绝对(差分)系：只做尺度与转角，平移交给 offset 动态吸收。
    // 方向必须是"里程计映射到差分"，而不是反过来：差分的米才是地图上真实的米，
    // 反过来把差分压进里程计尺度，会把 mapcalib 的残余尺度误差带进最终位姿。
    // 两者手性一致（都是 x 东、y 南、顺时针角），不存在镜像。
    const double c = std::cos(_calib.alphaRad);
    const double s = std::sin(_calib.alphaRad);
    const double convX = _calib.k * (c * odomX - s * odomY);
    const double convY = _calib.k * (s * odomX + c * odomY);
    const double convYaw = radStdLocal(odomYaw + _calib.alphaRad);

    const double yawRateDeg = std::fabs(yawRateRad) * 180.0 / kPi;
    const bool stillNow = (speedMps < _p.offStaticSpeed) && (yawRateDeg < _p.offSlowYawRateDeg);
    const bool slowNow = (speedMps < _p.offSlowSpeed) && (yawRateDeg < _p.offSlowYawRateDeg);

    // ---- 差分航向零位 b：在“建参考之后”的静止窗口内取一次 ----
    // 差分航向每次上电的零位是随机的（转向增量准确）。建参考那一姿态就是地图 0°，
    // 所以 b = 那一姿态下的差分原值，归一化 yaw = rtkYaw - b 在参考姿态恒为 0。
    // 取到之前一律不建立/更新 offset：否则会用一个随机零位把航向基准初始化错。
    if (_p.autoZeroYaw && _zeroPending)
    {
        if (!stillNow)
        {
            _zeroSinceMs = 0;      // 窗口作废，等下一次停稳
            _zeroN = 0;
            _zeroSin = _zeroCos = 0.0;
            _zeroMaxDev = 0.0;
        }
        else
        {
            if (_zeroSinceMs == 0)
            {
                _zeroSinceMs = tMs;
                _zeroFirstYaw = rtkYaw;
                _zeroSin = _zeroCos = 0.0;
                _zeroN = 0;
                _zeroMaxDev = 0.0;
            }
            if ((tMs - _zeroSinceMs) >= static_cast<qint64>(_p.zeroHoldMs))
            {
                _zeroSin += std::sin(rtkYaw);
                _zeroCos += std::cos(rtkYaw);
                ++_zeroN;
                const double devDeg = std::fabs(radStdLocal(rtkYaw - _zeroFirstYaw)) * 180.0 / kPi;
                if (devDeg > _zeroMaxDev)
                    _zeroMaxDev = devDeg;
                if (_zeroN >= _p.zeroMinFrames)
                {
                    if (_zeroMaxDev > 3.0)
                    {
                        // 静止却没稳住：窗内抖动超限，丢掉重来，不把噪声写进基准
                        qWarning().noquote() << QStringLiteral(
                            "[Rtk] 差分零位窗口抖动 %1°（上限 3°），重置重取")
                            .arg(_zeroMaxDev, 0, 'f', 2);
                        _zeroSinceMs = 0;
                        _zeroN = 0;
                        _zeroSin = _zeroCos = 0.0;
                        _zeroMaxDev = 0.0;
                    }
                    else
                    {
                        // b 的定义：使"归一化航向"在本次取零的姿态上等于该姿态的地图朝向。
                        // 建参考时目标为 0（等价于旧的 b=读数）；模块重启时目标传当前里程计
                        // 地图 yaw，从而不会把"自参考以来转过的角"变成固定偏置。
                        _zeroYawRad = radStdLocal(std::atan2(_zeroSin, _zeroCos) - _zeroTargetYawRad);
                        _zeroValid = true;
                        _zeroPending = false;
                        _zeroMs = tMs;
                        _zeroSpreadDeg = _zeroMaxDev;
                        _snapNext = true;   // 偏移的航向分量是按旧零位算的，必须重算
                        qInfo().noquote() << QStringLiteral(
                            "[Rtk] 差分零位已记录 b=%1°（静止 %2 帧，抖动 %3°，对齐地图 yaw %4°）；"
                            "此后差分航向按“相对 b”解释，重启终端需重取")
                            .arg(deg0To360(_zeroYawRad), 0, 'f', 1)
                            .arg(_zeroN).arg(_zeroMaxDev, 0, 'f', 2)
                            .arg(deg0To360(_zeroTargetYawRad), 0, 'f', 1);
                    }
                }
            }
        }
    }

    // 本帧用于一切换算的航向：已扣除零位 b
    const double rtkYawUsed = radStdLocal(rtkYaw - _zeroYawRad);
    _lastRtkYaw = rtkYawUsed;   // trySolve 用它重算 offset 的航向分量
    // 零位还在等：位置偏移照常更新（位置不依赖 b），但航向偏移必须保持旧值——
    // 用一个随机零位去更新航向偏移，等于把随机角焊进地图。
    const bool zeroPendingNow = (_p.autoZeroYaw && _zeroPending);

    // ---- 直线反算零位（与 k/α 标定完全独立，k/α 已标定的场次也能用）----
    // 停点 A→B 的差分位移方向 θ 就是狗在服务端地图里的真实行进方向；这段路程里
    // 差分给自己的航向读数与 θ 之差，就是零位 b（天线安装角一并合并进去）。
    // 所以"把 b 归到地图东"不再依赖"建参考时狗正好朝东"。
    if (_lzActive && stillNow)
    {
        if (!_lzHasSeg)
        {
            _lzSegPx = rtkX;
            _lzSegPy = rtkY;
            _lzSegYaw = rtkYaw;
            _lzHasSeg = true;
        }
        else
        {
            const double ldx = rtkX - _lzSegPx;
            const double ldy = rtkY - _lzSegPy;
            const double segLen = std::sqrt(ldx * ldx + ldy * ldy);
            const double turnDeg = std::fabs(radStdLocal(rtkYaw - _lzSegYaw)) * 180.0 / kPi;
            if (segLen > 5.0 || turnDeg > 3.0)
            {
                // 中途拐弯/掉头/坐标系跳变：本段不成立，起点挪到当前位置
                if (!_lzWarned)
                {
                    _lzWarned = true;
                    qWarning().noquote() << QStringLiteral(
                        "[Rtk] 直线段作废（位移 %1m，航向变化 %2°）：反算零位要求中途不纠偏、不拐弯")
                        .arg(segLen, 0, 'f', 2).arg(turnDeg, 0, 'f', 1);
                }
                _lzSegPx = rtkX;
                _lzSegPy = rtkY;
                _lzSegYaw = rtkYaw;
            }
            else if (segLen >= _p.zeroLineSegM)
            {
                const double theta = std::atan2(ldy, ldx);
                const double yawMid = std::atan2(std::sin(_lzSegYaw) + std::sin(rtkYaw),
                                                 std::cos(_lzSegYaw) + std::cos(rtkYaw));
                _lzObs.append(radStdLocal(yawMid - theta));
                _lzLen += segLen;
                _lzSegPx = rtkX;
                _lzSegPy = rtkY;
                _lzSegYaw = rtkYaw;
                if (_lzLen >= _p.zeroLineMinM && _lzObs.size() >= 2)
                {
                    double sSum = 0.0;
                    double cSum = 0.0;
                    for (int i = 0; i < _lzObs.size(); ++i)
                    {
                        sSum += std::sin(_lzObs.at(i));
                        cSum += std::cos(_lzObs.at(i));
                    }
                    const double bNew = std::atan2(sSum, cSum);
                    double maxDev = 0.0;
                    for (int i = 0; i < _lzObs.size(); ++i)
                    {
                        const double dv =
                            std::fabs(radStdLocal(_lzObs.at(i) - bNew)) * 180.0 / kPi;
                        if (dv > maxDev)
                            maxDev = dv;
                    }
                    if (maxDev <= _p.zeroLineSpreadDeg)
                    {
                        const double bOld = _zeroYawRad;
                        _zeroYawRad = bNew;
                        _zeroValid = true;
                        _zeroPending = false;
                        _zeroSpreadDeg = maxDev;
                        _zeroMs = tMs;
                        _zeroFromLineSet = true;
                        _snapNext = true;   // 零位一换，offset 的航向分量整体作废
                        // 本帧的归一化航向与已累计的航向差口径都要跟着挪
                        _lastRtkYaw = radStdLocal(_lastRtkYaw + (bOld - bNew));
                        if (_n > 0)
                            _sdyaw += (bOld - bNew) * static_cast<double>(_n);
                        qInfo().noquote() << QStringLiteral(
                            "[Rtk] 差分零位（直线反算）b=%1°｜直线 %2m / %3 段，散布 %4°｜"
                            "原建参考读数 b0=%5°，差 %6°")
                            .arg(deg0To360(bNew), 0, 'f', 1)
                            .arg(_lzLen, 0, 'f', 2).arg(_lzObs.size()).arg(maxDev, 0, 'f', 2)
                            .arg(deg0To360(bOld), 0, 'f', 1)
                            .arg(radStdLocal(bOld - bNew) * 180.0 / kPi, 0, 'f', 1);
                        _lzActive = false;   // 已定，等下一次建参考 / 模块重启再观测
                    }
                    else
                    {
                        qWarning().noquote() << QStringLiteral(
                            "[Rtk] 直线反算零位散布 %1°（上限 %2°）：各段不一致，丢弃重观测"
                            "（常见原因：中途纠偏、走成折线、差分模块重启、服务端 yaw 恒定）")
                            .arg(maxDev, 0, 'f', 2).arg(_p.zeroLineSpreadDeg, 0, 'f', 1);
                        _lzObs.clear();
                        _lzLen = 0.0;
                    }
                }
            }
        }
    }

    // 静止段判定：连续静止 calibStaticHoldMs 以上才算"停稳"
    if (stillNow)
    {
        if (_staticSinceMs == 0)
            _staticSinceMs = tMs;
        if (!_staticStill && (tMs - _staticSinceMs) >= _p.calibStaticHoldMs)
            _staticStill = true;
    }
    else
    {
        _staticSinceMs = 0;
        _staticStill = false;
        _staticPointTaken = false;
    }

    // 标定样本只采"静止锚点"：停稳后每段采一个点，段内不再重复采。
    // 差分滞后在静止时等于零（狗没动、差分也给同一处），所以这些点的
    // (差分坐标, 里程计坐标) 是严格同时刻的，解 k/α 不会被时延污染。
    // 运动中不再采点：500ms 的滞后在 0.5m/s 时就是 0.25m 的形状误差，
    // 会让 k 和 α 系统性偏掉。
    if (_p.autoCalib && !_calib.valid && !zeroPendingNow && _staticStill && !_staticPointTaken)
    {
        bool farEnough = true;
        for (int i = 0; i < _samples.size(); ++i)
        {
            const Sample &sm = _samples.at(i);
            if (std::sqrt((odomX - sm.qx) * (odomX - sm.qx)
                          + (odomY - sm.qy) * (odomY - sm.qy)) < _p.calibPointGapM)
            {
                farEnough = false;   // 原地重复停，同一个位置不算新点
                break;
            }
        }
        _staticPointTaken = true;
        if (farEnough)
        {
            Sample sa;
            sa.px = rtkX;
            sa.py = rtkY;
            sa.qx = odomX;
            sa.qy = odomY;
            sa.dyaw = radStdLocal(rtkYawUsed - odomYaw);
            addSample(sa, true);
            qInfo().noquote() << QStringLiteral(
                "[Rtk] 静止锚点 #%1 差分(%2, %3) 里程计(%4, %5) 累计里程=%6m")
                .arg(_n).arg(rtkX, 0, 'f', 2).arg(rtkY, 0, 'f', 2)
                .arg(odomX, 0, 'f', 2).arg(odomY, 0, 'f', 2)
                .arg(_path, 0, 'f', 2);
        }
    }

    const double measX = rtkX - convX;
    const double measY = rtkY - convY;
    // 服务端航向可能恒为 0（只算位置不算朝向）。此时若把它当绝对朝向，
    // 会把狗的朝向钉死在一个固定角上，导航必然跑偏。
    // 置 useRtkYaw=false 后，朝向只走"里程计 + 位置标定出的转角 alpha"，不进 offset。
    // 零位待取期间也不更新航向偏移（保持旧值），位置偏移照常跟踪。
    double measYaw = 0.0;
    if (zeroPendingNow)
        measYaw = _offInit ? _offYaw : 0.0;
    else if (_p.useRtkYaw)
        measYaw = radStdLocal(rtkYawUsed - convYaw);

    if (!_offInit)
    {
        _offX = measX;
        _offY = measY;
        _offYaw = measYaw;
        _offInit = true;
        _offMs = tMs;
        _lastSrc = SrcRtk;
        _offFrozen = false;
        return;
    }

    // 突变判定：地图参考点重设、终端重启、标定从"无"跳到"有"都会带来
    // 整体平移或转角的阶跃，此时必须直接吸附，用低通追会拖出几十厘米的假位姿。
    // 运动中把门限放宽 3 倍：否则差分滞后本身（0.1~0.5m 的错位）会被误判成突变，
    // 吸一次就把错位固化成 offset，比不更新更糟。
    const double dPos = std::sqrt((measX - _offX) * (measX - _offX)
                                  + (measY - _offY) * (measY - _offY));
    const double dYaw = std::fabs(radStdLocal(measYaw - _offYaw));
    const double snapPos = stillNow ? _p.snapM : (_p.snapM * 3.0);
    const double snapYaw = stillNow ? 0.5236 : 1.5708;   // 30° / 90°
    if (_snapNext || dPos > snapPos || dYaw > snapYaw)
    {
        const bool forced = _snapNext;   // 标定刚更新导致的吸附，不算异常，不告警
        _snapNext = false;
        if (!forced && !_bigWarned)
        {
            _bigWarned = true;
            qWarning().noquote() << QStringLiteral(
                "[Rtk] 融合偏移突变，已直接吸附 Δ=%1m Δyaw=%2°（参考点重设/标定切换/坐标系跳变）")
                .arg(dPos, 0, 'f', 2).arg(dYaw * 180.0 / kPi, 0, 'f', 1);
        }
        _offX = measX;
        _offY = measY;
        _offYaw = measYaw;
        _offFrozen = false;
        _driftSum = 0.0;
        _driftCount = 0;
        _driftWarned = false;
    }
    else if (!slowNow)
    {
        // 运动中（或转动中）冻结 offset：这时的差分值是 0.2~0.5s 前的位姿，
        // 拿它去算偏移只会把这 0.1~0.5m 的错位固化进地图，宁可整段不信。
        // 位姿仍由"里程计增量 + 冻结的 offset"推进，时间上是连续的。
        _offFrozen = true;
        _driftSum += dPos;
        ++_driftCount;
        // 冻结期间的残差就是"里程计自己走了这么久之后与绝对坐标的偏差"。
        // 均值长期偏大说明尺度/转角标定已不符（换轮、换狗、mapcalib 被改），
        // 但运动中无法直接校正，只能提示停车重标。
        if (!_driftWarned && _driftCount >= 20)
        {
            const double avg = _driftSum / _driftCount;
            if (avg > 0.5)
            {
                _driftWarned = true;
                qWarning().noquote() << QStringLiteral(
                    "[Rtk] 运动中累积偏差均值 %1m（冻结期 %2 帧）：里程计与差分已明显不符，"
                    "建议停车让静止门控重锚；若停稳后仍偏 >0.3m，删除 rtkcalib 重新标定")
                    .arg(avg, 0, 'f', 2).arg(_driftCount);
            }
        }
    }
    else
    {
        const double dt = (_offMs > 0) ? static_cast<double>(tMs - _offMs) / 1000.0 : 0.2;
        const double tau = stillNow
                               ? ((_p.offStaticTauSec > 1e-3) ? _p.offStaticTauSec : 0.3)
                               : ((_p.offSlowTauSec > 1e-3) ? _p.offSlowTauSec : 2.0);
        double a = 1.0 - std::exp(-dt / tau);
        if (a < 0.02) a = 0.02;
        if (a > 1.0) a = 1.0;
        _offX += a * (measX - _offX);
        _offY += a * (measY - _offY);
        _offYaw = radStdLocal(_offYaw + a * radStdLocal(measYaw - _offYaw));
        _offFrozen = false;
        _driftSum = 0.0;
        _driftCount = 0;
        _driftWarned = false;
    }
    _offMs = tMs;
    _lastSrc = SrcRtk;
}

bool RtkFusion::logReject(qint64 tMs, const QString &why)
{
    if (_lastRejectLogMs == 0 || tMs - _lastRejectLogMs >= 3000)
    {
        _lastRejectLogMs = tMs;
        qInfo().noquote() << QStringLiteral(
            "[Rtk] 停点标定暂不可解（%1）：锚点 %2/%3 里程 %4/%5m 样本 %6")
            .arg(why).arg(_n).arg(_p.calibMinSamples)
            .arg(_path, 0, 'f', 2).arg(_p.calibMinPathM, 0, 'f', 2)
            .arg(_samples.size());
    }
    return false;
}

bool RtkFusion::trySolve(qint64 tMs)
{
    if (!_p.autoCalib || _calib.valid)
        return false;
    if (_zeroPending)
        return false;   // 零位未落定前，样本的 dyaw 带随机角，解出来的一致性无法判断
    if (_n < _p.calibMinSamples || _path < _p.calibMinPathM)
        return logReject(tMs, QStringLiteral("锚点数或锚点里程不够"));
    if (_lastSolveMs > 0 && tMs - _lastSolveMs < 500)
        return false;
    _lastSolveMs = tMs;

    const double n = static_cast<double>(_n);
    const double cpx = _spx / n;   // 差分绝对坐标质心
    const double cpy = _spy / n;
    const double cqx = _sqx / n;   // 里程计地图坐标质心
    const double cqy = _sqy / n;

    // 去质心后的方差与互相关：p ≈ k·R(alpha)·q（复数形式 k·e^{iα} = Σ p·conj(q) / σ）
    const double sigma = _sqq - (_sqx * _sqx + _sqy * _sqy) / n;
    // 静点太集中（狗在同一个地方反复停）时几何退化，k 与 α 不可辨
    if (sigma < 1e-3)
        return logReject(tMs, QStringLiteral("锚点太集中（σ<1e-3），几何退化"));

    const double a = _spxqx + _spyqy - (_spx * _sqx + _spy * _sqy) / n;
    const double b = _spyqx - _spxqy - (_spy * _sqx - _spx * _sqy) / n;

    const double k = std::sqrt(a * a + b * b) / sigma;
    const double alpha = std::atan2(b, a);
    if (k < 0.5 || k > 2.0)
        return logReject(tMs, QStringLiteral("k=%1 越界(0.5~2.0)").arg(k, 0, 'f', 3));

    // 残差 RMS：平移取质心对质心（自由），只衡量形状一致程度
    const double cc = std::cos(alpha);
    const double ss = std::sin(alpha);
    const double tx = cpx - k * (cc * cqx - ss * cqy);
    const double ty = cpy - k * (ss * cqx + cc * cqy);
    double sum2 = 0.0;
    const int cnt = _samples.isEmpty() ? 0 : _samples.size();
    if (cnt <= 0)
        return logReject(tMs, QStringLiteral("无样本"));
    for (int i = 0; i < cnt; ++i)
    {
        const Sample &sm = _samples.at(i);
        const double rx = sm.px - (k * (cc * sm.qx - ss * sm.qy) + tx);
        const double ry = sm.py - (k * (ss * sm.qx + cc * sm.qy) + ty);
        sum2 += rx * rx + ry * ry;
    }
    const double rms = std::sqrt(sum2 / static_cast<double>(cnt));
    // 这一条以前是静默 return：样本被"里程计重锚"污染时它就是唯一的拒绝点，
    // 现场只能看到面板说"里程不足"，永远等不到结果。必须报出数值。
    if (rms > _p.calibMaxRmsM)
        return logReject(tMs, QStringLiteral("RMS=%1m 超上限 %2m（样本跨了里程计重锚或差分重启，形状对不上）")
                                 .arg(rms, 0, 'f', 3).arg(_p.calibMaxRmsM, 0, 'f', 2));

    // 合理性判据：k 应≈1（mapcalib 的尺度是主尺，这里只解残差），α 应≈0。
    // 越界不采用：一旦置 valid 并写盘，后续 trySolve 会因 valid 直接返回，错值就永久留下。
    // 所以宁可继续采样，并提示回查 mapcalib。
    if (k < _p.calibKMin || k > _p.calibKMax
        || std::fabs(alpha) * 180.0 / kPi > _p.calibAlphaMaxDeg)
    {
        if (!_bandRejectWarned)
        {
            _bandRejectWarned = true;
            qWarning().noquote() << QStringLiteral(
                "[Rtk] 停点标定结果越界：k=%1（正常 %2~%3）α=%4°（正常 ±%5°）RMS=%6m → 不采用。"
                "先回查 mapcalib 的 Scale 与 refYaw（重做一次 mapcheck/mapline，3~5m 直线），"
                "不要重复跑停点标定")
                .arg(k, 0, 'f', 3)
                .arg(_p.calibKMin, 0, 'f', 2).arg(_p.calibKMax, 0, 'f', 2)
                .arg(alpha * 180.0 / kPi, 0, 'f', 2).arg(_p.calibAlphaMaxDeg, 0, 'f', 1)
                .arg(rms, 0, 'f', 3);
        }
        return false;
    }

    Calib c;
    c.valid = true;
    c.k = k;
    c.alphaRad = alpha;
    c.alphaYawRad = radStdLocal(_sdyaw / n);
    c.rmsM = rms;
    c.pathM = _path;
    c.samples = _n;
    _calib = c;
    _snapNext = true;   // 尺度/转角一变，offset 立刻作废，下一帧直接吸附

    // 用最近一帧立即重算 offset：否则这一帧到下一帧之间（≤200 ms、最多数个
    // 20 Hz 控制节拍）会拿旧 offset 配新尺度，出现一次成比例的错位
    if (_hasLastFrame)
    {
        const double c2 = std::cos(alpha);
        const double s2 = std::sin(alpha);
        const double cx = k * (c2 * _lastOdomX - s2 * _lastOdomY);
        const double cy = k * (s2 * _lastOdomX + c2 * _lastOdomY);
        const double cyaw = radStdLocal(_lastOdomYaw + alpha);
        _offX = _lastRtkX - cx;
        _offY = _lastRtkY - cy;
        _offYaw = _p.useRtkYaw ? radStdLocal(_lastRtkYaw - cyaw) : 0.0;
        _offInit = true;
        _offMs = tMs;
        _lastSrc = SrcRtk;
    }
    return true;
}

bool RtkFusion::fusedMapPose(double odomX, double odomY, double odomYaw, qint64 tMs,
                             double *x, double *y, double *yaw, int *src) const
{
    if (!_offInit)
        return false;
    // 输出 = 里程计映射到绝对系 + 偏移（偏移每帧由差分重新测定）
    const double c = std::cos(_calib.alphaRad);
    const double s = std::sin(_calib.alphaRad);
    if (x)   *x = _calib.k * (c * odomX - s * odomY) + _offX;
    if (y)   *y = _calib.k * (s * odomX + c * odomY) + _offY;
    if (yaw) *yaw = radStdLocal(odomYaw + _calib.alphaRad + _offYaw);
    const int srcNow = ((_lastFrameMs > 0) && (tMs - _lastFrameMs <= _p.holdMs)) ? SrcRtk : SrcHold;
    if (src) *src = srcNow;
    return true;
}

bool RtkFusion::loadCalib(const QString &path, QString *message)
{
    QSettings s(path, QSettings::IniFormat);
    if (s.value("RtkCalib/Valid", false).toBool() != true)
    {
        if (message) *message = QStringLiteral("未找到已保存的差分标定，进入在线标定");
        return false;
    }
    Calib c;
    c.valid = true;
    c.k = s.value("RtkCalib/K", 1.0).toDouble();
    c.alphaRad = s.value("RtkCalib/AlphaDeg", 0.0).toDouble() * kPi / 180.0;
    c.alphaYawRad = s.value("RtkCalib/AlphaYawDeg", 0.0).toDouble() * kPi / 180.0;
    c.rmsM = s.value("RtkCalib/RmsM", 0.0).toDouble();
    c.pathM = s.value("RtkCalib/PathM", 0.0).toDouble();
    c.samples = s.value("RtkCalib/Samples", 0).toInt();
    if (c.k < 0.5 || c.k > 2.0)
    {
        if (message) *message = QStringLiteral("已保存的差分标定尺度越界，已忽略");
        return false;
    }
    _calib = c;
    if (message)
        *message = QStringLiteral("已加载差分标定: k=%1 α=%2° RMS=%3m")
                       .arg(c.k, 0, 'f', 3)
                       .arg(c.alphaRad * 180.0 / kPi, 0, 'f', 2)
                       .arg(c.rmsM, 0, 'f', 3);
    return true;
}

bool RtkFusion::saveCalib(const QString &path, QString *message) const
{
    if (!_calib.valid)
    {
        if (message) *message = QStringLiteral("差分标定尚未解出，未保存");
        return false;
    }
    QSettings s(path, QSettings::IniFormat);
    s.setValue("RtkCalib/Valid", true);
    s.setValue("RtkCalib/K", _calib.k);
    s.setValue("RtkCalib/AlphaDeg", _calib.alphaRad * 180.0 / kPi);
    s.setValue("RtkCalib/AlphaYawDeg", _calib.alphaYawRad * 180.0 / kPi);
    s.setValue("RtkCalib/RmsM", _calib.rmsM);
    s.setValue("RtkCalib/PathM", _calib.pathM);
    s.setValue("RtkCalib/Samples", _calib.samples);
    s.sync();
    if (s.status() != QSettings::NoError)
    {
        if (message) *message = QStringLiteral("差分标定写入失败: ") + path;
        return false;
    }
    if (message) *message = QStringLiteral("差分标定已保存: ") + path;
    return true;
}

QString RtkFusion::describe() const
{
    const QString zeroText = _zeroValid
        ? QStringLiteral(" ｜b=%1°(抖动%2°·%3)").arg(deg0To360(_zeroYawRad), 0, 'f', 1)
              .arg(_zeroSpreadDeg, 0, 'f', 2)
              .arg(_zeroFromLineSet ? QStringLiteral("直线") : QStringLiteral("参考"))
        : (_zeroPending ? QStringLiteral(" ｜b=待取") : QStringLiteral(" ｜b=未取"));

    if (!_calib.valid)
        return QStringLiteral("k=1 α=0°（未标定，仅吸收平移；静止锚点 %1 个 里程 %2m，需 ≥%3 个分散停点）")
                   .arg(_n).arg(_path, 0, 'f', 2).arg(_p.calibMinSamples)
               + zeroText;
    return QStringLiteral("k=%1 α=%2° 航向差=%3° RMS=%4m 静止锚点 %5 个 里程 %6m")
               .arg(_calib.k, 0, 'f', 3)
               .arg(_calib.alphaRad * 180.0 / kPi, 0, 'f', 2)
               .arg(_calib.alphaYawRad * 180.0 / kPi, 0, 'f', 2)
               .arg(_calib.rmsM, 0, 'f', 3)
               .arg(_calib.samples).arg(_calib.pathM, 0, 'f', 2)
           + zeroText;
}
