// ============================================================
// 航向稳定器 —— 实现
//
// 每帧的顺序固定：静止零偏/静止锁 → raw 跳变抑制（陀螺保持连续）
//                → 位移方向校验（直线时校核/重建基准）→ 输出 _yawCorrected
// 尺度标定只在建参考与直线标定时读写，与航向不是同一条链路，但同属"运动学标定"。
// 位姿融合见 Convert/RtkPoseTracker，导航推进见 MainControllerNav.cpp。
// ============================================================
#include "Control/YawStabilizer.h"

#include <cmath>
#include <QCoreApplication>
#include <QDebug>
#include <QSettings>

#include "Common/DogText.h"
#include "Config/TerminalConfig.h"
#include "Control/MotionState.h"
#include "Control/Navigator.h"
#include "Protocol/DogProtocol.h"

YawStabilizer::YawStabilizer(const TerminalConfig &cfg, Navigator &navigator,
                             MotionState &st, bool &velCtrlActive)
    : _cfg(cfg), _navigator(navigator), _st(st), _velCtrlActive(velCtrlActive)
{
}

void YawStabilizer::update(const DogOdom &odom, const DogImu &imu)
{
    if (!imu.valid)
        return;

    const qint64 now = nowMs();
    const double raw = imu.rpy[2];

    // 关闭稳定化时直接透传原始 yaw；
    // 原始模式（Pose/RawOdom）同样透传——现场要拿狗体数据本身去对照时，
    // 静止锁、跳变抑制、位移方向校核一律不参与，输出就是 imu.rpy[2]。
    if (!_cfg.yawStabilize || _cfg.poseRawOdom)
    {
        _yawCorrected = DogProtocol::radStd(raw);
        _yawGyro = _yawCorrected;
        return;
    }

    if (!_yawInit)
    {
        _yawInit = true;
        _yawCorrected = DogProtocol::radStd(raw);
        _yawGyro = _yawCorrected;
        _lastRawYaw = raw;
        _lastYawMs = now;
        _st.staticSinceMs = now;
        _gyroAccum = 0.0;
        return;
    }

    const double dt = static_cast<double>(now - _lastYawMs) / 1000.0;
    if (dt <= 0.0)
        return;
    // 狗端状态包常见 1Hz 更新，旧实现用 1.0s 上限会把整段转动直接丢弃，
    // 只剩偶发的短间隔帧参与判断，导致"真实快速转向被误判为跳变"。
    if (dt > _cfg.poseMaxGapSec)
    {
        // 真正断线/长间隔：丢弃增量并重锚，避免把长时间漂移当成转动
        _lastYawMs = now;
        _lastRawYaw = raw;
        _gyroAccum = 0.0;
        _yawSuspect = false;
        qWarning().noquote()
            << QStringLiteral("[Pose] 状态间隔 %1s 超过上限 %2s，航向基准已重锚于 raw yaw %3°")
                   .arg(dt, 0, 'f', 2)
                   .arg(_cfg.poseMaxGapSec, 0, 'f', 1)
                   .arg(DogProtocol::radStd(raw) * kRadToDeg, 0, 'f', 2);
        return;
    }
    _lastYawMs = now;

    // 航向角速度来源：优先 IMU gyro；固件不发 gyro 字段（gyroField=false，例如老版本
    // imu_info 只有 rpy/quat/acc）时退回里程计的机体角速度 omegaBody[2]，两者都是绕 z 的角速度。
    // 不加这条兜底，零偏估计与陀螺推进项会恒为 0，航向只能跟随 1Hz raw yaw，静止漂移压不住。
    const double gyroZ = imu.gyroField
                             ? static_cast<double>(imu.gyro[2])
                             : (odom.valid ? static_cast<double>(odom.omegaBody[2]) : 0.0);
    if (!imu.gyroField && !_gyroFallbackLogged)
    {
        _gyroFallbackLogged = true;
        qWarning().noquote()
            << QStringLiteral("[Pose] imu_info 未带 gyro 字段，航向角速度改用 odom.omega_body[2]");
    }
    const double vBody = odom.valid
                             ? std::sqrt(static_cast<double>(odom.vBody[0]) * odom.vBody[0]
                                         + static_cast<double>(odom.vBody[1]) * odom.vBody[1])
                             : 0.0;
    // 导航/速度直控期间不能按"原地慢转"判静止：否则陀螺读数会被当成零偏，
    // 真实的慢速转向会被从 yaw 增量里扣掉，表现为 turnJoy 饱和但朝向几乎不变
    const bool controlActive = _navigator.isActive() || _velCtrlActive;
    // 速度阈值必须与"静止"的统一口径一致（RtkFusion 的 offStaticSpeed 默认 0.03）。
    // 原来卡 0.02：四足站立时 vBody 的噪声常年在 0.03 上下，静止判据于是永不成立，
    // 零偏一辈子学不到（日志里"零偏=0.000°/s"恒 0 就是这个原因），静止锁也永不触发。
    const bool still = !controlActive && odom.valid && (vBody < 0.05)
                       && (std::fabs(gyroZ) < 0.05);

    if (still)
    {
        // 静止时 gyro 读数≈零偏，低通跟踪
        _st.yawBias = _st.yawBias * 0.95 + static_cast<double>(gyroZ) * 0.05;
        if (!_staticLocked && _cfg.staticLockSec > 0.0
            && (now - _st.staticSinceMs) >= static_cast<qint64>(_cfg.staticLockSec * 1000.0))
        {
            _staticLocked = true;
            qInfo().noquote()
                << QStringLiteral("[Pose] 静止 %1s，yaw 已锁定（零偏≈%2°/s），漂移不再累计")
                       .arg(_cfg.staticLockSec, 0, 'f', 1)
                       .arg(_st.yawBias * kRadToDeg, 0, 'f', 3);
        }
        // 静止且已锁定：此时陀螺无转动证据，raw 若与连续朝向偏离很大，只可能是 IMU
        // 内部重锚（或滑移后的新基准），继续坚持旧值会让"待校核"永远学不回来。
        if (_staticLocked)
        {
            const double gap0 = std::fabs(DogProtocol::radStd(DogProtocol::radStd(raw) - _yawCorrected));
            if (gap0 > 20.0 / kRadToDeg)
            {
                qWarning().noquote()
                    << QStringLiteral("[Pose] 静止期间 raw 基准变化 %1°（陀螺无转动证据），已重锚并解除可疑")
                           .arg(gap0 * kRadToDeg, 0, 'f', 1);
                _yawCorrected = DogProtocol::radStd(raw);
                _yawGyro = _yawCorrected;
                _yawSuspect = false;
                _lastRawYaw = raw;
                _gyroAccum = 0.0;
                return;
            }
        }
    }
    else
    {
        _staticLocked = false;
        _st.staticSinceMs = now;
    }

    // ---- 连续传播：陀螺积分推进，不依赖 raw 是否更新 ----
    // 零偏仅在合理范围内扣除，异常估计（如刚起步的 ±2°/s）不参与，避免反而带偏
    double biasRate = _st.yawBias;
    if (std::fabs(biasRate) > 1.0)
        biasRate = 0.0;
    const double g = (gyroZ - biasRate) * dt;
    _gyroAccum += g;
    if (!_staticLocked)
        _yawGyro = DogProtocol::radStd(_yawGyro + g);

    const bool rawChanged = (std::fabs(raw - _lastRawYaw) > 1e-9);
    if (!rawChanged)
    {
        // raw 未更新（狗端 1Hz 刷新 + 高频状态包，或 imu 停更）：跟陀螺推算即可
        if (!_staticLocked)
            _yawCorrected = _yawGyro;
        return;
    }

    const double d = DogProtocol::radStd(raw - _lastRawYaw);
    // 帧间微抖动（0.2° 量级）不构成可校验事件：既不清零陀螺累加器，也不重锚。
    // 现场 raw 每帧都有 0.01~0.1° 的小变化，若每次变化都清零累加器，1 s 的真实转角
    // （数十度）到达时累加器只剩几毫秒的积分，必然被误判成跳变——这是旧实现
    // "每帧都丢"的直接放大来源。
    if (std::fabs(d) <= 0.2 / kRadToDeg
        && std::fabs(_gyroAccum) < _cfg.yawJumpDeg / kRadToDeg)
    {
        _lastRawYaw = raw;
        if (!_staticLocked)
            _yawCorrected = _yawGyro;
        return;
    }
    const double expect = _gyroAccum;
    const double dev = std::fabs(DogProtocol::radStd(d - expect));
    // 一致性容差：绝对下限为 yawJumpDeg，另留 4 倍陀螺积分的比例余量。
    // 真实快速转向时 d≈expect（哪怕单帧几十度）会被接受；而 IMU 基准重置时
    // raw 突变但陀螺没转，dev 会远超容差。
    const double tol = qMax(_cfg.yawJumpDeg / kRadToDeg,
                            4.0 * std::fabs(expect)) + 0.05;

    _lastRawYaw = raw;
    _gyroAccum = 0.0;

    if (dev <= tol)
    {
        // 可疑期间即使增量一致，也不能直接重锚到 raw：滑移后的 raw 会整体偏移，
        // 直接锚回会把刚才用陀螺保住的连续性又拉回去。只有 raw 已回到连续朝向附近，
        // 或位移方向已确认，才允许重锚。
        const double gap = std::fabs(DogProtocol::radStd(DogProtocol::radStd(raw) - _yawCorrected));
        if (_yawSuspect && gap > 12.0 / kRadToDeg)
        {
            _yawCorrected = _yawGyro;
            qInfo().noquote()
                << QStringLiteral("[Pose] raw 与连续朝向仍相差 %1°，暂不重锚（航向待校核）")
                       .arg(gap * kRadToDeg, 0, 'f', 1);
            return;
        }
        if (_yawSuspect)
        {
            qInfo().noquote()
                << QStringLiteral("[Pose] 航向基准已与陀螺一致（偏差 %1°），解除可疑")
                       .arg(dev * kRadToDeg, 0, 'f', 1);
            _yawSuspect = false;
        }
        // raw 为绝对基准：一致时向 raw 收敛，抑制陀螺长期漂移。
        // 但不能再"每帧硬重锚到 raw"：那样陀螺积分（含零偏补偿）每帧都被丢掉，
        // 稳定yaw 退化成 raw 的复读机——日志里"稳定yaw==陀螺yaw==raw yaw"逐帧全等、
        // 而地图 yaw 仍按 raw 的 0.2~0.3°/s 漂，就是这条造成的。
        // 小幅差按 10%/帧 收敛（20Hz 下 τ≈0.5s）；超过 5° 才认定 IMU 基准真跳了，直接重锚。
        const double gapC = DogProtocol::radStd(DogProtocol::radStd(raw) - _yawGyro);
        if (std::fabs(gapC) > 5.0 / kRadToDeg)
            _yawGyro = DogProtocol::radStd(raw);
        else
            _yawGyro = DogProtocol::radStd(_yawGyro + 0.10 * gapC);
        if (!_staticLocked)
            _yawCorrected = _yawGyro;
        return;
    }

    // ---- 不一致：用"直线位移方向"做独立裁决，而不是一律忽略 ----
    const bool dispFresh = _dispYawValid && (now - _dispYawMs) <= 5000;
    if (dispFresh)
    {
        const double devRaw  = std::fabs(DogProtocol::radStd(DogProtocol::radStd(raw) - _dispYaw));
        const double devCont = std::fabs(DogProtocol::radStd(_yawCorrected - _dispYaw));
        qWarning().noquote()
            << QStringLiteral("[Pose] yaw 跳变 %1°（陀螺积分 %2°，偏差 %3°）："
                              "位移方向 %4°，raw 偏 %5°，当前偏 %6° → %7")
                   .arg(d * kRadToDeg, 0, 'f', 1)
                   .arg(expect * kRadToDeg, 0, 'f', 1)
                   .arg(dev * kRadToDeg, 0, 'f', 1)
                   .arg(_dispYaw * kRadToDeg, 0, 'f', 1)
                   .arg(devRaw * kRadToDeg, 0, 'f', 1)
                   .arg(devCont * kRadToDeg, 0, 'f', 1)
                   .arg(devRaw <= devCont ? QStringLiteral("采用 raw（更符合实际运动方向）")
                                          : QStringLiteral("保持陀螺推算（raw 与实际运动方向不符）"));
        if (devRaw <= devCont)
        {
            _yawGyro = DogProtocol::radStd(raw);
            _yawCorrected = _yawGyro;
            _yawSuspect = false;
        }
        else
        {
            _yawCorrected = _yawGyro;   // 保留这段时间陀螺积分的真实转动
            _yawSuspect = true;
            _suspectSinceMs = now;
        }
        return;
    }

    // 无位移证据：保持陀螺推算的连续朝向并标记可疑，等下一段直线位移裁决
    _yawCorrected = _yawGyro;
    _yawSuspect = true;
    _suspectSinceMs = now;
    qWarning().noquote()
        << QStringLiteral("[Pose] 检测到 yaw 跳变 %1°（陀螺积分 %2°，偏差 %3°），"
                          "暂按陀螺推算保持连续朝向 %4°（待直线位移校核）")
               .arg(d * kRadToDeg, 0, 'f', 1)
               .arg(expect * kRadToDeg, 0, 'f', 1)
               .arg(dev * kRadToDeg, 0, 'f', 1)
               .arg(_yawCorrected * kRadToDeg, 0, 'f', 1);
}

void YawStabilizer::updateMotionHeading(const DogOdom &odom, const DogImu &imu)
{
    // 原始模式：不用位移方向去改航向基准
    if (_cfg.poseRawOdom)
    {
        _mwActive = false;
        _dispYawValid = false;
        _yawSuspect = false;
        return;
    }
    if (!_cfg.poseDispAssist || !odom.valid || !imu.valid)
    {
        _mwActive = false;
        return;
    }

    const double vx = odom.vBody[0];
    const double vy = odom.vBody[1];
    const double speed = std::sqrt(vx * vx + vy * vy);
    const double wyaw = imu.gyro[2];
    const double lx = odom.position[0];
    const double ly = odom.position[1];

    // 只取"向前、基本走直、角速度小"的片段：此时位移方向才代表机头朝向
    const bool straight = (speed >= _cfg.poseDispMinSpeed) && (vx > 0.0)
                          && (std::fabs(vy) <= qMax(0.15, 0.35 * std::fabs(vx)))
                          && (std::fabs(wyaw) <= 0.20);
    if (!straight)
    {
        _mwActive = false;
        return;
    }

    if (!_mwActive)
    {
        _mwActive = true;
        _mwStartX = _mwPrevX = lx;
        _mwStartY = _mwPrevY = ly;
        _mwLen = 0.0;
        _mwStartMs = nowMs();
        return;
    }

    const double step = std::sqrt((lx - _mwPrevX) * (lx - _mwPrevX)
                                  + (ly - _mwPrevY) * (ly - _mwPrevY));
    if (step > 0.6)
    {
        // 单帧位移过大（里程计重置），本窗口作废
        _mwActive = false;
        return;
    }
    _mwLen += step;
    _mwPrevX = lx;
    _mwPrevY = ly;
    // 窗口超时保护：长时间低速蠕动时重置，避免把跨越多段弯路的位移当成一条直线
    if ((nowMs() - _mwStartMs) > 15000)
    {
        _mwActive = false;
        return;
    }
    if (_mwLen < _cfg.poseDispMinDist)
        return;

    const double dx = lx - _mwStartX;
    const double dy = ly - _mwStartY;
    const double chord = std::sqrt(dx * dx + dy * dy);
    if (chord < 0.5 * _cfg.poseDispMinDist)
    {
        // 绕圈/来回：合位移方向不可信
        _mwActive = false;
        return;
    }

    const double dispYaw = std::atan2(dy, dx);
    const double res = DogProtocol::radStd(dispYaw - _yawCorrected);
    _dispYaw = dispYaw;
    _dispYawValid = true;
    _dispYawMs = nowMs();

    const double reanchorRad = _cfg.poseDispReanchorDeg / kRadToDeg;
    const double correctRad = _cfg.poseDispCorrectDeg / kRadToDeg;

    if (_yawSuspect || std::fabs(res) > reanchorRad)
    {
        qWarning().noquote()
            << QStringLiteral("[Pose] 依据直线位移重建航向：位移方向 %1°，原航向 %2°"
                              "（偏差 %3°），位移 %4m → 已采用位移方向")
                   .arg(dispYaw * kRadToDeg, 0, 'f', 1)
                   .arg(_yawCorrected * kRadToDeg, 0, 'f', 1)
                   .arg(res * kRadToDeg, 0, 'f', 1)
                   .arg(_mwLen, 0, 'f', 2);
        _yawCorrected = DogProtocol::radStd(dispYaw);
        _yawGyro = _yawCorrected;
        _yawSuspect = false;
    }
    else if (std::fabs(res) > correctRad)
    {
        // 小幅偏差多为打滑/侧向漂移，按 20% 渐进修正，不做硬跳
        qInfo().noquote()
            << QStringLiteral("[Pose] 位移方向校核：偏差 %1°，轻度修正 %2°（位移 %3m）")
                   .arg(res * kRadToDeg, 0, 'f', 1)
                   .arg(0.2 * res * kRadToDeg, 0, 'f', 1)
                   .arg(_mwLen, 0, 'f', 2);
        _yawCorrected = DogProtocol::radStd(_yawCorrected + 0.2 * res);
        _yawGyro = _yawCorrected;
    }

    _mwActive = false;
}

void YawStabilizer::loadCalibScale()
{
    const QString path = QCoreApplication::applicationDirPath()
                         + QStringLiteral("/mapcalib_%1.ini").arg(_cfg.terminalId);
    QSettings s(path, QSettings::IniFormat);
    const double v = s.value(QStringLiteral("Map/Scale"), 1.0).toDouble();
    _calibScale = (v > 1e-6) ? v : 1.0;
    if (qAbs(_calibScale - 1.0) > 1e-6)
        qInfo().noquote() << QStringLiteral("[Map] 已加载持久化标定尺度 %1（%2）")
                                 .arg(_calibScale, 0, 'f', 4)
                                 .arg(path);
}

void YawStabilizer::saveCalibScale(double scale)
{
    if (scale <= 1e-6)
        return;
    _calibScale = scale;
    const QString path = QCoreApplication::applicationDirPath()
                         + QStringLiteral("/mapcalib_%1.ini").arg(_cfg.terminalId);
    QSettings s(path, QSettings::IniFormat);
    s.setValue(QStringLiteral("Map/Scale"), scale);
    s.sync();
}
