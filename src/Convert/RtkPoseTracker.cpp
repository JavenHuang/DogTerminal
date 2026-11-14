// ============================================================
// 差分(RTK)位姿跟踪器 —— 实现
//
// 一帧的路子：读取器读出载荷 → 跳变门控放行 → 观测原点/统计 → 送入 RtkFusion
//            → 1Hz 汇总 + 面板快照
// 只做这一件事：把外来的绝对位姿变成终端可用的位姿源。
// 载荷解析(JSON/二进制/ID 过滤/前几帧打印)见 Convert/RtkFrameReader.cpp，
// 跳变与模块重启判据见 Control/RtkJumpGate.cpp。
// 导航推进见 MainControllerNav.cpp，上报打包见 MainController.cpp。
// ============================================================
#include "Convert/RtkPoseTracker.h"

#include <cmath>
#include "Common/DogText.h"
#include <QByteArray>
#include <QCoreApplication>
#include <QDebug>

#include "Config/TerminalConfig.h"
#include "Control/MotionState.h"
#include "Convert/MapTransform.h"
#include "Protocol/DogProtocol.h"
#include "ServerLink/CalibPanel.h"

// ============================================================
// 差分(RTK)绝对位姿（MQ 路由键 la / 协议消息 0x1A）
// 只接收 → 换算 → 与终端位姿比对 → 打印，不改导航位姿来源、不参与任何控制。
// 现场要确认三件事：
//   1) 200ms 周期是否稳定、最大间隔多少、有没有成片空洞；
//   2) 坐标口径是否正确 —— x 东、y 南，角度 度、正东 0、顺时针；
//   3) 差分航向与里程计推算航向的固定夹角 θ（自动对齐要解的就是它）。
// 载荷形态(JSON / 二进制 AbsPoseCmd)与前几帧原样打印在 RtkFrameReader 里。
// ============================================================

RtkPoseTracker::RtkPoseTracker(const TerminalConfig &cfg, MapTransform &map,
                               RtkFusion &fusion, MotionState &state)
    : _cfg(cfg), _map(map), _fusion(fusion), _st(state), _jump(cfg, fusion)
{
}

void RtkPoseTracker::onFrame(const QByteArray &body)
{
    if (!_cfg.rtkEnabled)
        return;   // 未启用：连解析都不做，避免无谓开销与误判

    DogProtocol::AbsPoseCmd cmd;
    if (!_reader.parse(body, _cfg.rtkDiffId, &cmd))
        return;   // 载荷非法/ID 不匹配都在读取器里判掉并已打日志

    const qint64 recvMs = nowMs();
    if (!_rtkSeen)
    {
        _rtkSeen = true;
        _rtkStartMs = recvMs;
    }
    else
    {
        _rtkLastInterMs = recvMs - _rtkRecvMs;
        if (_rtkLastInterMs > _rtkMaxGapMs)
            _rtkMaxGapMs = _rtkLastInterMs;
    }
    _rtkRecvMs = recvMs;
    _rtkStaleWarned = false;
    ++_rtkFrames;

    // 航向：服务端已是"度、正东 0、顺时针"，与地图 yaw 同口径，只需转弧度并归一化
    const double yawOffsetRad = _cfg.rtkYawOffsetDeg / kRadToDeg;
    const double yawMap = DogProtocol::radStd(cmd.yawDeg / kRadToDeg + yawOffsetRad);

    // 航向观测范围：服务端若一直给同一个角度（典型是恒 0），说明它没在算航向，
    // 此时绝对锚定会把狗的朝向钉死，必须看这一项才能判断
    const double yawNormDeg = degNorm360(yawMap * kRadToDeg);
    if (yawNormDeg < _rtkYawMinDeg) _rtkYawMinDeg = yawNormDeg;
    if (yawNormDeg > _rtkYawMaxDeg) _rtkYawMaxDeg = yawNormDeg;

    // 安装偏置：天线相位中心 → 狗体中心（机体系 前/左 → 地图系）
    // 地图系下机头单位向量 = (cos, sin)，机左单位向量 = (sin, -cos)（y 轴指南）
    const double ax = _cfg.rtkAntennaX;
    const double ay = _cfg.rtkAntennaY;
    const double cx = cmd.x - ax * std::cos(yawMap) - ay * std::sin(yawMap);
    const double cy = cmd.y - ax * std::sin(yawMap) + ay * std::cos(yawMap);

    // 同一帧的里程计地图 yaw：判"狗是不是真在转"的唯一佐证
    double omyawNow = 0.0;
    const bool hasOdomNow = (_map.isActive() && _st.valid);
    if (hasOdomNow)
    {
        double omx = 0.0, omy = 0.0;
        _map.toMap(_st.x, _st.y, _st.yaw, &omx, &omy, &omyawNow);
    }

    // 单帧跳变门控：位置野值、航向真转向/模块重启/轴向错三分支全在 RtkJumpGate 里判
    if (_rtkOriginValid)
    {
        if (!_jump.accept(cx, cy, yawMap, recvMs, _rtkX, _rtkY, _rtkYaw,
                          hasOdomNow, omyawNow))
            return;
    }

    // 同一时刻的里程计地图位姿：融合的参照点。
    // 若地图参考尚未建立（启动初期），本帧只做观测，不建立融合偏移
    if (_map.isActive() && _st.valid)
    {
        // 地图参考系的 refYaw 交给融合：标定解出的 α 是"差分系→地图系"的旋转，
        // 其理论值就是 -refYaw 附近。之后 α 的判据变成"相对 refYaw 的残差"，
        // 不再要求 α 落在 ±5°（否则每次重建参考都把 α 顶到 -90°~-108° 直接被拒）。
        _fusion.setMapRefYawRad(_map.refYaw(), _map.isActive());

        double omx = 0.0;
        double omy = 0.0;
        double omyaw = 0.0;
        _map.toMap(_st.x, _st.y, _st.yaw, &omx, &omy, &omyaw);
        _fusion.onRtkFrame(cx, cy, yawMap, omx, omy, omyaw, recvMs,
                           _st.speed, _st.yawRate);
    }

    // 首帧打一条基线日志后置位；地图/上报坐标一律取差分原值，
    // 不做整体平移（服务端给 2,2 就是 2,2），所以不需要留存首帧数值
    if (!_rtkOriginValid)
    {
        _rtkOriginValid = true;
        qInfo().noquote() << QStringLiteral(
            "[Rtk] 首帧差分位姿已接收 id=%1 ENU(%2, %3) yaw=%4°")
            .arg(cmd.diffId)
            .arg(cx, 0, 'f', 3).arg(cy, 0, 'f', 3)
            .arg(degNorm360(yawMap * kRadToDeg), 0, 'f', 1);
    }

    _rtkX = cx;
    _rtkY = cy;
    _rtkYaw = yawMap;

    // 记录本帧被接受时的里程计地图 yaw：下一帧判"真转向 / 假重启"的基准
    _jump.noteAccepted(hasOdomNow, omyawNow);

    // 在线标定：样本帧数与行走距离都达标时解一次（失败就等下一次）
    if (_fusion.trySolve(recvMs))
    {
        QString msg;
        _fusion.saveCalib(calibPath(), &msg);
        const RtkFusion::Calib c = _fusion.calib();
        qInfo().noquote() << QStringLiteral("[Rtk] 在线标定完成 %1（%2）")
                                 .arg(_fusion.describe()).arg(msg);
        // 一致性校核：位置拟合出的转角应与"航向差均值"一致；
        // 差太多说明航向口径或天线安装角有问题，此时位姿会持续偏
        const double dA = DogProtocol::radStd(c.alphaRad - c.alphaYawRad) * kRadToDeg;
        if (_cfg.rtkYawFromRtk && std::fabs(dA) > 5.0)
            qWarning() << "[Rtk] 标定一致性告警：位置解转角与航向差相差" << dA
                       << "°，请核对航向口径与天线安装角";
    }

    const QString calibText = _fusion.calib().valid
        ? QStringLiteral("k=%1 α=%2°").arg(_fusion.calib().k, 0, 'f', 3)
              .arg(_fusion.calib().alphaRad * kRadToDeg, 0, 'f', 2)
        : QStringLiteral("未解");
    const QString srcText = (_navPoseSrc == RtkFusion::SrcRtk)
                                ? QStringLiteral("差分")
                                : ((_navPoseSrc == RtkFusion::SrcHold)
                                       ? QStringLiteral("差分保持") : QStringLiteral("里程计"));
    // 静止门控状态：运动中 offset 被冻结，差分滞后不进地图
    const QString gateText = _fusion.offsetFrozen()
        ? QStringLiteral("冻结(残差%1m)").arg(_fusion.frozenDriftM(), 0, 'f', 2)
        : QStringLiteral("跟踪");

    // 1Hz 汇总打印：与终端当前地图位姿比对，输出位置差、航向差与帧周期统计
    if (recvMs - _lastRtkPrintMs < 1000)
        return;
    _lastRtkPrintMs = recvMs;

    const double avgPeriod = (_rtkFrames > 1)
                                 ? static_cast<double>(recvMs - _rtkStartMs) / (_rtkFrames - 1)
                                 : 0.0;

    double mapX = 0.0, mapY = 0.0, mapYaw = 0.0;
    const bool hasMap = (_map.isActive() && _st.valid);
    if (hasMap)
        _map.toMap(_st.x, _st.y, _st.yaw, &mapX, &mapY, &mapYaw);

    // 航向零位 b 的状态（会话量：建参考时取，重启终端/模块重启时重取）
    const double bDeg = degNorm360(_fusion.yawZeroRad() * kRadToDeg);
    const QString zeroText = _fusion.hasYawZero()
                                 ? QStringLiteral("b=%1°·%2").arg(bDeg, 0, 'f', 1)
                                       .arg(_fusion.yawZeroFromLine() ? QStringLiteral("线")
                                                                      : QStringLiteral("参考"))
                                 : (_fusion.yawZeroPending() ? QStringLiteral("b=待取")
                                                             : QStringLiteral("b=未取"));

    // 归一化后的差分航向（扣掉零位 b）：参考姿态处应≈0；与融合 yaw 比对才有意义
    const double yawZeroed = DogProtocol::radStd(yawMap - _fusion.yawZeroRad());
    const double yawZeroNormDeg = degNorm360(yawZeroed * kRadToDeg);

    // Δψ 增量校核：差分航向增量 vs 里程计航向增量（只诊断，不干预控制）。
    // 绝对航向可以靠 b 归零，但增量一致性只能靠这一项巡出"符号反了/漏转 90°"。
    QString dpsiText = QStringLiteral("Δψ=n/a");
    double dpsiDeg = 0.0;
    bool dpsiOk = false;   // 本窗口的 Δψ 是否可信（yaw 活着且窗口没在真转）
    if (hasMap && _rtkPrevCheckMs > 0)
    {
        const double dtSec = static_cast<double>(recvMs - _rtkPrevCheckMs) / 1000.0;
        const double dRtk = DogProtocol::radStd(yawMap - _rtkPrevRtkYawRad);
        const double dOdom = DogProtocol::radStd(mapYaw - _rtkPrevMapYawRad);
        dpsiDeg = std::fabs(DogProtocol::radStd(dRtk - dOdom)) * kRadToDeg;
        // 服务端不算航向时 yaw 恒定，Δψ 没有意义（会给一个假的 0），明确标不可用
        const bool yawAlive = (_rtkYawMaxDeg - _rtkYawMinDeg) > 1.0;
        // 窗口内里程计真转过（>6°）时，差分航向的保持/台阶本身就会造成偏差，
        // 此时的 Δψ 测的是差分刷新节奏，不是符号或轴向错误——不判、不告警。
        const double dOdomWindowDeg = std::fabs(dOdom) * kRadToDeg;
        const bool windowRotating = (dOdomWindowDeg > 6.0);
        dpsiOk = (yawAlive && !windowRotating);
        if (!yawAlive)
            dpsiText = QStringLiteral("Δψ=n/a(yaw恒定)");
        else if (windowRotating)
            dpsiText = QStringLiteral("Δψ=%1°/%2s(转%3°不判)")
                           .arg(dpsiDeg, 0, 'f', 2).arg(dtSec, 0, 'f', 1)
                           .arg(dOdomWindowDeg, 0, 'f', 1);
        else
            dpsiText = QStringLiteral("Δψ=%1°/%2s%3")
                           .arg(dpsiDeg, 0, 'f', 2).arg(dtSec, 0, 'f', 1)
                           .arg(dpsiDeg > 2.0 ? QStringLiteral("(偏大)") : QString());
        if (yawAlive && !windowRotating && dpsiDeg > 10.0 && (recvMs - _rtkDpsiWarnMs) > 10000)
        {
            _rtkDpsiWarnMs = recvMs;
            qWarning().noquote() << QStringLiteral(
                "[Rtk] Δψ 校核超限：差分航向增量与里程计相差 %1°（最近 %2s），"
                "先做四方向核对（东0°/南90°/西180°/北270°），确认符号与轴向后再标定")
                .arg(dpsiDeg, 0, 'f', 1).arg(dtSec, 0, 'f', 1);
        }
    }
    _rtkPrevRtkYawRad = yawMap;
    _rtkPrevMapYawRad = mapYaw;
    _rtkPrevCheckMs = recvMs;

    // 打印顺序：服务端原值 → 补偿后 → 里程计地图位姿（含与差分的差）→ 实际上报值
    // 上报值必须与差分原值同一点位（差分给 2,2 就报 2,2），对差≈0 即证明链路正确
    const QString srvText = QStringLiteral("服务端 id=%1 x=%2 y=%3 z=%4 yaw=%5°")
                                .arg(cmd.diffId)
                                .arg(cmd.x, 0, 'f', 3).arg(cmd.y, 0, 'f', 3)
                                .arg(cmd.z, 0, 'f', 3).arg(cmd.yawDeg, 0, 'f', 2);

    double fmx = 0.0, fmy = 0.0, fmyaw = 0.0;
    int fsrc = RtkFusion::SrcNone;
    const bool hasFused = fusedMapPose(&fmx, &fmy, &fmyaw, &fsrc);
    const double fusedDX = hasFused ? (fmx - cx) : 0.0;
    const double fusedDY = hasFused ? (fmy - cy) : 0.0;
    const double fusedDYaw = hasFused ? DogProtocol::radStd(fmyaw - yawZeroed) * kRadToDeg : 0.0;

    if (hasMap)
    {
        const double dxm = cx - mapX;
        const double dym = cy - mapY;
        const double dpos = std::sqrt(dxm * dxm + dym * dym);
        const double dyawDeg = DogProtocol::radStd(yawMap - mapYaw) * kRadToDeg;
        _rtkBiasSumDeg += dyawDeg;
        ++_rtkBiasCount;
        const double biasAvg = _rtkBiasSumDeg / _rtkBiasCount;

        // α 松动监视：bias 是"差分航向 vs 里程计地图航向"的常值偏差。
        // 判据用低通值而非累计均值：累计均值会被历史稀释（重新标定后正常了
        // 也长期停在高位），低通值才反映"现在"这个安装角是否还对得上。
        // 5Hz 差分帧下 τ≈20 帧（约 4s）。
        _rtkBiasLpfDeg += (dyawDeg - _rtkBiasLpfDeg) * 0.05;

        if (_cfg.rtkYawBiasWarnDeg > 0.0 && _rtkBiasCount >= 20
            && std::fabs(_rtkBiasLpfDeg) > _cfg.rtkYawBiasWarnDeg)
        {
            if (!_rtkBiasWarned)
            {
                _rtkBiasWarned = true;
                qWarning().noquote() << QStringLiteral(
                    "[Rtk] 航向常值偏差 当前=%1°（阈值 %2°，累计均值=%3°，已 %4 帧）："
                    "先查天线安装角 α 是否松动或被撞偏，再重做直线标定/停点标定刷新 α 与零位")
                    .arg(_rtkBiasLpfDeg, 0, 'f', 1)
                    .arg(_cfg.rtkYawBiasWarnDeg, 0, 'f', 1)
                    .arg(biasAvg, 0, 'f', 1)
                    .arg(_rtkBiasCount);
            }
        }
        else if (_rtkBiasWarned && std::fabs(_rtkBiasLpfDeg) < _cfg.rtkYawBiasWarnDeg * 0.5)
        {
            // 迟滞：回落到阈值一半以下才解除，避免在阈值附近来回刷屏
            _rtkBiasWarned = false;
            qInfo().noquote() << QStringLiteral("[Rtk] 航向常值偏差已回落至 当前=%1°，α 监视恢复")
                                     .arg(_rtkBiasLpfDeg, 0, 'f', 1);
        }

        qInfo().noquote() << QStringLiteral(
            "[Rtk] 帧=%1 周期=%2ms 最大间隔=%3ms 航向范围=%4° 均值=%5° | %6 | "
            "补偿后(%7, %8) yaw=%9° | 里程计地图(%10, %11) yaw=%12° 差=%13m/%14° | "
            "上报(差分锚定)(%15, %16) yaw=%17° 对差=%18m/%19° | 跳变=%20 | 速度=%21m/s 偏移=%22 | "
            "锚点=%23 定标=%24 源=%25 零位=%26 %27")
            .arg(_rtkFrames)
            .arg(avgPeriod, 0, 'f', 0).arg(_rtkMaxGapMs)
            .arg(_rtkYawMaxDeg - _rtkYawMinDeg, 0, 'f', 1)
            .arg(biasAvg, 0, 'f', 1)
            .arg(srvText)
            .arg(cx, 0, 'f', 2).arg(cy, 0, 'f', 2).arg(yawZeroNormDeg, 0, 'f', 1)
            .arg(mapX, 0, 'f', 2).arg(mapY, 0, 'f', 2)
            .arg(degNorm360(mapYaw * kRadToDeg), 0, 'f', 1)
            .arg(dpos, 0, 'f', 2).arg(dyawDeg, 0, 'f', 1)
            .arg(fmx, 0, 'f', 2).arg(fmy, 0, 'f', 2)
            .arg(degNorm360(fmyaw * kRadToDeg), 0, 'f', 1)
            .arg(std::sqrt(fusedDX * fusedDX + fusedDY * fusedDY), 0, 'f', 2)
            .arg(fusedDYaw, 0, 'f', 1)
            .arg(_jump.badJumps())
            .arg(_fusion.lastSpeed(), 0, 'f', 2)
            .arg(gateText)
            .arg(_fusion.staticPoints())
            .arg(calibText).arg(srcText)
            .arg(zeroText).arg(dpsiText);
    }
    else
    {
        qInfo().noquote() << QStringLiteral(
            "[Rtk] 帧=%1 周期=%2ms 最大间隔=%3ms 航向范围=%4° | %5 | "
            "补偿后(%6, %7) yaw=%8° | 地图参考未建立，暂不比对 | 定标=%9 源=%10 零位=%11")
            .arg(_rtkFrames)
            .arg(avgPeriod, 0, 'f', 0).arg(_rtkMaxGapMs)
            .arg(_rtkYawMaxDeg - _rtkYawMinDeg, 0, 'f', 1)
            .arg(srvText)
            .arg(cx, 0, 'f', 2).arg(cy, 0, 'f', 2).arg(yawNormDeg, 0, 'f', 1)
            .arg(calibText).arg(srcText).arg(zeroText);
    }

    // ---- 标定面板：把上面这些量压成固定 8 行，1Hz 发到 MQ 供服务端常驻显示 ----
    // 判定（下一步该做什么）在这里当场算，服务端不需要懂标定逻辑，只显示。
    if (_panel)
    {
        CalibPanel::Snapshot ps;
        ps.nowMs        = recvMs;
        ps.terminalId   = _cfg.terminalId;
        ps.rtkEnabled   = _cfg.rtkEnabled;
        ps.autoCalib    = _cfg.rtkAutoCalib;
        ps.rtkSeen      = _rtkSeen;
        ps.periodMs     = static_cast<qint64>(avgPeriod);
        ps.gapMaxMs     = _rtkMaxGapMs;
        ps.frames       = _rtkFrames;
        ps.badJumps     = _jump.badJumps();
        ps.restarts     = _jump.restarts();
        ps.yawRangeDeg  = _rtkYawMaxDeg - _rtkYawMinDeg;
        ps.biasAvgDeg   = (_rtkBiasCount > 0) ? (_rtkBiasSumDeg / _rtkBiasCount) : 0.0;
        ps.rtkX         = cx;
        ps.rtkY         = cy;
        ps.rtkYawDeg    = degNorm360(yawMap * kRadToDeg);

        ps.zeroValid    = _fusion.hasYawZero();
        ps.zeroPending  = _fusion.yawZeroPending();
        ps.zeroFromLine = _fusion.yawZeroFromLine();
        ps.bDeg         = bDeg;
        ps.bSpreadDeg   = _fusion.yawZeroSpreadDeg();

        const RtkFusion::Calib rc = _fusion.calib();
        ps.calibValid   = rc.valid;
        ps.k            = rc.k;
        ps.alphaDeg     = rc.alphaRad * kRadToDeg;
        ps.alphaYawDeg  = rc.alphaYawRad * kRadToDeg;
        ps.rmsM         = rc.rmsM;
        // 已解时用"解算时用了多少里程"，未解时必须用实时累计里程（pathM()）。
        // 原来这里恒取 rc.pathM，而未解时它是 0，面板于是显示"里程不足 0.00/0.80m"，
        // 把"样本污染/RMS 超限"错报成"再多走点"——现场走 26m 也解不出来就是这个坑。
        ps.pathM        = rc.valid ? rc.pathM : _fusion.pathM();
        ps.anchors      = _fusion.staticPoints();

        ps.hasMap       = hasMap;
        ps.mapX         = mapX;
        ps.mapY         = mapY;
        ps.mapYawDeg    = degNorm360(mapYaw * kRadToDeg);
        ps.hasFused     = hasFused;
        ps.fusedX       = fmx;
        ps.fusedY       = fmy;
        ps.fusedYawDeg  = degNorm360(fmyaw * kRadToDeg);
        ps.fusedVsRtkM  = std::sqrt(fusedDX * fusedDX + fusedDY * fusedDY);
        ps.fusedVsRtkYawDeg = fusedDYaw;
        ps.src          = fsrc;
        ps.useForPose   = _cfg.rtkUseForPose;
        ps.speedMps     = _fusion.lastSpeed();
        ps.staticSec    = (_st.staticSinceMs > 0)
                              ? (static_cast<double>(recvMs - _st.staticSinceMs) / 1000.0)
                              : 0.0;
        ps.gyroBiasDps  = _st.yawBias * kRadToDeg;
        ps.dpsiDeg      = dpsiDeg;
        ps.dpsiValid    = dpsiOk;

        ps.calibMinSamples = _cfg.rtkCalibMinSamples;
        ps.calibMinPathM   = _cfg.rtkCalibMinPathM;
        ps.calibMaxRmsM    = _cfg.rtkCalibMaxRmsM;
        ps.holdMs          = _cfg.rtkHoldMs;

        _panel->feed(ps);
    }
}

bool RtkPoseTracker::fusedMapPose(double *mx, double *my, double *myaw, int *src) const
{
    if (!_cfg.rtkEnabled || !_cfg.rtkUseForPose)
        return false;
    if (!_map.isActive() || !_st.valid)
        return false;
    double ox = 0.0;
    double oy = 0.0;
    double oyaw = 0.0;
    _map.toMap(_st.x, _st.y, _st.yaw, &ox, &oy, &oyaw);
    int s = RtkFusion::SrcNone;
    if (!_fusion.fusedMapPose(ox, oy, oyaw, nowMs(), mx, my, myaw, &s))
        return false;
    if (src)
        *src = s;
    return true;
}

bool RtkPoseTracker::fusedLocalPose(double *lx, double *ly, double *lyaw, int *src) const
{
    double mx = 0.0;
    double my = 0.0;
    double myaw = 0.0;
    int s = RtkFusion::SrcNone;
    if (!fusedMapPose(&mx, &my, &myaw, &s))
        return false;
    // 导航与服务器下发的目标都在局部坐标（MapTransform 反变换后），
    // 所以这里把融合后的地图位姿再反算回局部，Navigator 一行都不用改
    _map.toLocal(mx, my, myaw, lx, ly, lyaw);
    if (src)
        *src = s;
    return true;
}

QString RtkPoseTracker::calibPath() const
{
    // 与 mapcalib_<id>.ini 同目录（应用工作目录），不污染主配置的注释
    return QCoreApplication::applicationDirPath()
           + QStringLiteral("/rtkcalib_%1.ini").arg(_cfg.terminalId);
}

void RtkPoseTracker::onMapRefReset()
{
    // 融合偏移是"参考点 + 当时差分"的差，参考点变了旧 offset 立即失效；
    // 尺度与转角属于物理量，保留（_calib 不受 reset 影响）
    _fusion.reset();
    _navPoseSrc = RtkFusion::SrcNone;
    // 新参考下地图 yaw 全部改变：重启判据的里程计基准必须作废，
    // 否则会拿旧参考下的 yaw 去判"狗有没有转过"，把整圈转角误判成模块重启
    _jump.reset();
}

void RtkPoseTracker::setNavActive(bool active)
{
    if (!_cfg.rtkEnabled)
        return;   // 差分关闭时不产生"请求同步/任务同步"日志，也不动融合状态
    _fusion.setNavActive(active);
}

// 断流观测：服务端标称 200ms，实测常见 500~560ms，偶尔 1s。
// 阈值取 max(配置值, 3 个实测平均周期)，避免把正常抖动当成断流刷告警。
// 只告警，不干预控制；恢复后不再重复告警（与原行为一致）。
void RtkPoseTracker::checkStale()
{
    if (!_cfg.rtkEnabled || !_rtkSeen || _rtkStaleWarned)
        return;
    const qint64 avgMs = (_rtkFrames > 1)
                             ? static_cast<qint64>((_rtkRecvMs - _rtkStartMs) / (_rtkFrames - 1))
                             : 0;
    const qint64 limitMs = qMax(static_cast<qint64>(_cfg.rtkStaleMs), avgMs * 3);
    const qint64 idleMs = nowMs() - _rtkRecvMs;
    if (idleMs > limitMs)
    {
        _rtkStaleWarned = true;
        qWarning() << "[Rtk] 差分位姿断流" << idleMs
                   << "ms（阈值" << limitMs << "ms，实测平均周期" << avgMs << "ms）";
    }
}
