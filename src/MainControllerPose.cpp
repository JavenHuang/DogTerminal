// ============================================================
// MainController 里程计位姿：yaw 稳定器、运动航向、融合位姿与地图参考持久化
// 本类是四个编译单元之一：Setup（装配）/ Nav（驱动槽）/ Pose（本文件）/
// 主 TU（对外接口与消息分发）。拆分只分实现，类声明仍在 MainController.h，
// 因此其它源文件不受影响。
// ============================================================

#include "MainController.h"

#include <QDateTime>
#include <QDataStream>
#include <QDebug>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QJsonParseError>
#include <QStringList>
#include <QSettings>
#include <QCoreApplication>
#include <cmath>
#include "Common/DogText.h"
#include "DogLink/ZsibotDefs.h"
#include "DogLink/UdpDogClient.h"
#ifdef DOGTERMINAL_HAS_SDK
#include "DogLink/SdkDogClient.h"
#endif
#ifdef DOGTERMINAL_HAS_MATRIX
#include "DogLink/MatrixDogClient.h"
#endif
#include "ServerLink/DetectionLink.h"
#include "ServerLink/CalibPanel.h"

namespace
{

// feedback 枚举转可读文本；固件实际取值未完全确认，故同时打印数值便于核对

// 角度归一化到 [0, 360)（地图角约定：0=正东 顺时针）

}  // namespace

// yaw 稳定化：
//  1) 连续传播：无论 raw yaw 是否更新，都用陀螺积分推进 YawStabilizer 内部的连续 yaw，保证 1Hz 状态包
//     与快速转向（60°/s 量级）下朝向仍连续
//  2) 一致性校验：raw 增量与该段陀螺积分一致时以 raw 重锚（抑制陀螺漂移）；
//     不一致（IMU 基准重置/单帧滑移）时不再静默忽略，而是按陀螺推算保持连续，
//     并标记"可疑"，等直线位移校核裁决
//  3) 零偏估计：静止时用 gyro[2] 低通估计，运动时从积分中扣除（限幅），仅用于传播
//  4) 静止锁：静止超过 StaticLockSec 后冻结朝向，避免 0.2~0.4°/s 的漂移累计


// 位移方向校验：腿足平台直线前进时，里程计位移方向 ≈ 机头朝向（打滑带来几度偏差）。
// 因此可用一段"直线前进"的位移方向作为与 IMU 完全独立的航向观测：
//  - 与当前航向偏差小 → 轻度修正，抑制慢漂
//  - 偏差大或航向可疑 → 直接以位移方向重建基准，避免"一直忽略跳变"越走越偏


void MainController::onDogStateData()
{
    const DogOdom &odom = _dog->odom();
    const DogImu &imu = _dog->imu();

    // ---- 链路体检：每 10s 一行，现场不用翻上万行日志 ----
    // 收包数≈10 说明狗端 1Hz 推送正常；空洞数多、最长间隔 3~5s，说明
    // "狗→终端"这一方向在成串丢包（与 MQ 无关，需要查 192.168.168.x 这段）。
    {
        const qint64 t = nowMs();
        if (_linkLastRxMs > 0)
        {
            const qint64 gap = t - _linkLastRxMs;
            if (gap > 2000)
            {
                ++_linkHoleCount;
                if (gap > _linkMaxGapMs)
                    _linkMaxGapMs = static_cast<int>(gap);
            }
        }
        _linkLastRxMs = t;
        ++_linkRxCount;
        if (_linkReportMs == 0)
            _linkReportMs = t;
        else if (t - _linkReportMs >= 10000)
        {
            qInfo().noquote()
                << QStringLiteral("[Link] 10s 体检: 收包 %1 帧 · 空洞(>2s) %2 次 · 最长间隔 %3ms"
                                  " | 连接=%4 控制权=%5")
                       .arg(_linkRxCount).arg(_linkHoleCount).arg(_linkMaxGapMs)
                       .arg(_dog->isConnected() ? QStringLiteral("是") : QStringLiteral("否"))
                       .arg(_controlAcquired ? QStringLiteral("有") : QStringLiteral("无"));
            _linkRxCount = 0;
            _linkHoleCount = 0;
            _linkMaxGapMs = 0;
            _linkReportMs = t;
        }
    }

    // ---- 0) 位置源：抗跳变死推算（[Pose] PosSource=dr，默认）----
    // 狗端 odom.position 会整体重置：run9 实测 8 次单帧跳变 0.63~3.12m，
    // 而包间隔只有 8~11ms（物理上最多走 9mm）——不是丢包，是坐标系重新起算。
    // 跳变直接进导航 → Pure Pursuit 的方位角 α 瞬变几十度 → 原地画圈；
    // 进地图 → "以为回到 0，其实没回"。DrPose 只积分合理步长，跳变帧整帧丢弃。
    double poseX = odom.position[0];
    double poseY = odom.position[1];
    if (odom.valid)
    {
        const double vb = std::sqrt(static_cast<double>(odom.vBody[0]) * odom.vBody[0]
                                    + static_cast<double>(odom.vBody[1]) * odom.vBody[1]);
        const qint64 t = nowMs();
        if (_cfg.posePosSourceDr)
        {
            _dr.feed(odom.position[0], odom.position[1], vb, t);
            poseX = _dr.x();
            poseY = _dr.y();
        }
        else
        {
            // 原始模式：只对齐 raw 基准，位姿仍用狗体原值
            _dr.alignRaw(odom.position[0], odom.position[1], t);
        }
    }

    // 里程计跳变补偿：死推算已把跳变挡在位姿之外，再平移地图参考反而把位姿推歪
    if (_cfg.posePosSourceDr)
        _lastOdomValid = false;      // 切回原始模式时重新对齐，避免跨模式误判
    else
        compensateOdomJump(odom);

    // yaw 稳定化：之后所有导航/地图/上报统一用 _yaw->yaw()，不再直接用 imu.rpy[2]
    // 位置用过滤后的 pose，位移方向校核才不会把一次 3m 跳变当成"一步真实位移"
    DogOdom pose = odom;
    pose.position[0] = static_cast<float>(poseX);
    pose.position[1] = static_cast<float>(poseY);
    _yaw->update(pose, imu);
    compensateYawReset(odom, imu);
    // 位移方向校核：用直线前进的里程计位移方向独立验证/重建航向基准
    _yaw->updateMotionHeading(pose, imu);

    // 参考建立：终端启动后首个有效位姿自动建立（当前点=O点 朝向=0°正东），
    // 与狗开机时刻无关——狗可以先开机、先被遥控移动
    if (_map.params().enabled && _map.params().autoReference
        && !_map.hasReference() && odom.valid && imu.valid)
    {
        resetMapReference(poseX, poseY, _yaw->yaw());
        qInfo().noquote() << "[Map] 参考已建立（终端启动时狗位姿=O点/0°正东）:" << _map.describe();
    }

    // ---- 差分静置锚定：只在停稳时把位置钉到差分上，段内由死推算推进 ----
    // 不产生位姿流：差分 1 次停稳只修正一次，修正量小于阈值直接不动（防抖）。
    if (_rtk && _map.isActive() && odom.valid && imu.valid)
    {
        double ax = 0.0;
        double ay = 0.0;
        double asp = 0.0;
        int    an = 0;
        if (_rtk->takeAnchor(&ax, &ay, &an, &asp))
        {
            double mx = 0.0;
            double my = 0.0;
            double myaw = 0.0;
            _map.toMap(poseX, poseY, _yaw->yaw(), &mx, &my, &myaw);
            applyRtkAnchor(ax, ay, mx, my, an, asp);
            poseX = _dr.x();
            poseY = _dr.y();
        }
    }

    // 直线标定行程监控：mapreset → mapline 之间每秒报"走了多少米"
    if (_traceActive && odom.valid)
        traceTick(odom);

    // 本地状态打印：同时输出原始局部坐标与地图坐标
    _printer->print();

    // 缓存最近一次可用状态：20Hz 控制节拍用它推进导航，
    // 这样控制节拍不再被状态包到达率（名义 1Hz / 空洞时 3~4s）拖住。
    if (imu.valid)
    {
        if (odom.valid)
        {
            const double nx = poseX;
            const double ny = poseY;
            // 位置差分推算速度（低通）：差分 5Hz 天生滞后，只有确实"没在动"时
            // 才能把差分当地图锚点用，所以这个速度值直接决定 offset 是否更新
            const double dtSec = (_st.ms > 0) ? (nowMs() - _st.ms) / 1000.0 : 0.0;
            if (_st.valid && dtSec > 1e-3)
            {
                const double d = std::sqrt((nx - _st.x) * (nx - _st.x)
                                           + (ny - _st.y) * (ny - _st.y));
                const double v = d / dtSec;
                if (v < 3.0)   // 超过 3m/s 是单帧跳变，不参与速度估计
                    _st.speed = (_st.speed <= 0.0) ? v : (_st.speed + 0.3 * (v - _st.speed));
            }
            _st.x   = nx;
            _st.y   = ny;            _st.vx  = odom.vBody[0];
        }
        _st.yaw     = _yaw->yaw();
        // 上报用的航向角速度：与 YawStabilizer 同源，gyro 缺失时退回里程计机体角速度
        _st.yawRate = static_cast<double>(imu.gyroField ? imu.gyro[2]
                                                        : (odom.valid ? odom.omegaBody[2] : 0.0f));
        _st.ms      = nowMs();
        _st.valid   = true;
        _st.staleStopped = false;   // 状态恢复，解除盲发暂停
    }

    // 到点复核：位姿缓存刚更新（含刚才那次锚点修正），在这里判断残差
    if (_avHold)
        maybeFinishArriveVerify(nowMs());

    if (!_navigator->isActive() || !_dog->isConnected() || !_controlAcquired)
    {
        if (_navigator->isActive())
        {
            const qint64 now = nowMs();
            if (now - _lastNavDiagMs >= 1000)
            {
                _lastNavDiagMs = now;
                qWarning() << "[Main] 导航闭环阻断: connected=" << _dog->isConnected()
                           << "controlAcquired=" << _controlAcquired;
            }
        }
        return;
    }

    // 旋转只依赖 IMU；位置任务同时需要原始里程计和 IMU
    if (!imu.valid || (!_navigator->isRotationActive() && !odom.valid))
    {
        const qint64 now = nowMs();
        if (now - _lastNavDiagMs >= 1000)
        {
            _lastNavDiagMs = now;
            qWarning() << "[Main] 导航闭环阻断: odomValid=" << odom.valid
                       << "imuValid=" << imu.valid;
        }
        return;
    }

    // 原始 odom.position 直接进入导航；朝向用稳定后的 yaw；gyro[2] 仅作转向阻尼
    // 这里是"状态更新即推进一拍"的快路径；持续节拍由 onControlTick() 负责，
    // 两处走同一入口 NavBridge::stepFromCache()，避免两套算法分叉。
    _nav->stepFromCache();
}

void MainController::resetMapReference(double localX, double localY, double localYaw)
{
    _map.resetReference(localX, localY, localYaw);
    // 尺度固定 1.0：差分每帧直接钉位置，不再用旧直线标定系数
    _map.setScale(1.0);

    // mapreset 标定：把当前差分 yaw 读数记为零位——之后差分报的角度都减这个零位，
    // 等于"mapreset 那一刻的差分朝向 = 地图 0°"。开机标定一次，运行期不再改。
    double rx = 0.0, ry = 0.0, ryaw = 0.0;
    qint64 rms = 0;
    if (_rtk && _rtk->lastFix(&rx, &ry, &rms, &ryaw))
    {
        _rtkYawZero = ryaw;
        // 地图原点 = 当前差分绝对坐标：之后 toMap/toLocal 换算都以差分为绝对基准
        MapTransform::Params p = _map.params();
        p.originX = rx;
        p.originY = ry;
        _map.setParams(p);
        qWarning().noquote() << QStringLiteral(
            "[Map] 差分标定完成：原点=(%1, %2) · 航向零位=%3°"
            "（地图坐标=差分绝对坐标，之后差分报多少地图就是多少）")
               .arg(rx, 0, 'f', 2).arg(ry, 0, 'f', 2)
               .arg(ryaw * 180.0 / M_PI, 0, 'f', 1);
    }
    else
    {
        _rtkYawZero = 0.0;
        qWarning().noquote() << QStringLiteral(
            "[Map] 建参考时差分帧未就绪，原点/航向零位暂记 0；下一帧差分来后自动补标定");
    }

    // 融合偏移与重启判据基准都绑在旧参考点上，交给跟踪器统一作废（尺度/转角继续沿用）
    _rtk->onMapRefReset();
    // 参考朝向变了 → 地图系被整体旋转，差分系到地图系的转角估计与相对基准同时作废
    _rkBaseValid = false;
    _rkRotSamples = 0;
    _rkRotRad = 0.0;
    _rkRotApplied = 0;
    _rkAnchorN = 0;
    // 差分跟随也要重新认基准：旧基准的平移是旧原点下的，继续用会把位姿拽偏
    _rkFollowValid = false;
    _rkFollowX = 0.0;
    _rkFollowY = 0.0;
    _rkFollowLogMs = 0;
    // 地图 0° 就是"此刻狗的朝向"，所以差分零位必须锚在同一姿态上：
    // 融合模块自己等静止窗口取 b，取到之前不建立偏移，避免随机零位污染航向基准。
    if (_cfg.rtkEnabled)
    {
        _fusion.requestYawZero(QStringLiteral("地图参考建立"));
        if (_st.valid && _st.speed > _cfg.rtkOffStaticSpeed)
        {
            qWarning().noquote() << QStringLiteral(
                "[Rtk] 建参考时狗未停稳（速度 %1m/s）：零位将在停稳后补取，方向可能有偏差，"
                "建议先停稳再下发 mapreset").arg(_st.speed, 0, 'f', 2);
        }
    }
}



// 差分静置锚点 → 位姿修正 / 方向校准
//
// 现场口径（用户确认 + 日志复核）：差分输出已经是地图坐标——正东 0°、顺时针
// 0~360、x 轴东正西负、y 轴南正北负，与 MapTransform 的地图系同向同号。
// 所以差分与本地图之间只可能剩两个未知量：
//   ① 平移：差分原点 ≠ mapreset 原点（复位点就是地图 O），量级 1~3m 且每次
//     mapreset 都变（日志：同一段会话 offset_x 中位 1.68m、前 1/3 与后 1/3 差 2.8m）；
//   ② 旋转：mapreset 把"此刻狗体朝向"定义成 0°，若狗当时不是朝正东，
//     整张终端地图相对服务器地图就转了这一个角（日志：差分与地图 yaw 差
//     中位 -99°、p10/p90 -142/+161°，说明这个角常常是几十度而不是 0）。
// 处理方式：平移用"一次基准 + 位移差"吸收，基准不再逐站前移——前移会让每站
// 0.61×0.42m 格点的量化误差累积成随机游走；旋转则由两段 ≥1.2m 的
// "差分位移方向 vs 地图位移方向"估出来，够一致时直接 shiftRefYaw 把整个
// 地图系转正（一次性闭环），之后位置与航向同时对上差分。
//
// 注意：差分 yaw 本身不能用（日志实测 3.8Hz 输出、位置格点未变时 Δyaw 中位
// 0.2°、位置一动与位移方向夹角中位 68.6°、2 分钟窗口跨度 359.9°），
// 所以航向一律走 IMU + refYaw，差分只提供"位移方向"这一个方向信息。
void MainController::applyRtkAnchor(double ax, double ay, double mapX, double mapY,
                                    int frames, double spread)
{
    ++_rkAnchorN;
    // _anchorSeq 只记"真正用差分结算过一次"的锚点（修正或复核通过），
    // 因为到点复核拿它判"锚点已生效"。基准建立那次只是记一个基准、不改位姿，
    // 若也自增，复核会在没有任何修正的情况下直接宣布"锚点已生效"并上报残差。

    if (!_rkBaseValid)
    {
        _rkBaseValid = true;
        _rkBaseRtkX = ax;
        _rkBaseRtkY = ay;
        _rkBaseMapX = mapX;
        _rkBaseMapY = mapY;
        qWarning().noquote()
            << QStringLiteral("[Anchor] 基准建立：差分(%1, %2) ↔ 地图(%3, %4)"
                              "（窗口 %5 帧 · 散布 %6m）——之后每次停稳用差分位移差校正地图位姿")
                   .arg(ax, 0, 'f', 2).arg(ay, 0, 'f', 2)
                   .arg(mapX, 0, 'f', 2).arg(mapY, 0, 'f', 2)
                   .arg(frames).arg(spread, 0, 'f', 2);
        return;
    }

    const double drx = ax - _rkBaseRtkX;
    const double dry = ay - _rkBaseRtkY;
    const double dmx = mapX - _rkBaseMapX;
    const double dmy = mapY - _rkBaseMapY;
    const double lenR = std::sqrt(drx * drx + dry * dry);
    const double lenM = std::sqrt(dmx * dmx + dmy * dmy);

    // ---- 尺度（k）校核：差分位移是"真米数"，地图位移是 odom×当前 scale ----
    // 两者之比直接给出真实 k：k_true = k_cur × lenR / lenM。
    // 只管打印、不自动改 map：尺度会改变整张地图的坐标，现场跑任务时静默改
    // 会让正在执行的目标点在脚下移动；有偏差时用 mapline 重标一次更可控。
    if (!_cfg.rtkYawFollow && lenR >= 3.0 && lenM >= 2.0)
    {
        const double kTrue = _map.scale() * (lenR / lenM);
        const double devPct = (kTrue / _map.scale() - 1.0) * 100.0;
        if (std::fabs(devPct) >= 3.0)
            qWarning().noquote()
                << QStringLiteral("[Anchor] 尺度校核：本段差分 %1m / 地图 %2m → 真实 k≈%3"
                                  "（当前 k=%4，偏差 %5%）——建议按此重发 mapline")
                       .arg(lenR, 0, 'f', 2).arg(lenM, 0, 'f', 2)
                       .arg(kTrue, 0, 'f', 3).arg(_map.scale(), 0, 'f', 3)
                       .arg(devPct, 0, 'f', 1);
        else
            qInfo().noquote()
                << QStringLiteral("[Anchor] 尺度校核通过：本段差分 %1m / 地图 %2m → k≈%3"
                                  "（当前 k=%4，偏差 %5%）")
                       .arg(lenR, 0, 'f', 2).arg(lenM, 0, 'f', 2)
                       .arg(kTrue, 0, 'f', 3).arg(_map.scale(), 0, 'f', 3)
                       .arg(devPct, 0, 'f', 1);
    }

    // ---- 方向校准：估计 δ = 地图位移方向 − 差分位移方向，够一致就闭环进 refYaw ----
    // 现场口径（2026-11-16）：差分标定后角度即绝对真值，运行期不再闭环改 refYaw——
    // 那等于运行期重标定角度。rtkYawFollow 开启时此段整体关闭，绝对朝向由每帧钉入。
    const double boot = _cfg.rtkAnchorBootMoveM;
    if (!_cfg.rtkYawFollow && _rkRotApplied < 2 && lenR >= boot && lenM >= boot)
    {
        const double delta = DogProtocol::radStd(std::atan2(dmy, dmx) - std::atan2(dry, drx));
        const double kConsist = 12.0 * M_PI / 180.0;
        if (std::fabs(delta) <= 60.0 * M_PI / 180.0)
        {
            bool accept = false;
            if (_rkRotSamples == 0)
            {
                _rkRotRad = delta;
                accept = true;
            }
            else if (std::fabs(DogProtocol::radStd(delta - _rkRotRad)) <= kConsist)
            {
                // 圆周均值（两段幅值差已经被一致性门槛限制，这里只做平均）
                _rkRotRad = std::atan2(std::sin(_rkRotRad) + std::sin(delta),
                                       std::cos(_rkRotRad) + std::cos(delta));
                accept = true;
            }
            else
            {
                // 与已有估计差太多：以新样本重新起算，不叠加
                _rkRotRad = delta;
                _rkRotSamples = 0;
                accept = true;
                qWarning().noquote()
                    << QStringLiteral("[Anchor] 方向样本突变 %1°，重新起算（本段差分 %2m / 地图 %3m）")
                           .arg(delta * 180.0 / M_PI, 0, 'f', 1)
                           .arg(lenR, 0, 'f', 2).arg(lenM, 0, 'f', 2);
            }
            if (accept)
            {
                ++_rkRotSamples;
                qInfo().noquote()
                    << QStringLiteral("[Anchor] 方向观测 %1°（已采 %2 段，当前估计 %3°）")
                           .arg(delta * 180.0 / M_PI, 0, 'f', 1)
                           .arg(_rkRotSamples)
                           .arg(_rkRotRad * 180.0 / M_PI, 0, 'f', 1);
            }
            if (_rkRotSamples >= 2 && _rkRotApplied < 2)
            {
                const double applied = _rkRotRad;
                // 闭环＝绕地图原点整体旋转，当前点会跟着跳 d·sinδ。跳到 1m 以上就别硬转：
                // 这说明 mapreset 时狗横得太歪，重来一次比"把地图转飞"便宜。
                const double ox = _map.params().originX;
                const double oy = _map.params().originY;
                const double dFromO = std::sqrt((mapX - ox) * (mapX - ox) + (mapY - oy) * (mapY - oy));
                const double jumpM = dFromO * std::fabs(std::sin(applied));
                if (jumpM > 1.0)
                {
                    _rkRotRad = 0.0;
                    _rkRotSamples = 0;
                    qWarning().noquote()
                        << QStringLiteral("[Anchor] 方向闭环被拒绝：需转 %1°、当前点会跳 %2m（距原点 %3m）。"
                                          "请就地停稳 → mapreset 重新定原点 → 再走两段 ≥%4m 直线")
                               .arg(applied * 180.0 / M_PI, 0, 'f', 1)
                               .arg(jumpM, 0, 'f', 2)
                               .arg(dFromO, 0, 'f', 2)
                               .arg(boot, 0, 'f', 1);
                    return;
                }
                _map.shiftRefYaw(-applied);
                ++_rkRotApplied;
                _rkRotRad = 0.0;
                _rkRotSamples = 0;
                // 地图系刚被整体旋转：位移基准随之作废，下一次停稳重建
                _rkBaseValid = false;
                qWarning().noquote()
                    << QStringLiteral("[Anchor] 方向校准闭环：地图系旋转 %1°，refYaw → %2°"
                                      "（差分位移 %3m 与地图位移 %4m 同向；当前点因此移动 ≈%5m，"
                                      "地图上所有已画点同步旋转过去，不是狗真跳了）")
                           .arg((-applied) * 180.0 / M_PI, 0, 'f', 1)
                           .arg(_map.refYaw() * 180.0 / M_PI, 0, 'f', 1)
                           .arg(lenR, 0, 'f', 2).arg(lenM, 0, 'f', 2)
                           .arg(jumpM, 0, 'f', 2);
                return;
            }
        }
    }

    // 目标地图位姿 = 固定基准 + R(δ) × 差分位移（δ 未闭环时才有非零值）
    const double c = std::cos(_rkRotRad);
    const double s = std::sin(_rkRotRad);
    const double tx = _rkBaseMapX + (c * drx - s * dry);
    const double ty = _rkBaseMapY + (s * drx + c * dry);
    const double ex = tx - mapX;
    const double ey = ty - mapY;
    const double err = std::sqrt(ex * ex + ey * ey);

    // 超过 3m 的"修正"不可能是正常漂移：差分跳格 / 基准记错，本次不采纳并重置基准
    if (err > 3.0)
    {
        qWarning().noquote()
            << QStringLiteral("[Anchor] 修正量 %1m 异常，本次不采纳（疑似差分跳格或基准错位）；"
                              "基准重置到当前：差分(%2, %3) ↔ 地图(%4, %5)")
                   .arg(err, 0, 'f', 2)
                   .arg(ax, 0, 'f', 2).arg(ay, 0, 'f', 2)
                   .arg(mapX, 0, 'f', 2).arg(mapY, 0, 'f', 2);
        _rkBaseRtkX = ax;
        _rkBaseRtkY = ay;
        _rkBaseMapX = mapX;
        _rkBaseMapY = mapY;
        return;
    }

    if (err >= _cfg.rtkAnchorMinApplyM)
    {
        double lx = 0.0;
        double ly = 0.0;
        _map.toLocal(tx, ty, 0.0, &lx, &ly, nullptr);
        _dr.reset(lx, ly);
        ++_anchorSeq;   // 真发生了一次差分修正 → 到点复核可以据此结算
        qWarning().noquote()
            << QStringLiteral("[Anchor] 锚定修正 %1m：地图(%2, %3) → (%4, %5)"
                              "（差分(%6, %7) · 本段%8m · 窗口%9帧 · 散布%10m · 第%11次锚点）")
                   .arg(err, 0, 'f', 2)
                   .arg(mapX, 0, 'f', 2).arg(mapY, 0, 'f', 2)
                   .arg(tx, 0, 'f', 2).arg(ty, 0, 'f', 2)
                   .arg(ax, 0, 'f', 2).arg(ay, 0, 'f', 2)
                   .arg(lenM, 0, 'f', 2)
                   .arg(frames).arg(spread, 0, 'f', 2)
                   .arg(_rkAnchorN);
    }
    else
    {
        // 矫正量小于阈值时不即位姿：差分输出量化在 0.61×0.42m 格点上，
        // 同一格内重复采样是恒定值（日志实测连续 67 帧不变），跟着改只会来回蹭。
        // 但这一步"差分已核对过一次、结论是不必修正"，对到点复核同样算一次有效
        // 结算（否则残差已经很小也要白等到超时）。
        ++_anchorSeq;
        qInfo().noquote()
            << QStringLiteral("[Anchor] 复核通过：残差 %1m < 阈值 %2m，不修正"
                              "（差分(%3, %4) · 本段%5m · 窗口%6帧 · 第%7次锚点）")
                   .arg(err, 0, 'f', 2)
                   .arg(_cfg.rtkAnchorMinApplyM, 0, 'f', 2)
                   .arg(ax, 0, 'f', 2).arg(ay, 0, 'f', 2)
                   .arg(lenM, 0, 'f', 2)
                   .arg(frames).arg(_rkAnchorN);
    }

    // 关键：成功修正后"基准不动"。差分基准一旦前移，每站的量化误差就会
    // 一段一段累加进地图，停 10 次就有 10 次误差的随机游走；固定基准只保留
    // 一次常值偏置（可重复、可接受），漂移抑制效果完全一样。
}

// ============================================================
// 差分位置跟随（[Rtk] PosFollow=true）
//
// 与静置锚点的区别：锚点只在"停稳 1.2s + 攒满 12 帧"后给一次点（一次行走
// 往往只有一两站）；跟随则在**每一个差分数值更新**时都对一次位置，位姿全程
// 被差分约束，里程计只在格点内推进。两者共用同一套基准/转角换算，不打架。
//
// 三条现场约束写进实现：
//   1) 差分值没变（同格冻结）→ 不跟随。若每帧都钉，格点内的里程计推进会被
//      一遍遍抹掉，位姿停着不动直到跨格（实测冻结最长 17.6s），导航必崩。
//   2) 修正量 >3m 不采纳（差分跳格/基准错位），基准重置到当前，只告警。
//   3) 跟随推进 _anchorSeq：到点复核据此判"差分结算过"，不必再等停稳锚点。
// ============================================================
// ============================================================
// 把位姿钉到差分推算位置：基准（平移）+ 转角（_rkRotRad）一次换算
// 返回 true 表示位姿真的被修正了（_st/_dr 已同步）
// ============================================================
bool MainController::applyRtkFollowTo(double ax, double ay)
{
    double mx = 0.0;
    double my = 0.0;
    _map.toMap(_st.x, _st.y, _st.yaw, &mx, &my, nullptr);

    if (!_rkBaseValid)
    {
        _rkBaseValid = true;
        _rkBaseRtkX = ax;
        _rkBaseRtkY = ay;
        // 地图坐标 = 差分绝对坐标（不再是相对 (0,0)）：之后 tx = ax, ty = ay
        _rkBaseMapX = ax;
        _rkBaseMapY = ay;
        // mapreset 时差分帧未就绪的情况：补设地图原点
        MapTransform::Params p = _map.params();
        if (std::fabs(p.originX) < 1e-6 && std::fabs(p.originY) < 1e-6)
        {
            p.originX = ax;
            p.originY = ay;
            _map.setParams(p);
        }
        _rkFollowValid = true;
        _rkFollowX = ax;
        _rkFollowY = ay;
        qWarning().noquote()
            << QStringLiteral("[Anchor] 差分跟随基准：差分(%1, %2) ↔ 地图(%3, %4)"
                              "——地图坐标=差分绝对坐标，之后差分每跳一格就把位姿钉回它")
                   .arg(ax, 0, 'f', 2).arg(ay, 0, 'f', 2)
                   .arg(ax, 0, 'f', 2).arg(ay, 0, 'f', 2);
        return false;
    }

    const double drx = ax - _rkBaseRtkX;
    const double dry = ay - _rkBaseRtkY;
    const double c = std::cos(_rkRotRad);
    const double s = std::sin(_rkRotRad);
    const double tx = _rkBaseMapX + (c * drx - s * dry);
    const double ty = _rkBaseMapY + (s * drx + c * dry);
    const double ex = tx - mx;
    const double ey = ty - my;
    const double err = std::sqrt(ex * ex + ey * ey);

    if (err > 3.0)
    {
        qWarning().noquote()
            << QStringLiteral("[Anchor] 差分跟随修正量 %1m 异常，本次不采纳（疑似跳格/基准错位）；"
                              "基准重置到当前：差分(%2, %3) ↔ 地图(%4, %5)")
                   .arg(err, 0, 'f', 2)
                   .arg(ax, 0, 'f', 2).arg(ay, 0, 'f', 2)
                   .arg(mx, 0, 'f', 2).arg(my, 0, 'f', 2);
        _rkBaseRtkX = ax;
        _rkBaseRtkY = ay;
        _rkBaseMapX = mx;
        _rkBaseMapY = my;
        _rkFollowValid = true;
        _rkFollowX = ax;
        _rkFollowY = ay;
        return false;
    }

    // 格点量化本身就有 0.1~0.3m 的台阶：小于 0.03m 视为"同格重复"，不折腾位姿
    if (err < 0.03)
    {
        _rkFollowValid = true;
        _rkFollowX = ax;
        _rkFollowY = ay;
        return false;
    }

    double lx = 0.0;
    double ly = 0.0;
    _map.toLocal(tx, ty, 0.0, &lx, &ly, nullptr);
    _dr.reset(lx, ly);          // 下一个状态包起，_st 就从这里继续积分
    _st.x = lx;                 // 控制节拍读的是 _st，当场同步，不等下一包
    _st.y = ly;
    ++_anchorSeq;               // 真发生了一次差分修正 → 到点复核可据此结算
    _rkFollowValid = true;
    _rkFollowX = ax;
    _rkFollowY = ay;

    const qint64 tNow = nowMs();
    if (_rkFollowLogMs == 0 || tNow - _rkFollowLogMs >= 1000)
    {
        _rkFollowLogMs = tNow;
        qInfo().noquote()
            << QStringLiteral("[Anchor] 差分跟随 %1m：地图(%2, %3) → (%4, %5)"
                              "（差分(%6, %7) · 第%8次结算）")
                   .arg(err, 0, 'f', 2)
                   .arg(mx, 0, 'f', 2).arg(my, 0, 'f', 2)
                   .arg(tx, 0, 'f', 2).arg(ty, 0, 'f', 2)
                   .arg(ax, 0, 'f', 2).arg(ay, 0, 'f', 2)
                   .arg(_anchorSeq);
    }
    return true;
}

void MainController::syncRtkFollow()
{
    if (!_cfg.rtkEnabled || !_rtk || !_st.valid)
        return;
    if (!_map.isActive())
        return;

    double ax = 0.0;
    double ay = 0.0;
    double ayaw = 0.0;
    qint64 fixMs = 0;
    if (!_rtk->lastFix(&ax, &ay, &fixMs, &ayaw))
        return;

    // 差分航向 = 绝对真值：每个差分帧都把局部 yaw 钉到差分地图角，
    // 与位置是否跨格、是否开启 PosFollow 无关（同格冻结时角度仍照常钉）。
    // 帧间由陀螺积分桥接。mapreset 时当前差分 yaw 已记为零位，这里先减零位再换算。
    if (_cfg.rtkYawFollow && _map.hasReference())
    {
        const double mapYaw = DogProtocol::radStd(ayaw - _rtkYawZero);
        const double localYaw = _map.mapYawToLocalYaw(mapYaw);
        _yaw->forceReference(localYaw);
        _st.yaw = localYaw;
    }

    // 位置跟随：仅在 PosFollow 且位姿走 dr 时生效（角度路径已在上面独立完成）
    if (!_cfg.rtkPosFollow || !_cfg.posePosSourceDr)
        return;

    const qint64 nowFix = fixMs > 0 ? fixMs : nowMs();

    // 同格冻结：差分值没动就不碰位姿，交给 DrPose 推进
    if (_rkGuardSeen
        && std::fabs(ax - _rkGuardLastX) < 1e-6 && std::fabs(ay - _rkGuardLastY) < 1e-6)
        return;

    const double stepM = _rkGuardSeen
        ? std::sqrt((ax - _rkGuardLastX) * (ax - _rkGuardLastX)
                    + (ay - _rkGuardLastY) * (ay - _rkGuardLastY))
        : 0.0;
    const double dtChg = (_rkGuardSeen && _rkGuardLastChangeMs > 0 && nowFix > _rkGuardLastChangeMs)
        ? static_cast<double>(nowFix - _rkGuardLastChangeMs) / 1000.0
        : 0.26;   // 未记录时刻时按实测差分周期 262ms 估
    // 物理允许步长：狗最快 2.0m/s，差分周期 262ms → 单帧真实位移 ≤0.53m；
    // 一格 0.61×0.42m。所以"一步超过 0.8m（或 2m/s 在断流期间该走的距离）"只能是
    // 跳格/模块重启：此时把位姿钉过去等于把狗瞬移，改为停车等它稳定。
    const double allowM = qMax(_cfg.rtkGuardJumpM, 2.0 * dtChg);

    _rkGuardSeen = true;
    _rkGuardLastX = ax;
    _rkGuardLastY = ay;
    _rkGuardLastChangeMs = nowFix;

    // 现场口径：差分不会跳变，只会上报慢。单帧步长大于"物理允许"是延迟上报
    // 积累的真实位移，不是跳格——直接跟随，不停车。只保留断流停车（updateRtkGuard）。
    applyRtkFollowTo(ax, ay);
}

// ============================================================
// 差分异常守门（20Hz 控制节拍调用）
//
// 两条判据，命中就停车（NavBridge 收到 _rtkHold 后只发零速、不推进导航）：
//   1) 断流：超过 GuardStaleMs 没有新差分帧 → 位姿只能靠里程计外推，
//      现场实测断流 1.5~3.0s 出现过 8 次，这期间走多远就错多远；
//   2) 坐标跳动：差分单帧跳超过"物理允许步长"（见 syncRtkFollow）。
// 恢复：差分帧新鲜且数值连续 N 拍不再变（落在同一格）→ 若位姿还落后于差分，
// 先在原地一次性钉到差分（此时狗是停的，不会把修正量走成实位移），再继续任务。
// ============================================================
void MainController::updateRtkGuard()
{
    if (!_cfg.rtkGuardStop || !_cfg.rtkEnabled || !_cfg.posePosSourceDr || !_rtk)
        return;
    if (!_cfg.rtkPosFollow)
        return;

    double ax = 0.0;
    double ay = 0.0;
    qint64 fixMs = 0;
    const qint64 now = nowMs();
    if (!_rtk->lastFix(&ax, &ay, &fixMs))
        return;

    // 断流：差分长时间没上报
    const bool stale = (now - fixMs) > static_cast<qint64>(_cfg.rtkGuardStaleMs);
    if (stale)
    {
        _rkGuardGoodN = 0;
        if (!_rtkHold && _navigator && _navigator->isActive())
        {
            _rtkHold = true;
            _rtkHoldWhy = QStringLiteral("差分断流 %1ms").arg(now - fixMs);
            qWarning().noquote()
                << QStringLiteral("[Rtk] 差分断流 %1ms（阈值 %2ms）→ 先停车等数据，"
                                  "恢复后自动继续当前任务")
                       .arg(now - fixMs).arg(_cfg.rtkGuardStaleMs);
            _rkHoldLogMs = now;
        }
        return;
    }

    if (!_rtkHold)
        return;

    // 恢复判据：差分值连续 N 拍不变（稳定落在同一格）且帧新鲜
    const bool valueStable = std::fabs(ax - _rkGuardLastX) < 1e-6
                             && std::fabs(ay - _rkGuardLastY) < 1e-6;
    if (!valueStable)
    {
        _rkGuardGoodN = 0;
        return;
    }
    if (++_rkGuardGoodN < _cfg.rtkGuardRecoverFrames)
        return;

    // 位姿还落后于差分（跳格那次没钉）：趁机在原地一次性落地
    const bool lagging = !_rkFollowValid
                         || std::fabs(ax - _rkFollowX) > 1e-6
                         || std::fabs(ay - _rkFollowY) > 1e-6;
    if (lagging)
        applyRtkFollowTo(ax, ay);

    _rtkHold = false;
    _rkGuardGoodN = 0;
    qWarning().noquote()
        << QStringLiteral("[Rtk] 差分已恢复（连续 %1 帧稳定，差分(%2, %3)）→ 继续执行当前任务")
               .arg(_cfg.rtkGuardRecoverFrames)
               .arg(ax, 0, 'f', 2).arg(ay, 0, 'f', 2);
    _rkHoldLogMs = now;
}

void MainController::beginLineTrace()
{
    const DogOdom &odom = _dog->odom();
    _traceActive   = true;
    _traceStartX   = odom.position[0];
    _traceStartY   = odom.position[1];
    _traceLastX    = _traceStartX;
    _traceLastY    = _traceStartY;
    _tracePath     = 0.0;
    _traceAccX     = 0.0;
    _traceAccY     = 0.0;
    _traceLastMs   = 0;
    _traceLastChangeMs = 0;
    _traceJumpN    = 0;
    _traceJumpM    = 0.0;
    _tracePkts     = 0;
    _traceStartMs  = nowMs();
    _tracePrintMs  = 0;

    // 只在这一小段时间用 warn 打，保证 LogConsole=warn 的现场也能直接看见
    qWarning().noquote()
        << QStringLiteral("[Line] 行程监控开始：里程计起点已记忆。现在向东直线走，"
                          "每秒报一次里程计读数；停稳后发 mapline 1 <你实测的米数> 0");
}

void MainController::traceTick(const DogOdom &odom)
{
    // 单帧步长按"自上次位置变化以来的间隔 × 实测速度"判定，而不是包间隔：
    // 狗端 odom.position 是**成块刷新**的（2026-11-16 实测：同一值重复若干包，
    // 真正变化那一帧一步 0.12~0.45m，约 1s 一次），包间隔却只有 8~11ms。
    // 按包间隔算 allow≈0.11m，于是把真实位移判成跳变整帧丢掉：
    // 行程显示 0.43m，而差分位移 1.22m、原始 odom 1.14m，被丢的正是真走的 0.71m。
    // 判据改成"物理上不可能才丢"：静止挪一步、或走动中一跳超出 maxGap×速度。
    const double sdx = odom.position[0] - _traceLastX;
    const double sdy = odom.position[1] - _traceLastY;
    const double step = std::sqrt(sdx * sdx + sdy * sdy);

    const qint64 tNow = nowMs();
    const double dtPkt = (_traceLastMs > 0)
                       ? static_cast<double>(tNow - _traceLastMs) / 1000.0 : 0.0;
    _traceLastMs = tNow;

    const bool changed = (sdx != 0.0 || sdy != 0.0);
    const qint64 sinceChangeMs = (changed && _traceLastChangeMs > 0)
                               ? (tNow - _traceLastChangeMs) : 0;
    if (changed)
        _traceLastChangeMs = tNow;
    double dtEf = static_cast<double>(sinceChangeMs) / 1000.0;
    if (dtEf < dtPkt)
        dtEf = dtPkt;
    if (dtEf > 2.0)
        dtEf = 2.0;

    const double vBody = std::sqrt(static_cast<double>(odom.vBody[0]) * odom.vBody[0]
                                   + static_cast<double>(odom.vBody[1]) * odom.vBody[1]);
    double allow = 3.0 * vBody * (dtEf > 0.0 ? dtEf : 0.05) + 0.10;
    if (allow < 0.10)
        allow = 0.10;

    if (step > allow)
    {
        // 物理上不可能的单帧步长：整帧丢弃，只计数（真位移已由 dtEf 放过）
        ++_traceJumpN;
        _traceJumpM += step;
    }
    else
    {
        _tracePath += step;
        _traceAccX += sdx;
        _traceAccY += sdy;
    }
    _traceLastX = odom.position[0];
    _traceLastY = odom.position[1];
    ++_tracePkts;

    const qint64 t = tNow;
    if (_tracePrintMs != 0 && (t - _tracePrintMs) < 1000)
        return;
    _tracePrintMs = t;

    const double disp = std::sqrt(_traceAccX * _traceAccX + _traceAccY * _traceAccY);
    const double dx = _traceAccX;
    const double dy = _traceAccY;

    // 方位角按地图约定换算：0=东、顺时针为正
    double headingDeg = 0.0;
    if (disp > 0.05)
    {
        const double a = _map.refYaw() - std::atan2(dy, dx);
        headingDeg = std::atan2(std::sin(a), std::cos(a)) * (180.0 / 3.14159265358979323846);
    }

    // 直线度 = 位移/路径：1.00 表示完全直线，越小越弯。
    // （反比写法会出现 101%/102%，看着像 bug；路径因包空洞少算时也会虚高，故上钳到 100%）
    double straight = 1.0;
    if (_tracePath > 1e-3 && disp > 1e-3)
        straight = disp / _tracePath;
    if (straight > 1.0)
        straight = 1.0;
    const double mapDisp  = disp * _map.scale();

    double mapX = 0.0, mapY = 0.0, mapYaw = 0.0;
    // 与位姿流同源：优先用过滤后的 _st（DrPose 已剔除跳变、与导航/上报一致）。
    // 原来这里拿原始 odom 换地图坐标，k 的逐秒行与"收尾"那行口径不同，
    // 现场按每秒行判 k 会得出另一个数，白折腾。
    if (_st.valid)
        _map.toMap(_st.x, _st.y, _st.yaw, &mapX, &mapY, &mapYaw);
    else
        _map.toMap(odom.position[0], odom.position[1], _yaw->yaw(), &mapX, &mapY, &mapYaw);

    QString line = QStringLiteral("[Line] 里程计行程 %1m · 路径 %2m · 直线度 %3% · 方位 %4° "
                                  "· 地图位移 %5m（k=%6）· 地图(%7, %8) · 用时 %9s")
                       .arg(disp, 0, 'f', 2)
                       .arg(_tracePath, 0, 'f', 2)
                       .arg(straight * 100.0, 0, 'f', 0)
                       .arg(headingDeg, 0, 'f', 1)
                       .arg(mapDisp, 0, 'f', 2)
                       .arg(_map.scale(), 0, 'f', 3)
                       .arg(mapX, 0, 'f', 2)
                       .arg(mapY, 0, 'f', 2)
                       .arg((t - _traceStartMs) / 1000.0, 0, 'f', 1);
    if (straight < 0.90)
        line += QStringLiteral("  ← 走得不直（位移/路径 %1%），按这个落点标定会把朝向算歪，建议重走")
                    .arg(straight * 100.0, 0, 'f', 0);
    if (_traceJumpN > 0)
        line += QStringLiteral(" · 期间剔除里程计跳变 %1 帧（共 %2m）")
                    .arg(_traceJumpN)
                    .arg(_traceJumpM, 0, 'f', 2);

    qWarning().noquote() << line;
}

bool MainController::lineTraceDisplacement(double *vLocalX, double *vLocalY) const
{
    if (!_traceActive)
        return false;
    const double disp = std::sqrt(_traceAccX * _traceAccX + _traceAccY * _traceAccY);
    if (disp < 0.20)
        return false;                      // 走得太少，不能用来定尺度
    if (vLocalX) *vLocalX = _traceAccX;
    if (vLocalY) *vLocalY = _traceAccY;
    return true;
}

void MainController::endLineTrace(double targetMapX, double targetMapY)
{
    if (!_traceActive)
        return;
    _traceActive = false;

    // 用"跳变免疫的累计位移"，不再用 (终点-起点)：狗端 odom 中途跳 0.6~3m 时，
    // 后者会把 k 顶到 1.4x 并让标定直接失败。
    const double dx = _traceAccX;
    const double dy = _traceAccY;
    const double odomDist = std::sqrt(dx * dx + dy * dy);

    // 你报的终点相对地图原点的距离，就是"我以为走了多少米"
    const double tx = targetMapX - _map.params().originX;
    const double ty = targetMapY - _map.params().originY;
    const double toldDist = std::sqrt(tx * tx + ty * ty);

    // 直线度同样是 位移/路径（1.00=直，越小越弯），与逐秒行保持同一口径
    double straight = 1.0;
    if (_tracePath > 1e-3 && odomDist > 1e-3)
        straight = odomDist / _tracePath;
    if (straight > 1.0)
        straight = 1.0;

    QString msg = QStringLiteral("[Line] 收尾: 里程计读数 %1m · 累计路径 %2m · 直线度 %3% "
                                 "· 你报的终点距 %4m")
                      .arg(odomDist, 0, 'f', 2)
                      .arg(_tracePath, 0, 'f', 2)
                      .arg(straight * 100.0, 0, 'f', 0)
                      .arg(toldDist, 0, 'f', 2);
    if (odomDist > 0.2 && toldDist > 0.2)
    {
        const double k = toldDist / odomDist;
        msg += QStringLiteral(" · 尺度 k=%1（里程计%2 %3%）")
                   .arg(k, 0, 'f', 3)
                   .arg(k < 1.0 ? QStringLiteral("偏大") : QStringLiteral("偏小"))
                   .arg(std::fabs(1.0 - k) * 100.0, 0, 'f', 1);
    }
    if (_traceJumpN > 0)
        msg += QStringLiteral(" · 已剔除里程计跳变 %1 帧（共 %2m，未计入读数）")
                   .arg(_traceJumpN).arg(_traceJumpM, 0, 'f', 2);
    qWarning().noquote() << msg;
}

void MainController::applyMapCalibToFusion()
{
    // 差分已关闭（Rtk/Enabled=false）：整套融合/标定都不参与位姿，地图系的
    // 原点与朝向由 mapreset/mapline 直接负责，这里什么都不做，避免日志与
    // rtkcalib 文件里留下无效的"标定取自地图"。
    if (!_cfg.rtkEnabled)
        return;

    // 地图系是"参考姿态定 0°、以 refYaw 为起始角"建起来的，所以把里程计局部系
    // 转到地图系的转角恒等于 refYaw；尺度就是 mapcalib 估出的 Scale。
    // 这两个量一旦确定，k/α 就不必再从停点里解——现场的 4 点共线、差分跳格
    // 都不会再让标定卡住。
    if (!_map.isActive())
    {
        qWarning().noquote() << "[Rtk] 地图坐标未启用，跳过标定回填（融合继续用旧值）";
        return;
    }
    _fusion.setCalibFromMap(_map.scale(), _map.refYaw());
}


void MainController::compensateOdomJump(const DogOdom &odom)
{
    // 原始模式：位置跳变原样进地图，不做参考平移（现场要看到狗体的真实数据）
    if (_cfg.poseRawOdom)
    {
        _lastOdomValid = false;
        return;
    }
    if (!odom.valid)
    {
        _lastOdomValid = false;
        return;
    }

    const qint64 now = nowMs();
    if (!_lastOdomValid)
    {
        _lastOdomValid = true;
        _lastOdomX = odom.position[0];
        _lastOdomY = odom.position[1];
        _lastOdomMs = now;
        return;
    }

    const double dx = odom.position[0] - _lastOdomX;
    const double dy = odom.position[1] - _lastOdomY;
    const double dt = static_cast<double>(now - _lastOdomMs) / 1000.0;

    _lastOdomX = odom.position[0];
    _lastOdomY = odom.position[1];
    _lastOdomMs = now;

    // 采样间隔异常（断线重连/暂停）时不判定，避免误补偿
    if (dt <= 0.0 || dt > 2.0)
        return;

    const double dist = std::sqrt(dx * dx + dy * dy);
    const double vBody = std::sqrt(static_cast<double>(odom.vBody[0]) * odom.vBody[0]
                                   + static_cast<double>(odom.vBody[1]) * odom.vBody[1]);
    const double plausible = 1.8 * vBody * dt + 0.5;
    if (dist <= plausible)
        return;

    if (!_map.isActive() || !_map.params().jumpCompensate)
        return;

    _map.shiftReference(dx, dy);
    qWarning().noquote()
        << QStringLiteral("[Map] 里程计跳变 %1m (dt=%2ms, vBody=%3m/s)，已平移地图参考保持连续: %4")
               .arg(dist, 0, 'f', 2)
               .arg(static_cast<int>(dt * 1000.0))
               .arg(vBody, 0, 'f', 2)
               .arg(_map.describe());
}

// 狗端局部 yaw 基准的整体跳变补偿。
//
// 现场实测（2026-11-14 13:01:47）：状态包中断 3~5s 恢复后，狗自报 raw yaw 从
// 118° 直接跳到 -174°（60° 台阶），位置却只变了 0.09m。原因不是狗真转了，而是
// SDK 会话被重置后局部坐标系重新起算（遥控器接管、控制权重新申请都会触发）。
// 不补偿的后果：地图 yaw 从 11° 跳到 311°，随后的"相对旋转 90°"任务立刻判
// "到位"（剩余 7.79°），其实是把跳变后的错朝向当成了目标——假到位。
// 处理办法与位置跳变一致：让参考朝向跟着跳，地图 yaw 保持连续。
//
// 2026-11-15 增强：run8 原始模式取证发现 yaw 单帧跳 >30° 共 271 次，而旧判据
// （必须"断链 >=2s"）只抓到 2 次，其余全部原样进地图——这是"兜圈找不到去向"的
// 主因之一（目标方位随 yaw 一起乱跳）。新判据改用陀螺做独立证据：
//   本帧 yaw 变化 与 陀螺 z 轴积分 之差 > 阈值 → 一定是基准跳了，不是真转。
// 这样 1Hz 帧、不断链的跳变也能被识别；真实转向由陀螺积分保留下来。
void MainController::compensateYawReset(const DogOdom &odom, const DogImu &imu)
{
    const double kRadToDeg = 180.0 / M_PI;

    // 原始模式：yaw 台阶原样进地图
    if (_cfg.poseRawOdom)
    {
        _lastYawValid = false;
        return;
    }

    if (!odom.valid || !_yaw)
        return;

    const double yawNow = _yaw->yaw();
    const qint64 now = nowMs();
    if (!_lastYawValid)
    {
        _lastYawValid = true;
        _lastYawRad = yawNow;
        _lastYawMs = now;
        return;
    }

    const double dYawRad = DogProtocol::radStd(yawNow - _lastYawRad);
    const double gapSec = static_cast<double>(now - _lastYawMs) / 1000.0;
    const double vBody = std::sqrt(static_cast<double>(odom.vBody[0]) * odom.vBody[0]
                                   + static_cast<double>(odom.vBody[1]) * odom.vBody[1]);

    _lastYawRad = yawNow;
    _lastYawMs = now;

    if (!_map.isActive() || !_map.params().yawJumpCompensate)
        return;

    const double threshDeg = _cfg.yawJumpDeg > 1.0 ? _cfg.yawJumpDeg : 15.0;
    if (std::fabs(dYawRad) * kRadToDeg < threshDeg)
        return;                                  // 正常小步，无跳变

    // ---- 判据 A：陀螺独立证据（帧间隔正常、报文带 gyro）----
    // 真实转 Δyaw ≈ gyro_z*dt；两者悬殊说明是基准跳变而非真转。
    if (imu.valid && imu.gyroField && gapSec > 0.05 && gapSec < 1.6)
    {
        const double expectRad = static_cast<double>(imu.gyro[2]) * gapSec;
        const double diffRad   = DogProtocol::radStd(dYawRad - expectRad);
        if (std::fabs(diffRad) * kRadToDeg < threshDeg)
            return;                              // 与陀螺一致 → 真转，不动参考

        _map.shiftRefYaw(diffRad);               // 只抹掉"基准"部分，保留真实转动
        qWarning().noquote()
            << QStringLiteral("[Map] 狗端 yaw 基准跳变 %1°（陀螺证据：本帧实转 %2°，"
                              "差 %3°；vBody=%4m/s、间隔 %5ms），已同步平移参考朝向保持航向连续: %6")
                   .arg(dYawRad * kRadToDeg, 0, 'f', 1)
                   .arg(expectRad * kRadToDeg, 0, 'f', 1)
                   .arg(diffRad * kRadToDeg, 0, 'f', 1)
                   .arg(vBody, 0, 'f', 2)
                   .arg(static_cast<int>(gapSec * 1000.0))
                   .arg(_map.describe());
        return;
    }

    // ---- 判据 B：无陀螺可用时退回旧规则（断链 >=2s 且狗没动）----
    if (gapSec < 2.0 || vBody > 0.15)
        return;

    _map.shiftRefYaw(dYawRad);
    qWarning().noquote()
        << QStringLiteral("[Map] 狗端 yaw 基准跳变 %1°（断链 %2ms、vBody=%3m/s，狗未移动），"
                          "已同步平移参考朝向保持航向连续: %4")
               .arg(dYawRad * kRadToDeg, 0, 'f', 1)
               .arg(static_cast<int>(gapSec * 1000.0))
               .arg(vBody, 0, 'f', 2)
               .arg(_map.describe());
}

