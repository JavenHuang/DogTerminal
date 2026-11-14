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

    // 先做里程计跳变补偿，保证后续参考建立/坐标变换使用的局部位置连续
    compensateOdomJump(odom);
    // yaw 稳定化：之后所有导航/地图/上报统一用 _yaw->yaw()，不再直接用 imu.rpy[2]
    _yaw->update(odom, imu);
    compensateYawReset(odom, imu);
    // 位移方向校核：用直线前进的里程计位移方向独立验证/重建航向基准
    _yaw->updateMotionHeading(odom, imu);

    // 参考建立：终端启动后首个有效位姿自动建立（当前点=O点 朝向=0°正东），
    // 与狗开机时刻无关——狗可以先开机、先被遥控移动
    if (_map.params().enabled && _map.params().autoReference
        && !_map.hasReference() && odom.valid && imu.valid)
    {
        resetMapReference(odom.position[0], odom.position[1], _yaw->yaw());
        qInfo().noquote() << "[Map] 参考已建立（终端启动时狗位姿=O点/0°正东）:" << _map.describe();
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
            const double nx = odom.position[0];
            const double ny = odom.position[1];
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
            _st.y   = ny;
            _st.vx  = odom.vBody[0];
        }
        _st.yaw     = _yaw->yaw();
        // 上报用的航向角速度：与 YawStabilizer 同源，gyro 缺失时退回里程计机体角速度
        _st.yawRate = static_cast<double>(imu.gyroField ? imu.gyro[2]
                                                        : (odom.valid ? odom.omegaBody[2] : 0.0f));
        _st.ms      = nowMs();
        _st.valid   = true;
        _st.staleStopped = false;   // 状态恢复，解除盲发暂停
    }

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
    // 尺度是硬件特性，重建参考后仍然有效
    _map.setScale(_yaw->calibScale());
    // 融合偏移与重启判据基准都绑在旧参考点上，交给跟踪器统一作废（尺度/转角继续沿用）
    _rtk->onMapRefReset();
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



void MainController::beginLineTrace()
{
    const DogOdom &odom = _dog->odom();
    _traceActive   = true;
    _traceStartX   = odom.position[0];
    _traceStartY   = odom.position[1];
    _traceLastX    = _traceStartX;
    _traceLastY    = _traceStartY;
    _tracePath     = 0.0;
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
    // 单帧位移超过 1.5m 视为里程计跳变（遥控接管/坐标系重置），不计入路径长度。
    // 旧值 0.5m 太紧：状态包 1Hz、快速行走时单帧步长本身就有 0.5~0.8m，
    // 会被当成跳变丢掉，路径反而比位移短（直线度打出 75%~79% 的假象）。
    const double step = std::sqrt((odom.position[0] - _traceLastX) * (odom.position[0] - _traceLastX)
                                  + (odom.position[1] - _traceLastY) * (odom.position[1] - _traceLastY));
    if (step < 1.5)
        _tracePath += step;
    _traceLastX = odom.position[0];
    _traceLastY = odom.position[1];
    ++_tracePkts;

    const qint64 t = nowMs();
    if (_tracePrintMs != 0 && (t - _tracePrintMs) < 1000)
        return;
    _tracePrintMs = t;

    const double dx = odom.position[0] - _traceStartX;
    const double dy = odom.position[1] - _traceStartY;
    const double disp = std::sqrt(dx * dx + dy * dy);

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

    qWarning().noquote() << line;
}

void MainController::endLineTrace(double targetMapX, double targetMapY)
{
    if (!_traceActive)
        return;
    _traceActive = false;

    const DogOdom &odom = _dog->odom();
    const double dx = odom.position[0] - _traceStartX;
    const double dy = odom.position[1] - _traceStartY;
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

