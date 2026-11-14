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

    // 先做里程计跳变补偿，保证后续参考建立/坐标变换使用的局部位置连续
    compensateOdomJump(odom);
    // yaw 稳定化：之后所有导航/地图/上报统一用 _yaw->yaw()，不再直接用 imu.rpy[2]
    _yaw->update(odom, imu);
    compensateYawReset(odom);
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



void MainController::applyMapCalibToFusion()
{
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
void MainController::compensateYawReset(const DogOdom &odom)
{
    const double kRadToDeg = 180.0 / M_PI;

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

    // 只判"断过链 + 狗没动"的组合：真实转向一定伴随速度或角速度，
    // 正常 1Hz/20Hz 相邻帧也不会有 30° 以上台阶
    if (gapSec < 2.0 || std::fabs(dYawRad) * kRadToDeg < 30.0)
        return;
    if (vBody > 0.15)
        return;
    if (!_map.isActive() || !_map.params().jumpCompensate)
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

