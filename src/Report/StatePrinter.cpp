// ============================================================
// 控制台状态诊断打印 —— 实现
//
// 只读快照、只写控制台：不修改任何控制状态，也不进入报文。
// 打印内容按"原始 → 稳定 → 地图"三段排列，便于一眼看出是传感器跳变
// 还是融合/参考出了问题。
// ============================================================
#include "Report/StatePrinter.h"

#include <cmath>
#include <QDebug>

#include "Common/DogText.h"
#include "Config/TerminalConfig.h"
#include "Control/MotionState.h"
#include "Control/YawStabilizer.h"
#include "Convert/MapTransform.h"
#include "Convert/RtkPoseTracker.h"
#include "DogLink/DogClientBase.h"
#include "DogLink/DogTypes.h"
#include "Protocol/DogProtocol.h"

StatePrinter::StatePrinter(const TerminalConfig &cfg, DogClientBase *&dog, MapTransform &map,
                           YawStabilizer &yaw, RtkPoseTracker &rtk, MotionState &st)
    : _cfg(cfg), _dog(dog), _map(map), _yaw(yaw), _rtk(rtk), _st(st)
{
}

void StatePrinter::print()
{
    if (_cfg.dogStatePrintHz <= 0)
        return;

    const DogImu &imu = _dog->imu();
    const DogOdom &odom = _dog->odom();
    const DogFeedback &fb = _dog->feedback();

    // 严格按 PrintStateHz 节流。旧实现允许"模式变化立即打印"，
    // 而遥控时 functionMode 在 sdk/remote 之间频繁跳动，会绕过节流，
    // 反复打印同一帧缓存数据，日志看起来像"卡住后突然跳一大段"。
    const qint64 interval = qMax<qint64>(1, 1000 / _cfg.dogStatePrintHz);
    const qint64 now = nowMs();
    if (now - _lastDogPrintMs < interval)
        return;
    const qint64 dtMs = (_lastDogPrintMs > 0) ? (now - _lastDogPrintMs) : 0;
    _lastDogPrintMs = now;
    const QString stamp = QDateTime::currentDateTime().toString(
        QStringLiteral("HH:mm:ss.zzz"));

    // 2) 原始 IMU 欧拉角与角速度，不做归一化、滤波或标定换算
    if (imu.valid)
    {
        const double roll  = static_cast<double>(imu.rpy[0]) * kRadToDeg;
        const double pitch = static_cast<double>(imu.rpy[1]) * kRadToDeg;
        const double yaw   = static_cast<double>(imu.rpy[2]) * kRadToDeg;
        // gyro_z 与航向稳定器同源：imu_info 不带 gyro 时用里程计机体角速度，
        // 并在行尾标注来源，避免现场把"恒 0"误读为传感器故障
        const bool gyroFromOdom = !imu.gyroField;
        const double gyroZ = static_cast<double>(imu.gyroField ? imu.gyro[2]
                                                              : (odom.valid ? odom.omegaBody[2] : 0.0f))
                             * kRadToDeg;

        QString dyawText = QStringLiteral("--");
        if (_lastPrintYawValid)
        {
            const double dyawRad = DogProtocol::radStd(imu.rpy[2]
                                   - _lastPrintYaw * 3.14159265358979323846 / 180.0);
            dyawText = QString::number(dyawRad * kRadToDeg, 'f', 2);
        }

        qInfo().noquote()
            << QStringLiteral("[DogState] t=%1 dt=%2ms 姿态 raw rpy[deg]="
                              "(roll %3 | pitch %4 | yaw %5 Δyaw %6) gyro_z=%7°/s "
                              "稳定yaw=%8° 陀螺yaw=%9° 零偏=%10°/s%11")
                   .arg(stamp).arg(dtMs)
                   .arg(roll, 0, 'f', 2)
                   .arg(pitch, 0, 'f', 2)
                   .arg(yaw, 0, 'f', 2)
                   .arg(dyawText)
                   .arg(gyroZ, 0, 'f', 2)
                   .arg(_yaw.yaw() * kRadToDeg, 0, 'f', 2)
                   .arg(_yaw.gyroYaw() * kRadToDeg, 0, 'f', 2)
                   .arg(_st.yawBias * kRadToDeg, 0, 'f', 3)
                   .arg(_yaw.staticLocked() ? QStringLiteral(" [静止锁定]") : QString())
                   + ( _yaw.yawSuspect() ? QStringLiteral("  [航向待校核]") : QString() )
                   + ( gyroFromOdom ? QStringLiteral("  [gyro_z 取自 odom]") : QString() );

        _lastPrintYaw = yaw;
        _lastPrintYawValid = true;
    }
    else
    {
        qInfo().noquote() << QStringLiteral("[DogState] t=%1 姿态 未收到 imu_info").arg(stamp);
    }

    // 3) 位姿、机体系速度；附带帧间增量与推算速度，用于区分"真实位移"和"单帧跳变"
    if (odom.valid)
    {
        const double lx = static_cast<double>(odom.position[0]);
        const double ly = static_cast<double>(odom.position[1]);
        const double lz = static_cast<double>(odom.position[2]);

        QString deltaText = QStringLiteral("--");
        QString speedText = QStringLiteral("--");
        bool suspect = false;
        if (_lastPrintPoseValid)
        {
            const double dx = lx - _lastPrintX;
            const double dy = ly - _lastPrintY;
            const double dist = std::sqrt(dx * dx + dy * dy);
            const double dtSec = (dtMs > 0) ? (dtMs / 1000.0) : 0.0;
            const double speed = (dtSec > 1e-6) ? (dist / dtSec) : 0.0;
            deltaText = QStringLiteral("(%1, %2) |Δ|=%3m")
                            .arg(dx, 0, 'f', 2)
                            .arg(dy, 0, 'f', 2)
                            .arg(dist, 0, 'f', 2);
            speedText = QStringLiteral("%1m/s").arg(speed, 0, 'f', 2);
            // 腿足实际速度上限约 1.2m/s，明显超过即视为可疑跳变
            suspect = (speed > 3.0);
        }

        qInfo().noquote()
            << QStringLiteral("[DogState] t=%1 dt=%2ms 位姿 原始局部(x %3, y %4, z %5) "
                              "Δ=%6 推算速度=%7 机体系速度(vx %8, vy %9, vz %10) "
                              "yawRate %11 rad/s%12")
                   .arg(stamp).arg(dtMs)
                   .arg(lx, 0, 'f', 2)
                   .arg(ly, 0, 'f', 2)
                   .arg(lz, 0, 'f', 2)
                   .arg(deltaText)
                   .arg(speedText)
                   .arg(static_cast<double>(odom.vBody[0]), 0, 'f', 2)
                   .arg(static_cast<double>(odom.vBody[1]), 0, 'f', 2)
                   .arg(static_cast<double>(odom.vBody[2]), 0, 'f', 2)
                   .arg(static_cast<double>(odom.omegaWorld[2]), 0, 'f', 2)
                   .arg(suspect ? QStringLiteral("  <-- 可疑跳变") : QString());

        // 地图坐标（仅启用且已建立参考时打印），便于现场核对局部→地图变换。
        // 换了位置源之后，地图必须用"死推算位姿"（= 导航与上报同一份），
        // 不然这里打出来的地图会和上报/导航对不上，现场会误判成又出了新 bug。
        if (_map.isActive() && imu.valid)
        {
            const double px = _st.valid ? _st.x : lx;
            const double py = _st.valid ? _st.y : ly;
            double mx = 0.0;
            double my = 0.0;
            double myaw = 0.0;
            _map.toMap(px, py, _yaw.yaw(), &mx, &my, &myaw);
            QString fusedText;
            double fmx = 0.0;
            double fmy = 0.0;
            double fmyaw = 0.0;
            int fsrc = RtkFusion::SrcNone;
            if (_rtk.fusedMapPose(&fmx, &fmy, &fmyaw, &fsrc))
            {
                fusedText = QStringLiteral(" | 上报(x %1, y %2, yaw %3°, 源 %4)")
                                .arg(fmx, 0, 'f', 2)
                                .arg(fmy, 0, 'f', 2)
                                .arg(degNorm360(fmyaw * kRadToDeg), 0, 'f', 1)
                                .arg(fsrc == RtkFusion::SrcRtk ? QStringLiteral("差分")
                                                               : QStringLiteral("差分保持"));
            }
            qInfo().noquote()
                << QStringLiteral("[DogState] t=%1 地图(x %2, y %3, yaw %4°, scale %5) "
                                  "死推算局部(x %6, y %7)%8")
                       .arg(stamp)
                       .arg(mx, 0, 'f', 2)
                       .arg(my, 0, 'f', 2)
                       .arg(degNorm360(myaw * kRadToDeg), 0, 'f', 1)
                       .arg(_map.scale(), 0, 'f', 3)
                       .arg(px, 0, 'f', 2)
                       .arg(py, 0, 'f', 2)
                       .arg(fusedText);
        }

        _lastPrintX = lx;
        _lastPrintY = ly;
        _lastPrintPoseValid = true;
    }
    else
    {
        qInfo().noquote() << QStringLiteral("[DogState] t=%1 位姿 未收到 odom_info").arg(stamp);
    }

    // 4) 模式反馈（control 对应站立/趴下/锁定等姿态）
    if (fb.valid)
    {
        // qInfo().noquote()
        //     << QStringLiteral("[DogState] 模式 function=%1(%2) control=%3(%4) "
        //                       "motion=%5(%6) speed=%7(%8)")
        //            .arg(functionModeText(fb.functionMode))
        //            .arg(fb.functionMode)
        //            .arg(controlModeText(fb.controlMode))
        //            .arg(fb.controlMode)
        //            .arg(motionModeText(fb.motionMode))
        //            .arg(fb.motionMode)
        //            .arg(speedLevelText(fb.speedLevel))
        //            .arg(fb.speedLevel);
    }
    else
    {
        qInfo().noquote() << QStringLiteral("[DogState] 模式 未收到 feedback");
    }
}
