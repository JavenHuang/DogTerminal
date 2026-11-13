#ifndef STATE_PRINTER_H
#define STATE_PRINTER_H

#include <QtGlobal>

class TerminalConfig;
class MapTransform;
class YawStabilizer;
class RtkPoseTracker;
struct MotionState;
class DogClientBase;

// ============================================================
// 控制台状态诊断打印
//
// 职责（只有这一件事）：按配置频率把"原始位姿 + 地图位姿 + 航向诊断量"打到控制台，
// 用于现场核对方向/漂移/跳变，不参与任何控制与上报。
//
// 与上报分离的理由：上位机报文只需要稳定字段，而现场排障需要的是
// 原始 imu/odom、陀螺积分、跳变计数、门控原因这些"只给人看"的量；
// 两者节奏（1 Hz 报文 vs 可配打印频率）与生命周期都不同。
//
// 打印节流与"上一次打印值"属于本类自身的状态，随本类一起搬出协调者。
// ============================================================
class StatePrinter
{
public:
    StatePrinter(const TerminalConfig &cfg, DogClientBase *&dog, MapTransform &map,
                 YawStabilizer &yaw, RtkPoseTracker &rtk, MotionState &st);
    ~StatePrinter() = default;

    StatePrinter(const StatePrinter &) = delete;
    StatePrinter &operator=(const StatePrinter &) = delete;

    // 每帧姿态到达后调用；内部按 dogStatePrintHz 节流
    void print();

private:
    const TerminalConfig &_cfg;
    DogClientBase *&_dog;
    MapTransform &_map;
    YawStabilizer &_yaw;
    RtkPoseTracker &_rtk;
    MotionState &_st;

    qint64 _lastDogPrintMs = 0;       // 上次打印时刻（节流用）
    bool _lastPrintPoseValid = false; // 上一次打印的原始位姿是否有效
    double _lastPrintX = 0.0;
    double _lastPrintY = 0.0;
    double _lastPrintYaw = 0.0;
    bool _lastPrintYawValid = false;
};

#endif  // STATE_PRINTER_H
