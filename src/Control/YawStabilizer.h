#ifndef YAW_STABILIZER_H
#define YAW_STABILIZER_H

#include <QtGlobal>

#include "DogLink/DogTypes.h"   // DogOdom / DogImu

struct MotionState;
struct TerminalConfig;
class Navigator;

// ============================================================
// 航向稳定器
//
// 职责（只有这一件事）：把 IMU 的原始 yaw 变成"可信的朝向"，并维护两件制约它的量。
//   · 静止零偏在线估计 + 静止锁：静止时用陀螺均值估零偏，抑制慢漂
//   · 跳变抑制：raw yaw 突变但无位移证据时，用陀螺积分保持连续性
//   · 位移方向校验：直线前进时用里程计位移方向独立校核 / 重建航向基准
//   · 直线尺度标定：mapcalib_<id>.ini 的读写（尺度属于转向/位移硬件特性）
//
// 明确不负责：地图参考的建立（由 MainController 在参考变更时通知）、
// 位姿融合（RtkPoseTracker）、导航与上报。
//
// 输出：yaw() 是喂给导航/地图/上报的唯一朝向来源，外部不要再直接读 imu.rpy[2]。
// 依赖以引用注入，本类不持有也不销毁这些对象。
// ============================================================
class YawStabilizer
{
public:
    YawStabilizer(const TerminalConfig &cfg, Navigator &navigator,
                  MotionState &st, bool &velCtrlActive);
    ~YawStabilizer() = default;

    YawStabilizer(const YawStabilizer &) = delete;
    YawStabilizer &operator=(const YawStabilizer &) = delete;

    // 每帧里程计/IMU 入库：先稳定航向，再做位移方向校验
    void update(const DogOdom &odom, const DogImu &imu);
    void updateMotionHeading(const DogOdom &odom, const DogImu &imu);

    // 稳定后的局部 yaw（逆时针为正，rad）
    double yaw() const { return _yawCorrected; }

    // 外部真值接管航向（差分标定后角度即绝对真值）：把局部 yaw 直接钉到指定值，
    // 并重置陀螺连续性状态（连续推算、raw 对齐、运动窗口）。此后 update() 从该值
    // 继续陀螺积分桥接；运行期不再触发位移重建/raw 重锚/可疑裁决。
    void forceReference(double localYawRad);

    // 诊断量（只读）：陀螺积分推算的连续 yaw、静止锁状态、航向是否待校核
    double gyroYaw() const { return _yawGyro; }
    bool staticLocked() const { return _staticLocked; }
    bool yawSuspect() const { return _yawSuspect; }

    // 持久化的直线标定尺度
    double calibScale() const { return _calibScale; }
    void loadCalibScale();
    void saveCalibScale(double scale);

private:
    const TerminalConfig &_cfg;
    Navigator &_navigator;
    MotionState &_st;
    bool &_velCtrlActive;

    bool _yawInit = false;
    double _yawCorrected = 0.0;   // 稳定后的局部 yaw（逆时针为正）
    double _lastRawYaw = 0.0;
    double _gyroAccum = 0.0;      // 自上次 raw yaw 变化以来的陀螺积分 rad
    double _yawGyro = 0.0;        // 陀螺积分推算的连续 yaw（raw 停更/突变时保持连续性）
    bool _yawSuspect = false;     // 航向基准可疑（检测到跳变且尚无位移证据）
    qint64 _suspectSinceMs = 0;
    qint64 _lastYawMs = 0;
    bool _staticLocked = false;
    bool _gyroFallbackLogged = false;   // 已提示过"imu 无 gyro 字段、改用里程计角速度"

    // 位移方向校验：直线前进窗口
    bool _dispYawValid = false;
    double _dispYaw = 0.0;        // 最近一次有效直线位移方向（局部坐标，rad）
    qint64 _dispYawMs = 0;
    bool _mwActive = false;
    double _mwStartX = 0.0;
    double _mwStartY = 0.0;
    double _mwPrevX = 0.0;
    double _mwPrevY = 0.0;
    double _mwLen = 0.0;
    qint64 _mwStartMs = 0;

    double _calibScale = 1.0;     // 持久化的直线标定尺度
};

#endif  // YAW_STABILIZER_H
