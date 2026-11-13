#ifndef MOTION_STATE_H
#define MOTION_STATE_H

#include <QtGlobal>

// ============================================================
// 姿态快照（纯数据，无行为）
//
// 由"里程计/IMU 入库路径"写，被差分、导航、上报三条路读取。
// 之所以单独成结构：它原本散落在 MainController 里（_stX/_stY/_stYaw/
// _stSpeed…），却是三个不同职责的共同输入，放在任何一方都不合适。
//
// 约定：
//   yaw    —— 已稳定的局部朝向（逆时针为正，rad），不是 IMU 原始值
//   speed  —— 位置差分推算的水平速度（低通），差分的静止门控用它
//   staticSinceMs —— 静止起点，yaw 零偏估计与差分零偏取数共用
//   yawBias—— 静止零偏估计（rad/s），yaw 稳定器写、差分 Δψ 校核读
// ============================================================
struct MotionState
{
    bool valid = false;          // 已收到过可用于导航的状态
    double x = 0.0;              // 最近一次原始局部 x
    double y = 0.0;              // 最近一次原始局部 y
    double yaw = 0.0;            // 最近一次稳定后朝向（逆时针为正）
    double vx = 0.0;             // 机体系前向速度（增益自适应/阻尼用）
    double speed = 0.0;          // 位置差分推算的水平速度（低通）
    double yawRate = 0.0;        // 最近一次 gyro[2]
    qint64 ms = 0;               // 最近一次状态到达时刻
    bool staleStopped = false;   // 状态长时间中断后已暂停盲发（避免每拍重复零速）

    qint64 staticSinceMs = 0;    // 静止起点（0 表示未判定静止）
    double yawBias = 0.0;        // 静止零偏估计 rad/s
};

#endif  // MOTION_STATE_H
