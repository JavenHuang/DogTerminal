#ifndef MATRIX_DEFS_H
#define MATRIX_DEFS_H

// ============================================================
// MATRiX (Genisom ZSL1/ZSL2) 控制协议常量
// 下行：UDP 7447，JSON 摇杆 + 按键
// 上行：Zenoh rt/**，CDR 格式（ROS 2 XCDR1）
// ============================================================
namespace Matrix {

// ---- 摇杆范围 ----
constexpr float kJoystickMin = -1.0f;
constexpr float kJoystickMax =  1.0f;

// ---- 控制频率 ----
constexpr int kControlRateHz = 20;       // Matrix_MC 期望的持续发送频率
constexpr int kNeutralHoldMs = 100;      // 按键脉冲的 neutral 前后等待
constexpr int kPulseHoldMs = 150;        // 按键保持时长（Matrix_MC 上升沿触发）
constexpr int kDeadbandInt = 1200;       // 摇杆死区阈值（整数 0~32768，约 0.036）

// ---- 摇杆字段 ----
// Matrix_MC 内部映射：forward→LY, lateral→-LX, yaw→-RX
// 这些偏移量来自 matrix_mc_udp_test.py 官方测试脚本
constexpr int kLyForwardSign =  1;       // 正向：前后
constexpr int kLxLateralSign = -1;       // 负向：横移左→正、右→负
constexpr int kRxYawSign     = -1;       // 负向：左转→正、右转→负

}  // namespace Matrix

// ---- L1 命令码 → MATRiX 按键组合 ----
// 按键组合用 "LB,Y" 格式，由 MatrixDogClient 解析成 JSON 字段
namespace MatrixCmdMap {

struct CmdMapping {
    const char *buttons;   // 按键组合字符串（逗号分隔）
    const char *action;    // 动作描述，用于日志
};

// L1 CmdCode 定义（来自 zsibot_define.h）：
//   CMD_EMERGENCY_STOP       = 0x5A
//   CMD_SIT_DOWN             = 0x6A
//   CMD_STAND_UP             = 0x7A
//   CMD_MOVE_MODE            = 0x8A
//   CMD_SDK_CONTROL_RIGHT    = 0xB3
//
// MATRiX 按键组合（来自 matrix_mc_udp_test.py + 官方文档）：
//   LB+Y = 起身站立    LB+B = 平衡站立    LB+X = 关节保持    LB+RB = 被动模式
//   RB+X = 跳跃       RB+Y = 前跳       RB+B = 后空翻      RB+VIEW = 侧翻
inline CmdMapping fromL1Cmd(quint32 cmdCode)
{
    switch (cmdCode)
    {
    case 0x7A:  // CMD_STAND_UP
        return {"LB,Y", "起身站立"};
    case 0x6A:  // CMD_SIT_DOWN
        return {"LB,X", "关节保持（趴下近似）"};
    case 0x5A:  // CMD_EMERGENCY_STOP
        return {"LB,RB", "被动模式（释放力矩）"};
    default:
        return {nullptr, nullptr};
    }
}

inline bool isKnownCmd(quint32 cmdCode)
{
    return fromL1Cmd(cmdCode).buttons != nullptr;
}

}  // namespace MatrixCmdMap

#endif  // MATRIX_DEFS_H
