#ifndef ZSIBOT_DEFS_H
#define ZSIBOT_DEFS_H

#include <QtGlobal>
#include <QString>

// ============================================================
// 机器狗(genisom L1) SDK UDP+JSON 协议常量定义
// 对齐官方 SDK zsibot_define.h / docs/protocol/protocol.md
// 协议要点：
//   - 通信方式：UDP + JSON 纯文本
//   - 默认狗IP 192.168.234.1，控制口 8081，状态口 8080
//   - 心跳间隔 50ms，5s 无数据判定断连
//   - SDK角色需先申请 CMD_SDK_CONTROL_RIGHT 才能下发运动指令
// ============================================================
namespace Zsibot {

// 命令码（JSON "cmd" 字段值）
enum CmdCode
{
    CMD_NULL                 = 0x00,
    CMD_EMERGENCY_STOP       = 0x5A,  // 紧急停止（软急停，趴下）
    CMD_STAND_UP             = 0x7A,  // 站立
    CMD_SIT_DOWN             = 0x6A,  // 趴下
    CMD_MOVE_MODE            = 0x8A,  // 移动模式
    CMD_BALANCE_STAND_MODE   = 0x9A,  // 平衡站立模式
    CMD_LOCK_MODE            = 0x9B,  // 锁定模式
    CMD_SLOW_SPEED           = 0xAE,  // 慢速
    CMD_NORMAL_SPEED         = 0xAF,  // 正常速度
    CMD_FAST_SPEED           = 0xB0,  // 快速
    CMD_JUMP                 = 0x01,
    CMD_FORWARD_JUMP         = 0x02,
    CMD_BACK_FLIP            = 0x03,
    CMD_GREET                = 0x04,
    CMD_TWO_LEG_STAND        = 0x05,
    CMD_CRAWL_FORWARD        = 0x09,
    CMD_CLIMBING_HIGH_PLATFORM = 0x0A,
    CMD_HAND_STAND           = 0x0B,
    CMD_UNLOAD_SQUAT         = 0x0C,
    CMD_ENTER_LAB_MODE       = 0xC1,
    CMD_EXIT_LAB_MODE        = 0xC2,
    CMD_ENTER_ENTERTAINMENT_MODE = 0xC3,
    CMD_EXIT_ENTERTAINMENT_MODE  = 0xC4,
    CMD_REMOTE_CONTROL_RIGHT = 0xAA,
    CMD_SDK_CONTROL_RIGHT    = 0xB3,  // 申请SDK控制权
    CMD_ROAMERX_CONTROL_RIGHT = 0xB4,
    CMD_GENERAL_SDK_CONTROL_RIGHT = 0xB5
};

// 角色（JSON "role" 字段值）
inline QString roleRemote() { return QStringLiteral("remote"); }
inline QString roleSdk()    { return QStringLiteral("sdk"); }

// ---------- 以下为狗端上报 JSON 中的字符串枚举到 SDK 数值的映射 ----------

// feedback.function: remote/trace/sdk/general_sdk/roamer → FunctionMode
// 返回 -1 表示未识别
inline int functionModeFromString(const QString &s)
{
    if (s == QLatin1String("remote"))      return 1;
    if (s == QLatin1String("trace"))       return 2;
    if (s == QLatin1String("sdk"))         return 3;
    if (s == QLatin1String("general_sdk")) return 4;
    if (s == QLatin1String("roamer"))      return 5;
    return -1;  // TODO: verify 未知字符串按需补充
}

// feedback.control_mode → ControlMode
inline int controlModeFromString(const QString &s)
{
    if (s == QLatin1String("stand_up"))  return 1;
    // 固件可能上报 stand_down 或 sit_down，两种写法都按"趴下"处理
    if (s == QLatin1String("stand_down") || s == QLatin1String("sit_down")) return 2;
    if (s == QLatin1String("lock"))      return 3;
    if (s == QLatin1String("emergency")) return 4;
    if (s == QLatin1String("move"))      return 5;
    // 固件可能上报 stay 或 balance_stand，两种写法都按"平衡站立"处理
    if (s == QLatin1String("stay") || s == QLatin1String("balance_stand")) return 6;
    if (s == QLatin1String("init"))      return 7;
    return -1;
}

// feedback.motion_mode → MotionMode
inline int motionModeFromString(const QString &s)
{
    if (s == QLatin1String("running")) return 1;
    if (s == QLatin1String("rest"))    return 2;
    if (s == QLatin1String("forbid"))  return 3;
    return -1;
}

// feedback.motion_type → MotionType
inline int motionTypeFromString(const QString &s)
{
    if (s == QLatin1String("up_jump"))         return 1;
    if (s == QLatin1String("front_jump"))      return 2;
    if (s == QLatin1String("back_slip"))       return 3;  // 后空翻
    if (s == QLatin1String("greet"))           return 4;
    if (s == QLatin1String("tow_leg_stand"))   return 5;  // 固件拼写即为 tow
    if (s == QLatin1String("forbid"))          return 6;
    if (s == QLatin1String("unknown"))         return 7;
    if (s == QLatin1String("unloading_squat")) return 8;
    return -1;
}

// speed_set: low/mid/high → SpeedLevel
inline int speedLevelFromString(const QString &s)
{
    if (s == QLatin1String("low"))  return 1;
    if (s == QLatin1String("mid"))  return 2;
    if (s == QLatin1String("high")) return 3;
    return -1;
}

// SDK FunctionMode：SDK模式（已取得控制权）
const int FM_SDK = 3;

}  // namespace Zsibot

#endif  // ZSIBOT_DEFS_H
