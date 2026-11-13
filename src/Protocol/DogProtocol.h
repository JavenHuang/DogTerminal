#ifndef DOG_PROTOCOL_H
#define DOG_PROTOCOL_H

#include <QtGlobal>
#include <QDataStream>
#include <QString>

// ============================================================
// 机器狗终端 <-> 服务器 通信协议定义
// 协议形式：Qt枚举 + 结构体，通过 QDataStream 序列化后作为
//           RabbitMQ 消息体传输。
// 兼容性：序列化版本固定为 QDataStream::Qt_5_12，
//         保证服务器端(Qt5.12)与终端(Qt5.15)互通。
// 当前 DogTerminal 与内置 simulator 共用本协议；坐标与姿态字段布局不变，
// 其语义由 DogState::calibrated 区分：
//   true  → 本地地图坐标（+x 东 +y 南，朝向角 0=正东，顺时针为正）
//   false → 狗原始局部坐标（+x 前 +y 左，yaw 逆时针为正）
// ============================================================
namespace DogProtocol {

const quint8 PROTOCOL_VERSION = 2;

// 消息类型
enum MessageType
{
    MSG_HEARTBEAT     = 0x01,  // 双向心跳（终端→服务器 状态在线；服务器→终端 启用任务级看门狗）
    MSG_STATE_REPORT  = 0x02,  // 终端→服务器 状态上报
    MSG_TASK_FEEDBACK = 0x03,  // 终端→服务器 任务执行反馈
    MSG_ATTITUDE_CMD  = 0x11,  // 服务器→终端 姿态控制（离散模式命令）
    MSG_MOVE_CMD      = 0x12,  // 服务器→终端 机动控制（目标点）
    MSG_VELOCITY_CMD  = 0x13,  // 服务器→终端 速度直控
    MSG_STOP_CMD      = 0x14,  // 服务器→终端 停止/急停
    MSG_CALIBRATION   = 0x15,  // 保留线协议 ID，终端不再处理坐标标定
    MSG_PARAM_CMD     = 0x16,  // 服务器→终端 参数设置（速度档位）
    MSG_ROTATE_REL_CMD= 0x17,  // 服务器→终端 相对当前朝向旋转
    MSG_ROTATE_ABS_CMD= 0x18,  // 服务器→终端 转到绝对局部 yaw
    MSG_MAP_CALIB_CMD = 0x19,  // 服务器→终端 地图参考重置 / 直线标定结束
    MSG_ABS_POSE_CMD  = 0x1A   // 服务器→终端 差分(RTK)绝对位姿（约 200ms 周期）
};

// 终端工作状态
enum TerminalStatus
{
    TS_INIT = 0,           // 初始化
    TS_CONNECTING_DOG,     // 连接机器狗中
    TS_DOG_CONNECTED,      // 已连接机器狗（未取得控制权）
    TS_READY,              // 就绪（已取得SDK控制权）
    TS_EXECUTING,          // 任务执行中
    TS_FAULT               // 故障
};

// 姿态控制动作（离散模式命令），一一映射到机器狗SDK的CmdCode
enum AttitudeAction
{
    AA_NONE = 0,
    AA_STAND_UP        = 1,   // 站立
    AA_SIT_DOWN        = 2,   // 趴下
    AA_BALANCE_STAND   = 3,   // 平衡站立
    AA_LOCK            = 4,   // 锁定
    AA_EMERGENCY_STOP  = 5,   // 紧急停止
    AA_MOVE_MODE       = 6,   // 进入移动模式
    AA_SLOW_SPEED      = 7,   // 慢速档
    AA_NORMAL_SPEED    = 8,   // 正常速度档
    AA_FAST_SPEED      = 9,   // 快速档
    AA_REQUEST_CONTROL = 10   // 申请SDK控制权
};

// 机动控制执行结果
enum MoveResult
{
    MR_ACCEPTED = 0,   // 已接受
    MR_REJECTED = 1,   // 拒绝（忙/未就绪/参数非法）
    MR_MOVING   = 2,   // 执行中
    MR_ARRIVED  = 3,   // 到达目标点或目标航向
    MR_TIMEOUT  = 4,   // 执行超时
    MR_FAILED   = 5    // 失败（失联/被抢占/急停等）
};

// 机器狗速度档位（与SDK SpeedLevel 对齐）
enum SpeedLevel
{
    SL_NONE = 0,
    SL_SLOW = 1,
    SL_NORMAL = 2,
    SL_FAST = 3
};

// 地图标定动作（MSG_MAP_CALIB_CMD）
enum MapCalibAction
{
    MC_RESET    = 0,  // 以当前位姿重建参考（当前点=O点 朝向=0°正东）
    MC_END_LINE = 1,  // 直线标定结束：targetX/targetY 为终点地图坐标
    MC_DISABLE  = 2,  // 关闭地图坐标变换（回到原始局部坐标）
    MC_CHECK_LINE = 3 // 直线标定预检：只计算不修改，返回当前残余角/尺度
};

// 消息头（消息体前的定长头部）
struct MessageHeader
{
    quint8 version;      // 协议版本
    quint16 msgType;     // 消息类型 MessageType
    quint32 terminalId;  // 终端ID
    qint64 timestampMs;  // 发送时间戳（毫秒）

    MessageHeader()
        : version(PROTOCOL_VERSION), msgType(0), terminalId(0), timestampMs(0) {}
    MessageHeader(quint16 type, quint32 id)
        : version(PROTOCOL_VERSION), msgType(type), terminalId(id), timestampMs(0) {}
};

// 终端/机器狗状态
struct DogState
{
    bool dogConnected;      // 机器狗连接状态
    bool controlAcquired;   // 是否已取得SDK控制权
    qint32 status;          // TerminalStatus

    // 位置与 yaw（单位：米/弧度）
    // calibrated=true 时为本地地图坐标（+x 东 +y 南，朝向角 0=正东顺时针为正）；
    // false 时为狗原始局部坐标（+x 前 +y 左，yaw 逆时针为正）
    double x, y, z;
    double yaw, pitch, roll;

    double vWorldX, vWorldY, vWorldZ;  // 狗局部参考系速度 m/s（字段名保留兼容）
    double bodyVx, bodyVy, bodyVz;     // 机体系速度 m/s
    double yawRate;                    // 局部参考系 z 轴角速度 rad/s

    qint32 batteryPercent;  // 电量百分比（-1未知）
    double batteryVolt;     // 电池电压 V
    double temperature;     // 设备温度 ℃

    quint8 functionMode;    // SDK FunctionMode
    quint8 controlMode;     // SDK ControlMode
    quint8 speedLevel;      // SpeedLevel
    quint8 faultCount;      // 当前故障数

    // 保留线协议布局；true 表示 x/y/yaw 为本地地图坐标，false 为原始局部坐标。
    bool calibrated;

    // 狗原始局部坐标（永远填充，与 calibrated 无关，供服务端叠加对比显示）
    // +x 机头前方 +y 机体左侧，localYaw 逆时针为正
    double localX, localY, localYaw;

    DogState()
        : dogConnected(false), controlAcquired(false), status(TS_INIT)
        , x(0), y(0), z(0), yaw(0), pitch(0), roll(0)
        , vWorldX(0), vWorldY(0), vWorldZ(0)
        , bodyVx(0), bodyVy(0), bodyVz(0), yawRate(0)
        , batteryPercent(-1), batteryVolt(0), temperature(0)
        , functionMode(0), controlMode(0), speedLevel(0), faultCount(0)
        , calibrated(false)
        , localX(0), localY(0), localYaw(0) {}
};

struct StateReport
{
    DogState state;
};

struct HeartbeatMsg
{
    qint32 status;  // TerminalStatus
    HeartbeatMsg() : status(TS_INIT) {}
};

struct AttitudeCmd
{
    qint32 action;  // AttitudeAction
    AttitudeCmd() : action(AA_NONE) {}
};

// 机动控制命令：下发一个目标点
// 地图坐标启用时 (x,y) 为本地地图坐标；否则为狗局部坐标
struct MoveCmd
{
    quint32 taskId;      // 任务ID（用于反馈关联）
    double x, y;         // 目标点坐标 m
    double maxSpeed;     // 最大线速度 m/s（<=0用默认）
    double arriveRadius; // 到达半径 m（<=0用默认）
    qint64 timeoutMs;    // 超时 ms（<=0 不限）

    MoveCmd()
        : taskId(0), x(0), y(0), maxSpeed(0.0), arriveRadius(0.0), timeoutMs(0) {}
};

// 旋转命令：MSG_ROTATE_REL_CMD 时 angleRad 为相对增量（正左负右），与坐标系无关；
// MSG_ROTATE_ABS_CMD 时 angleRad 为目标 yaw（地图坐标启用时为地图 yaw，
// 否则为狗局部 yaw）。
struct RotateCmd
{
    quint32 taskId;
    double angleRad;
    double maxTurnRate;  // 最大角速度 rad/s（<=0 用终端默认值）
    double toleranceRad; // 到位容差 rad（<=0 用终端导航容差）
    qint64 timeoutMs;    // 超时 ms（<=0 不限）

    RotateCmd()
        : taskId(0), angleRad(0.0), maxTurnRate(0.0), toleranceRad(0.0), timeoutMs(0) {}
};

// 地图标定命令
//   MC_RESET：忽略 targetX/targetY，以当前位姿重建开机参考
//   MC_END_LINE：targetX/targetY 为本次直线标定的终点地图坐标
//   MC_DISABLE：忽略 targetX/targetY
//   MC_CHECK_LINE：同 MC_END_LINE 的输入，但只预检不修改
struct MapCalibCmd
{
    qint32 action;     // MapCalibAction
    quint32 taskId;    // 反馈关联ID
    double targetX;    // 终点地图 x m（MC_END_LINE 使用）
    double targetY;    // 终点地图 y m（MC_END_LINE 使用）

    MapCalibCmd()
        : action(MC_RESET), taskId(0), targetX(0.0), targetY(0.0) {}
};

// 差分(RTK)绝对位姿（MSG_ABS_POSE_CMD / MQ 路由键 la）
//
// 坐标与角度口径（与终端地图系完全一致，不需要二次换算）：
//   位置：本地 ENU 平面，x = 东、y = 南（北方向为 -y），z = 天顶；单位 m。
//   航向：单位 度，正东 = 0°，顺时针 0~360°（90=南、180=西、270=北）。
// 因此终端只需把度转弧度并归一化：yawMap = radStd(yawDeg / kRadToDeg)。
//
// 第一阶段用途：只观测、只打印、只统计与里程计航向的偏差，不参与导航。
struct AbsPoseCmd
{
    qint32 diffId;   // 差分/定位单元 ID（多设备同主题时用于区分，0=不校验）
    double x;         // 东向位置 m
    double y;         // 南向位置 m
    double z;         // 高程 m（仅记录，不参与平面导航）
    double yawDeg;    // 航向 度（正东 0，顺时针 0~360）

    AbsPoseCmd()
        : diffId(0), x(0), y(0), z(0), yawDeg(0) {}
};

// 速度直控命令（机体系连续控制，服务器需周期发送，超时自动零速）
struct VelocityCmd
{
    double vx, vy;      // 机体系线速度 m/s
    double yawRate;     // 偏航角速度 rad/s

    VelocityCmd() : vx(0), vy(0), yawRate(0) {}
};

struct StopCmd
{
    bool emergency;  // true=软急停(趴下)，false=零速停车
    StopCmd() : emergency(false) {}
};

// 旧坐标标定线协议占位：仅为保持消息兼容性保留，局部坐标模式不再使用。
struct CalibrationData
{
    double originX, originY, originZ;  // 狗当前的世界坐标 m（由服务端填入）
    double originYaw;                  // 狗当前的世界朝向 worldYaw 弧度（由服务端填入）

    CalibrationData()
        : originX(0), originY(0), originZ(0), originYaw(0) {}
};

struct ParamCmd
{
    qint32 speedLevel;  // SpeedLevel
    ParamCmd() : speedLevel(SL_NORMAL) {}
};

// 任务执行反馈
struct TaskFeedback
{
    quint32 taskId;
    qint32 result;      // MoveResult
    double currentX, currentY;  // 反馈时狗原始局部坐标
    double remainingDist;       // 距目标剩余距离 m
    QString message;

    TaskFeedback()
        : taskId(0), result(MR_ACCEPTED)
        , currentX(0), currentY(0), remainingDist(0) {}
};

// ---------- 序列化（统一 Qt_5_12 版本） ----------
QDataStream &operator<<(QDataStream &s, const MessageHeader &h);
QDataStream &operator>>(QDataStream &s, MessageHeader &h);
QDataStream &operator<<(QDataStream &s, const DogState &st);
QDataStream &operator>>(QDataStream &s, DogState &st);
QDataStream &operator<<(QDataStream &s, const StateReport &m);
QDataStream &operator>>(QDataStream &s, StateReport &m);
QDataStream &operator<<(QDataStream &s, const HeartbeatMsg &m);
QDataStream &operator>>(QDataStream &s, HeartbeatMsg &m);
QDataStream &operator<<(QDataStream &s, const AttitudeCmd &m);
QDataStream &operator>>(QDataStream &s, AttitudeCmd &m);
QDataStream &operator<<(QDataStream &s, const MoveCmd &m);
QDataStream &operator>>(QDataStream &s, MoveCmd &m);
QDataStream &operator<<(QDataStream &s, const RotateCmd &m);
QDataStream &operator>>(QDataStream &s, RotateCmd &m);
QDataStream &operator<<(QDataStream &s, const MapCalibCmd &m);
QDataStream &operator>>(QDataStream &s, MapCalibCmd &m);
QDataStream &operator<<(QDataStream &s, const AbsPoseCmd &m);
QDataStream &operator>>(QDataStream &s, AbsPoseCmd &m);
QDataStream &operator<<(QDataStream &s, const VelocityCmd &m);
QDataStream &operator>>(QDataStream &s, VelocityCmd &m);
QDataStream &operator<<(QDataStream &s, const StopCmd &m);
QDataStream &operator>>(QDataStream &s, StopCmd &m);
QDataStream &operator<<(QDataStream &s, const CalibrationData &m);
QDataStream &operator>>(QDataStream &s, CalibrationData &m);
QDataStream &operator<<(QDataStream &s, const ParamCmd &m);
QDataStream &operator>>(QDataStream &s, ParamCmd &m);
QDataStream &operator<<(QDataStream &s, const TaskFeedback &m);
QDataStream &operator>>(QDataStream &s, TaskFeedback &m);

// ---------- 消息打包/解包（头部+载荷） ----------
// 打包：自动填充时间戳后写头部，再附载荷
QByteArray packMessage(const MessageHeader &header, const QByteArray &body);

// 解包：校验协议版本，成功返回true，body为消息载荷
bool unpackMessage(const QByteArray &raw, MessageHeader &header, QByteArray &body);

// 角度归一化到 (-PI, PI]（注意：与弧度PI比较，而非度数）
double radStd(double rad);

}  // namespace DogProtocol

#endif  // DOG_PROTOCOL_H
