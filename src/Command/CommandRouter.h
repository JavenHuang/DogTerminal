#ifndef COMMAND_ROUTER_H
#define COMMAND_ROUTER_H

#include <QByteArray>

#include "Command/CommandHost.h"

class TerminalConfig;
class Navigator;
class AttitudeExecutor;
class MapTransform;
class YawStabilizer;
class DogClientBase;
class ReportService;
struct MotionState;

// ============================================================
// 下行指令处理器：报文解析 + 前置条件校验 + 下发
//
// 本类只做"一条指令进来 → 校验 → 驱动某个执行器"这件事，
// 不持有链路、定时器、控制权等运行状态，也不读时钟。
// 前置条件用到的三个运行标志（控制权、速度直控、任务活跃）以引用注入，
// 保证与主控制器看到的是同一份状态，而不是副本。
// ============================================================
class CommandRouter
{
public:
    CommandRouter(const TerminalConfig &cfg, Navigator &navigator, AttitudeExecutor &attitude,
                  MapTransform &map, YawStabilizer &yaw, DogClientBase *&dog,
                  bool &controlAcquired, bool &velCtrlActive, ReportService &report,
                  double &velFwd, double &velTurn, double &velShift, CommandHost &host,
                  MotionState &state);

    void handleAttitudeCmd(const QByteArray &body);
    void handleMoveCmd(const QByteArray &body);
    void handleRotateCmd(const QByteArray &body, bool absolute);
    void handleMapCalibCmd(const QByteArray &body);
    void handleVelocityCmd(const QByteArray &body);
    void handleStopCmd(const QByteArray &body);
    void handleParamCmd(const QByteArray &body);

    // 速度直控被拒的统一出口：2s 节流，避免服务器高频连发把终端刷满
    void rejectVelocityCmd(const QString &reason);

private:
    const TerminalConfig &_cfg;
    Navigator &_navigator;
    AttitudeExecutor &_attitude;
    MapTransform &_map;
    YawStabilizer &_yaw;
    DogClientBase *&_dog;
    bool &_controlAcquired;
    bool &_velCtrlActive;
    ReportService &_report;
    // 速度直控最近一次下发量：控制节拍按 20Hz 续发，替代依赖服务器重发
    double &_velFwd;
    double &_velTurn;
    double &_velShift;
    CommandHost &_host;
    // 姿态快照：mapreset / mapline 用的局部坐标必须与导航同一份
    // （原始 odom.position 会在行走中整体跳 0.6~3m，会把参考点与尺度算歪）
    MotionState &_st;

    // 速度直控被拒的日志/反馈节流：服务器常以 10~50Hz 连发 VelocityCmd，
    // 链路掉线时每帧都会打一行"狗未就绪"，把现场终端刷满屏。
    qint64 _lastVelRejectMs = 0;
    int _velRejectCount = 0;
};

#endif  // COMMAND_ROUTER_H
