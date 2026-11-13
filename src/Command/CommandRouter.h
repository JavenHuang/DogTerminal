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
                  double &velFwd, double &velTurn, double &velShift, CommandHost &host);

    void handleAttitudeCmd(const QByteArray &body);
    void handleMoveCmd(const QByteArray &body);
    void handleRotateCmd(const QByteArray &body, bool absolute);
    void handleMapCalibCmd(const QByteArray &body);
    void handleVelocityCmd(const QByteArray &body);
    void handleStopCmd(const QByteArray &body);
    void handleParamCmd(const QByteArray &body);

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
};

#endif  // COMMAND_ROUTER_H
