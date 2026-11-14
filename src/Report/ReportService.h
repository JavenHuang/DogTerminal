#ifndef REPORT_SERVICE_H
#define REPORT_SERVICE_H

#include <QtGlobal>
#include <QString>

#include "Protocol/DogProtocol.h"   // DogState / TerminalStatus / MoveResult

struct MotionState;

class TerminalConfig;
class MqLink;
class MapTransform;
class Navigator;
class YawStabilizer;
class RtkPoseTracker;
class DogClientBase;

// ============================================================
// 上行上报服务
//
// 职责（只有这一件事）：把终端当前状态打包成协议消息，按三条路由键发出去。
//   state.<id>      位置/朝向/电量/执行状态（1 Hz，StateReport）
//   heartbeat.<id>  终端状态机心跳（1 Hz）
//   feedback.<id>   任务反馈（按事件，不周期发）
//
// 明确不负责：控制台诊断打印（见 Report/StatePrinter）、导航决策、命令解析。
// 依赖以引用/指针引用注入，本类不持有也不销毁这些对象；
// 指针用"引用指针"是为了能在连接建立/断开后看到最新的 _dog/_mq，而不是抓着构造时的空指针。
// ============================================================
class ReportService
{
public:
    ReportService(const TerminalConfig &cfg, MqLink *&mq, MapTransform &map, YawStabilizer &yaw,
                  RtkPoseTracker &rtk, DogProtocol::TerminalStatus &status, DogClientBase *&dog,
                  Navigator &navigator, bool &controlAcquired, bool &velCtrlActive,
                  MotionState &state);
    ~ReportService() = default;

    ReportService(const ReportService &) = delete;
    ReportService &operator=(const ReportService &) = delete;

    // 状态机推进：连接/控制权/执行中 → 四态，变化时打一条日志
    void update();

    void publishState();
    void publishHeartbeat();
    void publishFeedback(quint32 taskId, DogProtocol::MoveResult result, const QString &message);

    // 任务结束：报文比普通反馈多一个剩余距离，且先刷新一帧位置再发
    void publishTaskResult(quint32 taskId, DogProtocol::MoveResult result,
                           double remainingDist, const QString &message);

    // 供任务反馈与调试取当前位置；纯计算，无副作用
    DogProtocol::DogState buildDogState() const;

private:
    const TerminalConfig &_cfg;
    MqLink *&_mq;
    MapTransform &_map;
    YawStabilizer &_yaw;
    RtkPoseTracker &_rtk;
    DogProtocol::TerminalStatus &_status;
    DogClientBase *&_dog;
    Navigator &_navigator;
    bool &_controlAcquired;
    bool &_velCtrlActive;
    // 姿态快照：位置取它（已含抗跳变死推算与静置锚定修正），
    // 不直接读 odom.position，否则上报的地图位置和导航用的位置会是两套
    MotionState &_st;
};

#endif  // REPORT_SERVICE_H
