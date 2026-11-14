#ifndef MAIN_CONTROLLER_H
#define MAIN_CONTROLLER_H

#include <QObject>
#include <QTimer>
#include "Config/TerminalConfig.h"
#include "Protocol/DogProtocol.h"
#include "ServerLink/MqLink.h"
#include "DogLink/DogClientBase.h"
#include "Control/Navigator.h"
#include "Control/AttitudeExecutor.h"
#include "Convert/MapTransform.h"
#include "Convert/DrPose.h"
#include "Convert/RtkFusion.h"
#include "Convert/RtkPoseTracker.h"
#include "Control/MotionState.h"
#include "Control/YawStabilizer.h"
#include "Command/CommandRouter.h"
#include "Control/NavBridge.h"
#include "Report/ReportService.h"
#include "Report/StatePrinter.h"

class DetectionLink;
class CalibPanel;

// ============================================================
// 主控制器：装配北向(RabbitMQ)/南向(UDP)链路与控制逻辑
//
// 数据流：
//   狗(UDP JSON) → UdpDogClient → 原始局部位置/姿态
//                → MapTransform(可选) → StateReport → MQ state.{id}
//   MQ cmd.{id} → 消息分发 → 姿态/导航/旋转/速度/标定
//                → MapTransform(反变换) → Navigator/AttitudeExecutor → UdpDogClient
//
// 地图坐标（[Map] Enabled=true）：
//   +x 正东 +y 正南，朝向角 0=正东、顺时针为正（0~360）；
//   状态上报与任务下发均使用地图坐标。
//   参考以终端启动后首个有效位姿为准（与狗开机时刻无关）：
//   该点=地图(0,0)、该朝向=0°正东；可随时由 MC_RESET 重建。
//
// 职责边界：本类只做装配、链路收发、消息分发、状态上报与生命周期。
//   差分位姿 → Convert/RtkPoseTracker；姿态快照 → Control/MotionState。
//
// 保护逻辑（已确认）：
//   - MQ断连 或 执行任务时服务器消息超时 → 取消任务 + 狗零速停车
//   - 速度直控指令 500ms 无刷新 → 自动零速（死手保护）
//   - 狗连接后自动申请SDK控制权，成功(反馈functionMode==sdk)后进入移动模式
// ============================================================
class MainController : public QObject, public CommandHost
{
    Q_OBJECT
public:
    explicit MainController(QObject *parent = nullptr);
    ~MainController() override;

    bool init(const TerminalConfig &cfg, QString *errMsg);
    void start();
    void stop();

private slots:
    // ---- 北向 ----
    void onMqConnected();
    void onMqDisconnected();
    void onMqMessage(const QString &routingKey, const QByteArray &payload);
    void onMqError(const QString &what);
    void onServerWatchdog();   // 服务器消息超时看门狗（仅任务执行期间生效）
    // 目标检测帧（UDP 重组后）→ 北向转发到 MQ
    void onDetectionFrame(const QByteArray &payload, int detectionCount, const QString &peer);

    // ---- 南向 ----
    void onDogConnectedChanged(bool connected);
    void onDogStateData();
    void onControlAcquireTick();  // 控制权申请重试

    // ---- 导航 ----
    void onNavRemote(double forwardJoy, double turnJoy);
    void onNavStop();
    void onNavFinished(quint32 taskId, DogProtocol::MoveResult result,
                       double remainingDist, const QString &message);

    // ---- 定时器 ----
    void onStateReportTick();  // 状态上报
    void onHeartbeatTick();    // 终端心跳
    void onVelocityTimeout();  // 速度直控死手保护
    void onControlTick();      // 控制节拍（20Hz）：推进导航并续发遥控量，与状态包解耦

private:
    // 装配四段（实现见 MainControllerSetup.cpp），由 init() 顺序调用
    void initLinks();      // MQ/驱动/导航/姿态/稳定器/上报/检测
    void initChannels();   // 标定面板、狗与导航/地图参数、差分融合参数
    void initSignals();    // 信号连接与 5 个定时器
    void initEngines();    // NavBridge 驱动节拍 + CommandRouter 下行路由

    void updateNavigatorFromCache();

    void dispatchMessage(quint16 msgType, const QByteArray &body);

    void cancelActiveTask(DogProtocol::MoveResult result, const QString &reason) override;
    void enterVelocityMode() override;
    void exitVelocityMode(bool sendZero) override;
    void beginControlAcquire() override;

    TerminalConfig _cfg;
    MqLink *_mq;
    DetectionLink *_det;   // 目标检测 UDP 接收（[Detection] Enabled=true 时创建）
    QString _detRoutingKey;
    quint64 _detForwarded;
    quint64 _detMqDropped;
    CalibPanel *_panel;    // 差分/标定状态面板（[Rtk] Enabled=true 时创建）
    QString _panelKey;
    DogClientBase *_dog;   // 具体实现由 [Dog] Driver 决定（udp / sdk）
    Navigator *_navigator;
    AttitudeExecutor *_attitude;
    YawStabilizer *_yaw;     // 航向稳定 + 位移方向校验 + 直线尺度
    RtkPoseTracker *_rtk;    // 差分位姿跟踪（收帧/校验/融合/标定）
    ReportService *_report;  // 上行上报（状态/心跳/反馈）
    StatePrinter *_printer;  // 控制台诊断打印
    CommandRouter *_cmd;     // 下行指令处理（姿态/移动/旋转/标定/速度/停止/档位）
    NavBridge *_nav;         // 驱动节拍：控制权申请 / 直控续发 / 导航推进
    MotionState _st;         // 姿态快照：差分、导航、上报三方共用的唯一来源
    MapTransform _map;     // 局部坐标 ↔ 本地地图坐标
    RtkFusion _fusion;     // 差分锚点 + 里程计增量 的位姿融合与在线标定
    // 抗跳变局部位置：只积分"合理步长"，跳变帧整帧丢弃。
    // 狗端 odom 会整体重置（run9 实测 8 次 0.63~3.12m 单帧跳变），
    // 原始位置直接进导航会让 Pure Pursuit 的方位角瞬变几十度 → 原地画圈。
    DrPose _dr;

    // ---- 差分静置锚定的相对基准 ----
    // 不用差分绝对坐标，而是记"上一次锚点：差分位置 + 当时地图位姿"，
    // 用两点的位移差去修正地图位姿。这样服务器/差分的坐标系原点、
    // 终端 mapreset 的原点各管各的，不需要知道谁对应谁。
    bool   _rkBaseValid = false;
    double _rkBaseRtkX = 0.0;
    double _rkBaseRtkY = 0.0;
    double _rkBaseMapX = 0.0;
    double _rkBaseMapY = 0.0;
    double _rkRotRad = 0.0;     // 差分系 → 地图系 的转角估计（首段前为 0）
    int    _rkRotSamples = 0;   // 已用于估计转角的段数
    // 消费一个静置锚点：更新基准、按需修正 _dr 位姿
    void applyRtkAnchor(double ax, double ay, double mapX, double mapY, int frames, double spread);

    QTimer *_stateTimer;      // 状态上报
    QTimer *_hbTimer;         // 终端心跳
    QTimer *_watchdogTimer;   // 服务器消息看门狗
    QTimer *_velocityTimer;   // 速度直控死手
    QTimer *_acquireTimer;    // 控制权申请重试
    QTimer *_ctrlTimer;       // 控制节拍（20Hz，与状态包到达率无关）

    qint64 _lastServerMsgMs;      // 最近收到服务器消息时刻
    bool _serverHeartbeatSeen;    // 是否收到过服务器下行心跳（决定看门狗是否启用）

    bool _running;
    bool _controlAcquired;    // 已取得SDK控制权
    bool _velCtrlActive;      // 速度直控进行中
    DogProtocol::TerminalStatus _status;

    // 里程计坐标系跳变检测：狗端 odom 重置（遥控接管/模式切换）时平移地图参考保持连续
    void compensateOdomJump(const DogOdom &odom);
    // 狗端局部 yaw 基准整体跳变（断链/控制权重置后 30°+ 台阶）的参考朝向补偿
    void compensateYawReset(const DogOdom &odom, const DogImu &imu);
    // 重建地图参考（尺度从 YawStabilizer 取，参考变更后仍沿用）
    void resetMapReference(double localX, double localY, double localYaw) override;
    // 地图标定结果 → 融合标定（现场"只标方向、不打 4 个停点"的入口）
    void applyMapCalibToFusion() override;
    // 直线标定行程监控（mapreset → mapline 期间，现场看"我到底走了多少米"）
    void beginLineTrace() override;
    void endLineTrace(double targetMapX, double targetMapY) override;
    bool lineTraceDisplacement(double *vLocalX, double *vLocalY) const override;

    // 里程计跳变检测：上一次有效原始位姿（用于识别 odom 坐标系重置）
    bool _lastOdomValid = false;
    double _lastOdomX = 0.0;
    double _lastOdomY = 0.0;
    qint64 _lastOdomMs = 0;
    // 断链恢复后狗端局部 yaw 基准整体跳变（SDK 坐标系重置）的检测与补偿
    double _lastYawRad = 0.0;
    qint64 _lastYawMs = 0;
    bool   _lastYawValid = false;
    qint64 _lastNavDiagMs = 0;    // 导航闭环阻断诊断打印节流

    // 链路体检（10s 一行）：用于区分"狗端不推"与"网络成串丢包"
    qint64 _linkLastRxMs = 0;
    qint64 _linkReportMs = 0;
    int _linkRxCount = 0;
    int _linkHoleCount = 0;
    int _linkMaxGapMs = 0;

    // 直线标定行程监控：mapreset 记起点，之后每秒报"里程计走了多少米"，
    // mapline 落点跟你实测的米数对比，直接给出 k 偏大/偏小。
    bool _traceActive = false;
    double _traceStartX = 0.0;
    double _traceStartY = 0.0;
    double _traceLastX = 0.0;
    double _traceLastY = 0.0;
    double _tracePath = 0.0;      // 累计路径长度（含左右摆动，用来判断"走不直"）
    // 跳变免疫的累计位移：每帧只累加"合理步长"，被判定为跳变的帧整帧丢弃。
    // 狗端 odom 会在行走中整体跳 0.6~3m（run9 实测），若直接用"终点-起点"，
    // 标定 k 会被跳变污染（实测算出过 1.445）。
    double _traceAccX = 0.0;
    double _traceAccY = 0.0;
    qint64 _traceLastMs = 0;
    int    _traceJumpN = 0;        // 标定行程中被丢弃的跳变帧数
    double _traceJumpM = 0.0;      // 被丢弃的跳变累计米数
    qint64 _traceStartMs = 0;
    qint64 _tracePrintMs = 0;
    int _tracePkts = 0;
    // 每次有效状态包调用：累计路径 + 1s 一行行程打印
    void traceTick(const DogOdom &odom);

};

#endif  // MAIN_CONTROLLER_H
