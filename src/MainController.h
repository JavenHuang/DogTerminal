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
    // 差分位置已经是地图坐标（正东 0°、顺时针、x 东正、y 南正），与本地图同向同号，
    // 唯一未知量是"差分原点 vs mapreset 原点"的平移，以及 mapreset 时狗朝向
    // 不是正东造成的整帧旋转。于是：
    //   平移 → 记一次基准（差分位置 + 当时地图位姿），之后用它做位移差，
    //          基准不再逐站前移（前移会把每站的量化误差累积成随机游走）；
    //   旋转 → 用两段 ≥1.2m 的"差分位移方向 vs 地图位移方向"之差估计，
    //          估计成立时直接 shiftRefYaw 把整个地图系转正，一次性闭环。
    bool   _rkBaseValid = false;
    double _rkBaseRtkX = 0.0;
    double _rkBaseRtkY = 0.0;
    double _rkBaseMapX = 0.0;
    double _rkBaseMapY = 0.0;
    double _rkRotRad = 0.0;     // 差分系 → 地图系 的转角估计（尚未闭环时用于换算）
    int    _rkRotSamples = 0;   // 已用于估计转角的段数
    int    _rkRotApplied = 0;   // 已把转角闭环进 refYaw 的次数（每次参考最多 2 次）
    int    _rkAnchorN = 0;      // 本参考下已消费的锚点次数（日志用）
    int    _anchorSeq = 0;      // 每消费一个静置锚点 +1（到点复核用它判"新锚点到了"）
    // 消费一个静置锚点：更新基准、按需修正 _dr 位姿
    void applyRtkAnchor(double ax, double ay, double mapX, double mapY, int frames, double spread);
    // 方向是否已闭环（差分方向校准未完成时 false，供 CommandRouter 门控）
    bool mapDirectionAligned() const override;

    // ---- 到点复核 + 修正段（[Nav] ArriveVerify）----
    // 到点不立刻上报：先等一个静置锚点把位姿钉准，再用"地图目标 vs 锚后位姿"
    // 的残差判断要不要补一小段。差分单点只有 ±0.2~0.3m，只靠到达半径判定，
    // 实际落点经常差 0.4m 以上。
    bool   _avHold = false;              // 正在等锚点：结果暂不发布
    quint32 _avTaskId = 0;
    DogProtocol::MoveResult _avResult = DogProtocol::MR_ARRIVED;
    double _avRemain = 0.0;
    QString _avMsg;
    int    _avFixLeft = 0;               // 本次任务还剩几段修正
    int    _avAnchorSeq = -1;            // 到点时刻的锚点序号
    qint64 _avDeadlineMs = 0;
    // 每拍调用：到点且锚点已到（或超时）时算残差、决定补段或发布
    void maybeFinishArriveVerify(qint64 nowMs);
    void publishHeldArrive();

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
