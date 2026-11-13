#ifndef TERMINAL_CONFIG_H
#define TERMINAL_CONFIG_H

#include <QString>
#include "Control/Navigator.h"
#include "Convert/MapTransform.h"

// ============================================================
// 终端配置：从 INI 文件加载（QSettings）
// 每个终端一份配置（terminalId 不同、狗 IP 不同），
// 支持多终端部署：一份程序 + 多份 INI。
// ============================================================
struct TerminalConfig
{
    // ---- 终端标识 ----
    quint32 terminalId;          // 终端ID（唯一，路由键用）

    // ---- RabbitMQ 北向 ----
    QString mqHost;              // 服务器地址
    quint16 mqPort;              // 默认 5672
    QString mqVhost;
    QString mqUser;
    QString mqPassword;
    QString mqExchange;          // Topic 交换机名
    int mqRetryMs;               // 断线重试周期

    // ---- 状态上报 ----
    int stateRateHz;             // 状态上报频率（默认10Hz）
    int heartbeatSec;            // 终端心跳周期（秒，默认1）
    int serverLinkTimeoutSec;    // 服务器失联判定（秒，默认5）
    int velocityTimeoutMs;       // 速度直控死手保护（ms，默认500）

    // ---- 机器狗南向 ----
    QString dogDriver;           // udp（默认）/ sdk（仅 Linux 构建）/ matrix（MATRiX 仿真）
    QString dogIp;
    quint16 dogSendPort;         // 控制口：udp/sdk=8081，matrix=7447
    quint16 dogRecvPort;         // 状态口：udp/sdk=8080，matrix=Zenoh 监听口(默认7447)
    QString dogRole;
    // 心跳里声明的角色；空串沿用 dogRole（推荐）。
    // 实测结论：设为 remote 时，20 Hz 心跳与 1 Hz 的 sdk 遥控帧互抢控制权，
    // 狗在 sdk/remote 间反复切模式，表现为"走两步就停"+状态包成片丢失。
    QString dogHeartbeatRole;
    // 启动时申请 SDK 控制权后，重试次数（0=只申请一次，不再周期重复）
    int acquireRetryCount;
    int acquireRetryMs;          // 控制权申请重试周期 ms
    int dogHeartbeatMs;          // 默认50
    int dogTimeoutMs;            // 默认5000
    bool autoAcquireControl;     // 连接后自动申请SDK控制权
    int dogStatePrintHz;         // 本地控制台状态打印频率（0=关闭，默认1）

    // ---- MATRiX 仿真专属（Driver=matrix 时生效）----
    QString matrixZenohEndpoint; // Zenoh 端点：tcp/<ip>:7447（默认 tcp/127.0.0.1:7447）
    QString matrixZenohMode;     // Zenoh 模式：client / router
    QString matrixTopicPrefix;   // 话题前缀（默认 rt）
    QString matrixImuTopic;      // IMU 话题（默认 /imu）
    QString matrixOdomTopic;     // 里程计话题（默认 /odom/mujoco_odom）

    // ---- 地图坐标（局部 ↔ 本地地图）----
    MapTransform::Params mapParams;

    // ---- 位姿质量（yaw 稳定化）----
    bool yawStabilize;          // yaw 零偏在线估计 + 跳变抑制
    double staticLockSec;       // 静止多少秒后冻结 yaw（0=不冻结）
    double yawJumpDeg;          // 单帧 yaw 跳变判定阈值（度）
    double poseMaxGapSec;       // 单帧最大有效时间间隔（s），超过视为断线重锚
    bool   poseDispAssist;      // 位移方向辅助航向校验/重建
    double poseDispMinDist;     // 直线位移窗口长度（m）
    double poseDispMinSpeed;    // 进入窗口的最小速度（m/s）
    double poseDispCorrectDeg;  // 轻度修正门限（度）
    double poseDispReanchorDeg; // 直接重建门限（度）

    // ---- 导航参数 ----
    Navigator::Params navParams;

    // ---- 控制节拍（与状态包解耦）----
    // 旧实现：导航推进与遥控下发都只挂在"状态包到达"这一事件上，
    // 于是控制刷新率 = 状态包到达率（名义 1 Hz，空洞时 3~4 s 一拍），
    // 狗每拍收到一帧摇杆量后随即失去刷新 → 走走停停。
    // 新增独立控制节拍，按固定周期用最近一次状态推进导航并续发遥控量。
    int navCtrlTickMs;           // 控制节拍周期 ms（默认 50 = 20 Hz）
    int navCtrlStaleStopMs;      // 状态包中断超过该值(ms)则暂停导航盲发（默认 3000，0=不限制）

    // ---- 差分(RTK)绝对位姿（服务器约 200ms 下发一次）----
    // 第一阶段只做"接收 → 换算 → 比对 → 打印"，不参与导航与上报，
    // 因此 Enabled=true 时也不改变现有位姿来源；后续阶段再由 Rtk/Mode 决定是否接管。
    bool   rtkEnabled;           // 是否接收差分位姿并做观测（默认 false）
    QString rtkRoutingKey;       // 差分定位 MQ 路由键（默认 la）
    qint32 rtkDiffId;            // 只接受该差分 ID 的帧，0=不校验（多狗共用 la 时必填）
    int    rtkStaleMs;           // 超过该值无新帧即判为断流 ms（默认 600）
    double rtkMaxJumpM;          // 相邻帧位置跳变上限 m（默认 1.5）
    double rtkMaxJumpDeg;        // 相邻帧航向跳变上限 度（默认 30）
    double rtkAntennaX;          // 天线相位中心相对狗体中心的前向偏置 m（前为正）
    double rtkAntennaY;          // 天线相位中心相对狗体中心的左向偏置 m（左为正）
    double rtkYawOffsetDeg;      // 天线基线方向与狗体纵轴的固定夹角 度（装反/装偏用）
    // ---- 融合与在线标定（第二阶段） ----
    bool   rtkUseForPose;        // 融合位姿是否用于导航与上报（默认 true）
    bool   rtkAutoCalib;         // 是否在线自动解尺度与转角（默认 true）
    int    rtkCalibMinSamples;   // 解算所需最少"静止锚点"个数（默认 4，需分散在不同位置）
    double rtkCalibMinPathM;     // 解算所需最少里程 m（默认 0.8）
    double rtkCalibMaxRmsM;      // 解算允许的最大残差 RMS m（默认 0.12）
    int    rtkCalibStaticHoldMs; // 持续静止多久才算一个静止锚点 ms（默认 1500）
    double rtkCalibPointGapM;    // 两个静止锚点最小间距 m（默认 0.35，原地重复停不算新点）
    double rtkOffsetTauSec;      // 兼容旧配置；慢速微调改由 OffsetSlowTauSec 控制
    int    rtkHoldMs;            // 差分断流后保持偏移外推的时长 ms（默认 2000）
    bool   rtkYawFromRtk;        // 是否用差分航向做绝对朝向（服务端航向恒 0 时置 false）
    // 差分 5Hz / 里程计 20Hz：差分帧滞后 0.2~0.5s，只有"确实没在动"才把它当锚点
    double rtkOffStaticSpeed;    // 低于该速度视为静止，可快速重锚 m/s（默认 0.03）
    double rtkOffSlowSpeed;      // 低于该速度允许慢速微调 m/s（默认 0.2）
    double rtkOffSlowYawRateDeg; // 角速度低于该值才允许微调航向 度/s（默认 5）
    double rtkOffStaticTauSec;   // 静止时偏移低通时间常数 s（默认 0.3）
    double rtkOffSlowTauSec;     // 慢速时偏移低通时间常数 s（默认 2.0）
    // ---- 差分航向零位与停点标定判据 ----
    bool   rtkAutoZero;          // 建参考后自动取差分零位 b（默认 true）
    int    rtkZeroHoldMs;        // 静止持续多久开始取零位 ms（默认 800）
    int    rtkZeroMinFrames;     // 取多少帧做圆周均值（默认 3）
    double rtkCalibKMin;         // 停点标定 k 下限（默认 0.95）
    double rtkCalibKMax;         // 停点标定 k 上限（默认 1.05）
    double rtkCalibAlphaMaxDeg;  // 停点标定 |α| 上限 度（默认 5）
    // ---- 直线反算零位 ----
    bool   rtkZeroFromLine;      // 用停点间差分位移方向反算 b（默认 true）
    double rtkZeroLineSegM;      // 单段最小位移 m（默认 0.6）
    double rtkZeroLineMinM;      // 累计直线长度门槛 m（默认 2.5）
    double rtkZeroLineSpreadDeg; // 各段 b 观测允许的最大散布 度（默认 2.0）

    // ---- 目标检测（YOLO26 检测结果 UDP → MQ 转发）----
    // 188 的检测服务把每帧推理结果（完整 JSON 或 base64 分片）发到本机端口，
    // 终端重组后原样转发到 MQ 路由键 detection.<terminalId>，服务端按同一份
    // UDP 协议文档解析。不改写字段，也不改变现有位姿/上报链路。
    bool    detEnabled;          // 是否开启 UDP 接收（默认 false，缺段即关闭）
    QString detBind;             // 监听地址，0.0.0.0 = 所有网卡
    quint16 detPort;             // 监听端口（默认 5006，与 188 网页设置一致）
    QString detRoutingKey;       // 转发路由键；空 = detection.<terminalId>
    int     detMaxHz;            // 转发限流 Hz（0=不限，默认 10，与推理上限一致）
    bool    detSkipEmpty;        // 无目标帧是否跳过（默认 true，省带宽）
    bool    detVerbose;          // 每帧打印一行（默认 false，只打 5s 统计）

    TerminalConfig();

    // 从 INI 文件加载，返回 false 表示文件缺失或关键字段缺失
    bool load(const QString &iniPath);
    // 加载后的检查（terminalId>0、dogIp 非空等）
    bool validate(QString *errMsg) const;
};

#endif  // TERMINAL_CONFIG_H
