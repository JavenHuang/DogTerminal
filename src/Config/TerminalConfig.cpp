#include "TerminalConfig.h"

#include <QSettings>
#include <QFileInfo>
#include <QHostAddress>

TerminalConfig::TerminalConfig()
    : terminalId(0)
    , mqPort(5672)
    , mqRetryMs(5000)
    , stateRateHz(10)
    , heartbeatSec(1)
    , serverLinkTimeoutSec(5)
    , velocityTimeoutMs(500)
    , dogSendPort(8081)
    , dogRecvPort(8080)
    , acquireRetryCount(0)
    , acquireRetryMs(500)
    , dogHeartbeatMs(50)
    , dogTimeoutMs(5000)
    , autoAcquireControl(true)
    , dogStatePrintHz(1)
    , matrixZenohEndpoint(QStringLiteral("tcp/127.0.0.1:7447"))
    , matrixZenohMode(QStringLiteral("client"))
    , matrixTopicPrefix(QStringLiteral("rt"))
    , matrixImuTopic(QStringLiteral("/imu"))
    , matrixOdomTopic(QStringLiteral("/odom"))
    , navCtrlTickMs(50)
    , navCtrlStaleStopMs(3000)
    , rtkEnabled(false)
    , rtkRoutingKey(QStringLiteral("la"))
    , rtkDiffId(0)
    , rtkStaleMs(600)
    , rtkMaxJumpM(1.5)
    , rtkMaxJumpDeg(30.0)
    , rtkAntennaX(0.0)
    , rtkAntennaY(0.0)
    , rtkYawOffsetDeg(0.0)
    , rtkUseForPose(true)
    , rtkAutoCalib(true)
    , rtkCalibMinSamples(4)
    , rtkCalibMinPathM(0.8)
    , rtkCalibMaxRmsM(0.12)
    , rtkCalibStaticHoldMs(1500)
    , rtkCalibPointGapM(0.35)
    , rtkOffsetTauSec(0.5)
    , rtkHoldMs(2000)
    , rtkYawFromRtk(true)
    , rtkOffStaticSpeed(0.03)
    , rtkOffSlowSpeed(0.2)
    , rtkOffSlowYawRateDeg(5.0)
    , rtkOffStaticTauSec(0.3)
    , rtkOffSlowTauSec(2.0)
    , rtkAutoZero(true)
    , rtkZeroHoldMs(800)
    , rtkZeroMinFrames(3)
    , rtkCalibKMin(0.95)
    , rtkCalibKMax(1.05)
    , rtkCalibAlphaMaxDeg(5.0)
    , rtkZeroFromLine(true)
    , rtkZeroLineSegM(0.6)
    , rtkZeroLineMinM(2.5)
    , rtkZeroLineSpreadDeg(2.0)
    , detEnabled(false)
    , detBind(QStringLiteral("0.0.0.0"))
    , detPort(5006)
    , detMaxHz(10)
    , detSkipEmpty(true)
    , detVerbose(false)
{
}

bool TerminalConfig::load(const QString &iniPath)
{
    if (!QFileInfo::exists(iniPath))
        return false;

    QSettings s(iniPath, QSettings::IniFormat);

    terminalId = static_cast<quint32>(s.value("Terminal/Id", 0).toUInt());

    mqHost     = s.value("RabbitMq/Host").toString();
    mqPort     = static_cast<quint16>(s.value("RabbitMq/Port", 5672).toUInt());
    mqVhost    = s.value("RabbitMq/Vhost", "/").toString();
    mqUser     = s.value("RabbitMq/User", "guest").toString();
    mqPassword = s.value("RabbitMq/Password", "guest").toString();
    mqExchange = s.value("RabbitMq/Exchange", "kdiss.dog").toString();
    mqRetryMs  = s.value("RabbitMq/RetryMs", 5000).toInt();

    stateRateHz          = s.value("Report/StateRateHz", 10).toInt();
    heartbeatSec         = s.value("Report/HeartbeatSec", 1).toInt();
    serverLinkTimeoutSec = s.value("Report/ServerLinkTimeoutSec", 5).toInt();
    velocityTimeoutMs    = s.value("Report/VelocityTimeoutMs", 500).toInt();

    // 南向驱动选择：udp=自实现 UDP+JSON；sdk=官方 libzsibot；matrix=MATRiX 仿真（Zenoh）
    dogDriver          = s.value("Dog/Driver", "udp").toString().trimmed().toLower();
    dogIp              = s.value("Dog/Ip").toString();
    // matrix 驱动默认端口与 udp/sdk 不同，这里按 Driver 给合理默认
    if (dogDriver == QStringLiteral("matrix")) {
        dogSendPort    = static_cast<quint16>(s.value("Dog/SendPort", 7447).toUInt());
        dogRecvPort    = static_cast<quint16>(s.value("Dog/RecvPort", 7447).toUInt());
    } else {
        dogSendPort    = static_cast<quint16>(s.value("Dog/SendPort", 8081).toUInt());
        dogRecvPort    = static_cast<quint16>(s.value("Dog/RecvPort", 8080).toUInt());
    }
    dogRole            = s.value("Dog/Role", "sdk").toString();
    // 心跳角色：留空=沿用 Role；设为 remote 后心跳不再周期声明 sdk 身份
    dogHeartbeatRole   = s.value("Dog/HeartbeatRole", "").toString().trimmed();
    dogHeartbeatMs     = s.value("Dog/HeartbeatMs", 50).toInt();
    dogTimeoutMs       = s.value("Dog/TimeoutMs", 5000).toInt();
    autoAcquireControl = s.value("Dog/AutoAcquireControl", true).toBool();
    // 启动时申请一次控制权；RetryCount=0 表示之后不再重复申请
    acquireRetryCount  = qBound(0, s.value("Dog/AcquireRetryCount", 0).toInt(), 1000);
    acquireRetryMs     = qBound(50, s.value("Dog/AcquireRetryMs", 500).toInt(), 60000);
    // 真狗联调时用于在终端控制台观察姿态与方向，默认1Hz；设为0可关闭
    dogStatePrintHz    = qBound(0, s.value("Dog/PrintStateHz", 1).toInt(), 20);

    // MATRiX 仿真专属
    matrixZenohEndpoint = s.value("Matrix/ZenohEndpoint", "tcp/127.0.0.1:7447").toString();
    matrixZenohMode     = s.value("Matrix/ZenohMode", "client").toString();
    matrixTopicPrefix   = s.value("Matrix/TopicPrefix", "rt").toString();
    matrixImuTopic      = s.value("Matrix/ImuTopic", "/imu").toString();
    matrixOdomTopic     = s.value("Matrix/OdomTopic", "/odom").toString();

    // ---- 地图坐标：局部坐标 ↔ 本地地图坐标 ----
    // 地图约定：+x 正东 +y 正南，朝向角 0=正东、顺时针为正
    mapParams.enabled       = s.value("Map/Enabled", false).toBool();
    mapParams.originX       = s.value("Map/OriginX", 0.0).toDouble();
    mapParams.originY       = s.value("Map/OriginY", 0.0).toDouble();
    mapParams.autoReference = s.value("Map/AutoReference", true).toBool();
    mapParams.jumpCompensate = s.value("Map/JumpCompensate", false).toBool();
    mapParams.scaleMin      = s.value("Map/ScaleMin", 0.7).toDouble();
    mapParams.scaleMax      = s.value("Map/ScaleMax", 1.3).toDouble();
    mapParams.maxHeadingErrRad = s.value("Map/MaxHeadingErrDeg", 15.0).toDouble()
                                    * 3.14159265358979323846 / 180.0;
    mapParams.coarseHeadingErrRad = s.value("Map/CoarseHeadingErrDeg", 40.0).toDouble()
                                        * 3.14159265358979323846 / 180.0;
    if (mapParams.coarseHeadingErrRad < mapParams.maxHeadingErrRad)
        mapParams.coarseHeadingErrRad = mapParams.maxHeadingErrRad;
    mapParams.minLineDist   = s.value("Map/MinLineDist", 1.0).toDouble();
    if (mapParams.scaleMax < mapParams.scaleMin)
        mapParams.scaleMax = mapParams.scaleMin;
    if (mapParams.scaleMin <= 0.0)
        mapParams.scaleMin = 0.1;
    if (mapParams.minLineDist < 0.1)
        mapParams.minLineDist = 0.1;

    // 位姿质量：yaw 稳定化（零偏在线估计 + 静止锁 + 跳变抑制）
    yawStabilize  = s.value("Pose/YawStabilize", true).toBool();
    staticLockSec = s.value("Pose/StaticLockSec", 2.0).toDouble();
    yawJumpDeg    = s.value("Pose/YawJumpDeg", 15.0).toDouble();
    poseMaxGapSec = s.value("Pose/MaxFrameGapSec", 3.0).toDouble();
    poseDispAssist = s.value("Pose/DispHdgAssist", true).toBool();
    poseDispMinDist = s.value("Pose/DispHdgMinDist", 0.8).toDouble();
    poseDispMinSpeed = s.value("Pose/DispHdgMinSpeed", 0.12).toDouble();
    poseDispCorrectDeg = s.value("Pose/DispHdgCorrectDeg", 15.0).toDouble();
    poseDispReanchorDeg = s.value("Pose/DispHdgReanchorDeg", 45.0).toDouble();
    if (staticLockSec < 0.0)
        staticLockSec = 0.0;
    if (yawJumpDeg < 1.0)
        yawJumpDeg = 1.0;
    if (poseMaxGapSec < 0.2)
        poseMaxGapSec = 0.2;
    if (poseDispMinDist < 0.2)
        poseDispMinDist = 0.2;
    if (poseDispMinSpeed < 0.02)
        poseDispMinSpeed = 0.02;
    if (poseDispCorrectDeg < 2.0)
        poseDispCorrectDeg = 2.0;
    if (poseDispReanchorDeg <= poseDispCorrectDeg)
        poseDispReanchorDeg = poseDispCorrectDeg + 5.0;

    // 速度策略：0=Cruise 匀速硬切，1=Profile 三段式（加速度+收油衰减）
    {
        const int mode = s.value("Nav/SpeedMode", 0).toInt();
        navParams.speedMode = (mode == 1)
                                  ? Navigator::SpeedMode_Profile
                                  : Navigator::SpeedMode_Cruise;
    }
    navParams.headingToleranceRad  = s.value("Nav/HeadingToleranceDeg", 20.0).toDouble() * 3.14159265358979323846 / 180.0;
    navParams.reAlignThresholdRad  = s.value("Nav/ReAlignThresholdDeg", 30.0).toDouble() * 3.14159265358979323846 / 180.0;
    navParams.kpAlign              = s.value("Nav/KpAlign", 1.5).toDouble();
    // 转向D项：用 yawRate 阻尼抑制真狗停转惯性过冲
    navParams.kdAlign              = s.value("Nav/KdAlign", 0.4).toDouble();
    // 接近对准容差时的转向减速带
    navParams.alignSlowBandRad     = s.value("Nav/AlignSlowBandDeg", 25.0).toDouble()
                                        * 3.14159265358979323846 / 180.0;
    navParams.alignSlowMaxTurnRate = s.value("Nav/AlignSlowMaxTurnRate", 0.4).toDouble();
    navParams.kpHoldYaw            = s.value("Nav/KpHoldYaw", 1.0).toDouble();
    navParams.kpDist               = s.value("Nav/KpDist", 0.6).toDouble();
    navParams.kpVel                = s.value("Nav/KpVel", 0.2).toDouble();
    navParams.maxTurnRate          = s.value("Nav/MaxTurnRate", 1.2).toDouble();
    navParams.maxLinearSpeed       = s.value("Nav/MaxLinearSpeed", 0.8).toDouble();
    navParams.joyLinearScale       = s.value("Nav/JoyLinearScale", 1.0).toDouble();
    navParams.joyTurnScale         = s.value("Nav/JoyTurnScale", 1.5).toDouble();
    // 默认禁止倒退：目标在侧后方时先转向再前进（不允许倒车接近）
    navParams.allowBackward        = s.value("Nav/AllowBackward", false).toBool();
    // MATRiX 下 walk 需 forward≠0 才执行转向，对准阶段的最小前进速度
    navParams.alignCreepSpeed      = s.value("Nav/AlignCreepSpeed", 0.25).toDouble();
    // MATRiX/UE 摇杆轴死区补偿：|axis| < 0.2 的输入会被仿真器丢弃
    navParams.minAxisOut           = s.value("Nav/MinAxisOut", 0.22).toDouble();
    // 收油段 + 抗振荡（两种速度模式都强制启用收油衰减，防快速巡航过冲）
    navParams.stopArriveRadiusMul  = s.value("Nav/StopArriveRadiusMul", 6.0).toDouble();
    navParams.minCreepSpeed        = s.value("Nav/MinCreepSpeed", 0.25).toDouble();
    navParams.stopBodyVxDeadzone   = s.value("Nav/StopBodyVxDeadzone", 0.05).toDouble();
    navParams.arriveRadiusDefault  = s.value("Nav/ArriveRadiusDefault", 1.5).toDouble();
    navParams.softArriveRadiusMul  = s.value("Nav/SoftArriveRadiusMul", 2.0).toDouble();
    navParams.holdYawOutDeadzoneRad = s.value("Nav/HoldYawOutDeadzoneDeg", 5.0).toDouble()
                                        * 3.14159265358979323846 / 180.0;
    // Profile 模式专用（Cruise 模式忽略这两个参数）
    navParams.maxAccel             = s.value("Nav/MaxAccel", 1.0).toDouble();
    navParams.maxDecel             = s.value("Nav/MaxDecel", 2.0).toDouble();

    // ---- 转向律：1=PurePursuit（默认，边走边转，避免腿足原地打滑）0=AlignThenGo ----
    navParams.steeringMode = (s.value("Nav/SteeringMode", 1).toInt() == 1)
                                 ? Navigator::Steering_PurePursuit
                                 : Navigator::Steering_AlignThenGo;
    navParams.ppLookaheadMin = s.value("Nav/PpLookaheadMin", 0.6).toDouble();
    navParams.ppLookaheadMax = s.value("Nav/PpLookaheadMax", 1.5).toDouble();
    navParams.ppLookaheadK   = s.value("Nav/PpLookaheadK", 0.5).toDouble();
    // 前视距离必须 > 到达半径，否则接近目标时曲率计算病态（原地打摆）
    if (navParams.ppLookaheadMin < 0.2) navParams.ppLookaheadMin = 0.2;
    if (navParams.ppLookaheadMax < navParams.ppLookaheadMin)
        navParams.ppLookaheadMax = navParams.ppLookaheadMin;

    // ---- 速度闭环：摇杆归一化值 → 实际 m/s 的自适应增益 ----
    navParams.joyGainAdaptEnable = s.value("Nav/JoyGainAdaptEnable", true).toBool();
    navParams.joyGainMin         = s.value("Nav/JoyGainMin", 0.3).toDouble();
    navParams.joyGainMax         = s.value("Nav/JoyGainMax", 3.0).toDouble();
    navParams.joyGainTau         = s.value("Nav/JoyGainTau", 0.05).toDouble();
    if (navParams.joyGainMax < navParams.joyGainMin)
        navParams.joyGainMax = navParams.joyGainMin;

    // 诊断日志：5Hz 的 [NvDbg]/[NvRot] 行，默认关闭（排查航向时置 true）
    navParams.debugLog = s.value("Nav/DebugLog", false).toBool();

    // ---- 控制节拍：与状态包到达率解耦（默认 20 Hz）----
    // 下限 10ms(100Hz) 防止误配拖垮 UDP 下发；上限 200ms 仍远快于狗端死手
    navCtrlTickMs      = qBound(10, s.value("Nav/CtrlTickMs", 50).toInt(), 200);
    // 状态包中断超过该值即暂停盲发（0=不限制，始终续发上一拍遥控量）
    navCtrlStaleStopMs = qBound(0, s.value("Nav/CtrlStaleStopMs", 3000).toInt(), 30000);

    // ---- 差分(RTK)绝对位姿：第一阶段只观测，不接管控制 ----
    // 服务器约 200ms 下发一帧本地 ENU 位置与真航向，终端只做换算与比对。
    rtkEnabled      = s.value("Rtk/Enabled", false).toBool();
    rtkRoutingKey   = s.value("Rtk/RoutingKey", QStringLiteral("la")).toString().trimmed();
    if (rtkRoutingKey.isEmpty())
        rtkRoutingKey = QStringLiteral("la");
    rtkDiffId       = static_cast<qint32>(s.value("Rtk/DiffId", 0).toInt());
    // 5 帧(1s)内无新帧即判为断流；下限 200ms，避免误配置成"每帧都判断"
    rtkStaleMs      = qBound(200, s.value("Rtk/StaleMs", 600).toInt(), 5000);
    rtkMaxJumpM     = qBound(0.2, s.value("Rtk/MaxJumpM", 1.5).toDouble(), 20.0);
    rtkMaxJumpDeg   = qBound(5.0, s.value("Rtk/MaxJumpDeg", 30.0).toDouble(), 179.0);
    rtkAntennaX     = qBound(-2.0, s.value("Rtk/AntennaOffsetX", 0.0).toDouble(), 2.0);
    rtkAntennaY     = qBound(-2.0, s.value("Rtk/AntennaOffsetY", 0.0).toDouble(), 2.0);
    rtkYawOffsetDeg = qBound(-180.0, s.value("Rtk/YawOffsetDeg", 0.0).toDouble(), 180.0);

    // ---- 融合位姿源与在线标定（第二阶段）----
    rtkUseForPose    = s.value("Rtk/UseForPose", true).toBool();
    rtkAutoCalib     = s.value("Rtk/AutoCalib", true).toBool();
    // 标定只用"静止锚点"：狗停稳一次算一个点，需要 4 个分散在不同位置的点
    // 才解 k 与 α（原地重复停不算新点）。旧键 CalibMinSamples 兼容读取。
    rtkCalibMinSamples = qBound(3, s.value("Rtk/CalibMinPoints",
                                           s.value("Rtk/CalibMinSamples", 4)).toInt(), 200);
    rtkCalibMinPathM = qBound(0.2, s.value("Rtk/CalibMinPathM", 0.8).toDouble(), 50.0);
    rtkCalibMaxRmsM  = qBound(0.02, s.value("Rtk/CalibMaxRmsM", 0.12).toDouble(), 1.0);
    rtkCalibStaticHoldMs = qBound(300, s.value("Rtk/CalibStaticHoldMs", 1500).toInt(), 20000);
    rtkCalibPointGapM = qBound(0.05, s.value("Rtk/CalibPointGapM", 0.35).toDouble(), 5.0);
    rtkOffsetTauSec  = qBound(0.05, s.value("Rtk/OffsetTauSec", 0.5).toDouble(), 5.0);
    rtkHoldMs        = qBound(200, s.value("Rtk/HoldMs", 2000).toInt(), 30000);
    // 服务端只给位置、航向恒为 0 时置 false：朝向只走里程计 + 位置标定转角
    rtkYawFromRtk     = s.value("Rtk/YawFromRtk", true).toBool();
    // 静止门控：差分 5Hz、滞后 0.2~0.5s，运动中一律不信（冻结 offset）
    rtkOffStaticSpeed    = qBound(0.0, s.value("Rtk/OffStaticSpeed", 0.03).toDouble(), 1.0);
    rtkOffSlowSpeed      = qBound(0.0, s.value("Rtk/OffSlowSpeed", 0.2).toDouble(), 2.0);
    rtkOffSlowYawRateDeg = qBound(0.5, s.value("Rtk/OffSlowYawRateDeg", 5.0).toDouble(), 180.0);
    rtkOffStaticTauSec   = qBound(0.05, s.value("Rtk/OffStaticTauSec", 0.3).toDouble(), 5.0);
    rtkOffSlowTauSec     = qBound(0.2, s.value("Rtk/OffSlowTauSec", 2.0).toDouble(), 20.0);

    // 差分航向零位 b：会话量，每次"建参考"重取；只存内存，不进 ini
    rtkAutoZero        = s.value("Rtk/ZeroAuto", true).toBool();
    rtkZeroHoldMs      = qBound(100, s.value("Rtk/ZeroHoldMs", 800).toInt(), 10000);
    rtkZeroMinFrames   = qBound(1, s.value("Rtk/ZeroMinFrames", 3).toInt(), 50);
    // 停点标定的合理性判据：k 应≈1、α 应≈0，越界不采用（提示回查 mapcalib）
    rtkCalibKMin       = qBound(0.50, s.value("Rtk/CalibKMin", 0.95).toDouble(), 2.0);
    rtkCalibKMax       = qBound(rtkCalibKMin + 1e-3,
                                s.value("Rtk/CalibKMax", 1.05).toDouble(), 2.0);
    rtkCalibAlphaMaxDeg = qBound(0.5, s.value("Rtk/CalibAlphaMaxDeg", 5.0).toDouble(), 180.0);
    // 直线反算零位：建参考先取读数当初始值，随后用"停点间差分位移方向"把 b 归到地图东
    rtkZeroFromLine    = s.value("Rtk/ZeroFromLine", true).toBool();
    rtkZeroLineSegM    = qBound(0.2, s.value("Rtk/ZeroLineSegM", 0.6).toDouble(), 5.0);
    rtkZeroLineMinM    = qBound(1.0, s.value("Rtk/ZeroLineMinM", 2.5).toDouble(), 20.0);
    rtkZeroLineSpreadDeg = qBound(0.2, s.value("Rtk/ZeroLineSpreadDeg", 2.0).toDouble(), 20.0);

    // ---- 目标检测（YOLO26 检测结果 UDP → MQ 转发）----
    // 只做"收包、重组、原样转发"，不参与位姿与控制；缺段时保持关闭。
    detEnabled    = s.value("Detection/Enabled", false).toBool();
    detBind       = s.value("Detection/Bind", QStringLiteral("0.0.0.0")).toString().trimmed();
    if (detBind.isEmpty())
        detBind = QStringLiteral("0.0.0.0");
    detPort       = static_cast<quint16>(qBound(1, s.value("Detection/Port", 5006).toInt(), 65535));
    detRoutingKey = s.value("Detection/RoutingKey", QString()).toString().trimmed();
    // 188 推理上限 10FPS；限流只防"上游改大后把 MQ 打满"
    detMaxHz      = qBound(0, s.value("Detection/MaxHz", 10).toInt(), 50);
    detSkipEmpty  = s.value("Detection/SkipEmpty", true).toBool();
    detVerbose    = s.value("Detection/VerboseLog", false).toBool();

    return true;
}

bool TerminalConfig::validate(QString *errMsg) const
{
    if (terminalId == 0)
    {
        if (errMsg) *errMsg = QStringLiteral("Terminal/Id 必须为大于0的整数");
        return false;
    }
    if (mqHost.trimmed().isEmpty())
    {
        if (errMsg) *errMsg = QStringLiteral("RabbitMq/Host 不能为空");
        return false;
    }
    if (dogIp.trimmed().isEmpty() || !QHostAddress().setAddress(dogIp.trimmed()))
    {
        if (errMsg) *errMsg = QStringLiteral("Dog/Ip 无效: %1").arg(dogIp);
        return false;
    }
    if (stateRateHz <= 0 || stateRateHz > 100)
    {
        if (errMsg) *errMsg = QStringLiteral("Report/StateRateHz 取值应为 1~100");
        return false;
    }
    if (dogHeartbeatMs <= 0 || dogTimeoutMs <= dogHeartbeatMs)
    {
        if (errMsg) *errMsg = QStringLiteral("Dog/TimeoutMs 必须大于 Dog/HeartbeatMs");
        return false;
    }
    if (detEnabled)
    {
        if (detPort == dogRecvPort)
        {
            if (errMsg) *errMsg = QStringLiteral("Detection/Port 不能与 Dog/RecvPort 相同（%1），两个接收端会抢同一个端口").arg(detPort);
            return false;
        }
        if (detBind != QStringLiteral("0.0.0.0") && !QHostAddress().setAddress(detBind))
        {
            if (errMsg) *errMsg = QStringLiteral("Detection/Bind 无效: %1").arg(detBind);
            return false;
        }
    }
    return true;
}
