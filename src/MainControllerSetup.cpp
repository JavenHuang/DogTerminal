// ============================================================
// MainController 装配段
//
// init() 的顺序不能随意调整：每段的产出都是下一段的输入。
//   ① initLinks()    MQ / 南向驱动 / 导航器 / 姿态 / 稳定器 / 上报 / 检测链
//   ② initChannels() 标定面板、狗与导航/地图参数、差分融合参数与自检日志
//   ③ initSignals()  北向/南向/导航/姿态信号连接、5 个定时器
//   ④ initEngines()  驱动节拍 NavBridge、下行 CommandRouter（依赖前 ③ 的定时器）
// 本 TU 只写装配，不含任何运行期逻辑。
// ============================================================
#include "MainController.h"

#include <QDebug>
#include <QTimer>

#include "Common/DogText.h"
#include "DogLink/UdpDogClient.h"
#ifdef DOGTERMINAL_HAS_SDK
#include "DogLink/SdkDogClient.h"
#endif
#ifdef DOGTERMINAL_HAS_MATRIX
#include "DogLink/MatrixDogClient.h"
#endif
#include "ServerLink/DetectionLink.h"
#include "ServerLink/CalibPanel.h"

// ① 链路：差分跟踪器 + 北向 MQ + 南向驱动 + 导航/姿态/稳定器/上报 + 目标检测
void MainController::initLinks()
{
    // 差分位姿跟踪器：依赖 cfg/地图变换/融合器/姿态快照，均在本对象生命周期内有效
    _rtk = new RtkPoseTracker(_cfg, _map, _fusion, _st);

    // ---- 模块创建与配置 ----
    _mq = new MqLink(this);

    // 南向驱动选择：INI 中 [Dog] Driver=udp|sdk|matrix
    // sdk 仅在 Linux（链接官方 libzsibot）可用；matrix 需链接 zenoh-cxx
    const QString driver = _cfg.dogDriver;
#ifdef DOGTERMINAL_HAS_MATRIX
    if (driver.compare(QStringLiteral("matrix"), Qt::CaseInsensitive) == 0)
    {
        _dog = new MatrixDogClient(this);
        // Zenoh 专属配置（MatrixDogClient 在 configure 里只设端口）
        auto *mc = static_cast<MatrixDogClient *>(_dog);
        mc->setZenohEndpoint(_cfg.matrixZenohEndpoint);
        mc->setZenohMode(_cfg.matrixZenohMode);
        mc->setTopicPrefix(_cfg.matrixTopicPrefix);
        mc->setImuTopic(_cfg.matrixImuTopic);
        mc->setOdomTopic(_cfg.matrixOdomTopic);
        qInfo() << "[Main] 南向驱动: MATRiX 仿真 (UDP 7447 + Zenoh)";
    }
    else
#endif
#ifdef DOGTERMINAL_HAS_SDK
    if (driver.compare(QStringLiteral("sdk"), Qt::CaseInsensitive) == 0)
    {
        _dog = new SdkDogClient(this);
        qInfo() << "[Main] 南向驱动: SDK (libzsibot)";
    }
    else
    {
        _dog = new UdpDogClient(this);
        qInfo() << "[Main] 南向驱动: UDP+JSON";
    }
#else
    if (driver.compare(QStringLiteral("sdk"), Qt::CaseInsensitive) == 0)
    {
        qWarning() << "[Main] 本平台未链接官方 SDK（仅 Linux 可用），[Dog] Driver=sdk 已回退为 udp";
    }
#ifdef DOGTERMINAL_HAS_MATRIX
    else if (driver.compare(QStringLiteral("matrix"), Qt::CaseInsensitive) == 0)
    {
        qWarning() << "[Main] 本平台未链接 zenoh-cxx（仅 Linux 可用），[Dog] Driver=matrix 已回退为 udp";
    }
#endif
    _dog = new UdpDogClient(this);
    qInfo() << "[Main] 南向驱动: UDP+JSON";
#endif

    _navigator = new Navigator(this);
    _attitude = new AttitudeExecutor(this);

    // 航向稳定器：依赖 cfg/地图/导航器/姿态快照；持久化尺度的载入也归它
    _yaw = new YawStabilizer(_cfg, *_navigator, _st, _velCtrlActive);
    _yaw->loadCalibScale();
    if (_cfg.poseRawOdom)
        qWarning().noquote()
            << QStringLiteral("[Pose] 原始模式已开启（Pose/RawOdom=true）：位置与朝向一律直通狗体上报值，"
                              "静止锁/跳变抑制/位移方向校核/参考平移全部关闭。"
                              "断链或遥控接管造成的 yaw 台阶会原样进地图——这是本模式的预期行为。");

    // 上报与诊断：只借用其它子对象，不拥有它们
    _report = new ReportService(_cfg, _mq, _map, *_yaw, *_rtk, _status, _dog, *_navigator,
                                _controlAcquired, _velCtrlActive);
    _printer = new StatePrinter(_cfg, _dog, _map, *_yaw, *_rtk, _st);

    _mq->configure(_cfg.mqHost, _cfg.mqPort, _cfg.mqVhost,
                   _cfg.mqUser, _cfg.mqPassword, _cfg.mqExchange,
                   QStringLiteral("cmd.%1").arg(_cfg.terminalId), _cfg.mqRetryMs);
    // 差分定位走独立路由键（默认 la），与指令同队列、按路由键分流
    if (_cfg.rtkEnabled && !_cfg.rtkRoutingKey.isEmpty())
    {
        _mq->addBindingKey(_cfg.rtkRoutingKey);
        // 多狗共用同一个 la 主题时，靠 Rtk/DiffId 过滤掉别狗的帧；
        // DiffId=0 是"不校验"，多狗部署会把别人的差分收进来，位姿会来回跳。
        if (_cfg.rtkDiffId == 0)
            qWarning().noquote()
                << QStringLiteral("[Rtk] DiffId=0（不校验差分流）：只有一台狗时可忽略；"
                                  "若同一 la 主题下还有别的狗，本终端会收下它们的差分帧，"
                                  "必须各自填写定位单元 ID");
    }

    // ---- 目标检测（YOLO26）：UDP 收包重组 → MQ 转发 ----
    // 只读通道：不参与位姿、导航与任务，服务端按同一份 UDP 协议解析。
    if (_cfg.detEnabled)
    {
        _det = new DetectionLink(this);
        _det->configure(_cfg.detBind, _cfg.detPort, _cfg.detMaxHz,
                        _cfg.detSkipEmpty, _cfg.detVerbose);
        _detRoutingKey = _cfg.detRoutingKey.isEmpty()
                             ? QStringLiteral("detection.%1").arg(_cfg.terminalId)
                             : _cfg.detRoutingKey;
        connect(_det, &DetectionLink::frameReady, this, &MainController::onDetectionFrame);
        connect(_det, &DetectionLink::errorText, this, [](const QString &w) {
            qWarning().noquote() << "[Det]" << w;
        });
        qInfo().noquote()
            << QStringLiteral("[Det] 已配置：UDP %1:%2 → MQ %3（限流 %4Hz，无目标帧%5，报文原样转发）")
                   .arg(_det->bindText()).arg(_cfg.detPort).arg(_detRoutingKey)
                   .arg(_cfg.detMaxHz > 0 ? QString::number(_cfg.detMaxHz)
                                          : QStringLiteral("不限"))
                   .arg(_cfg.detSkipEmpty ? QStringLiteral("跳过") : QStringLiteral("照转"));
    }
}

// ② 旁路通道与参数：标定面板、狗/导航/地图参数、差分融合参数与自检日志
void MainController::initChannels()
{
    // ---- 差分/标定状态面板：1Hz 一份固定字段快照发到 MQ，服务端常驻显示 ----
    // 与目标检测一样是只读通道：不进位姿、不进导航、不改任何现有上报。
    // 差分关闭（Rtk/Enabled=false）时不再创建/发布：现场用纯里程计时这条通道
    // 只会每帧报"差分未启用"，属于噪音。
    if (!_cfg.rtkEnabled)
    {
        qInfo().noquote() << QStringLiteral("[Panel] 差分已关闭（Rtk/Enabled=false）："
                                           "标定面板不启用、calib.%1 不发布")
                                 .arg(_cfg.terminalId);
    }
    else
    {
        _panel = new CalibPanel(this);
        _rtk->setPanel(_panel);
        _panelKey = QStringLiteral("calib.%1").arg(_cfg.terminalId);
        CalibPanel::Snapshot base;
        base.terminalId = _cfg.terminalId;
        base.rtkEnabled = _cfg.rtkEnabled;
        base.autoCalib = _cfg.rtkAutoCalib;
        base.useForPose = _cfg.rtkUseForPose;
        base.holdMs = _cfg.rtkHoldMs;
        base.calibMinSamples = _cfg.rtkCalibMinSamples;
        base.calibMinPathM = _cfg.rtkCalibMinPathM;
        base.calibMaxRmsM = _cfg.rtkCalibMaxRmsM;
        _panel->configure(_mq, _panelKey, 1000, base);
        connect(_panel, &CalibPanel::verdictChanged, this,
                [](const QString &head, const QString &action) {
                    qInfo().noquote() << QStringLiteral("[Panel] 判定: %1 → %2")
                                             .arg(head).arg(action);
                });
        qInfo().noquote()
            << QStringLiteral("[Panel] 标定面板已配置：1Hz → MQ %1（固定 8 行，服务端只留最后一条即常驻不滚动）")
                   .arg(_panelKey);
    }

    _dog->configure(_cfg.dogIp, _cfg.dogSendPort, _cfg.dogRecvPort,
                    _cfg.dogRole, _cfg.dogHeartbeatMs, _cfg.dogTimeoutMs);
    // 心跳角色独立配置，避免周期心跳持续以 sdk 身份抢占控制权
    _dog->setHeartbeatRole(_cfg.dogHeartbeatRole);
    _navigator->setParams(_cfg.navParams);
    _map.setParams(_cfg.mapParams);

    // ---- 差分融合（第二阶段）：差分做锚点、里程计做增量，位姿供导航与上报 ----
    {
        RtkFusion::Params fp;
        fp.autoCalib       = _cfg.rtkAutoCalib;
        fp.useForNav       = _cfg.rtkUseForPose;
        fp.calibMinSamples = _cfg.rtkCalibMinSamples;
        fp.calibMinPathM   = _cfg.rtkCalibMinPathM;
        fp.calibMaxRmsM    = _cfg.rtkCalibMaxRmsM;
        fp.offsetTauSec    = _cfg.rtkOffsetTauSec;
        fp.holdMs          = _cfg.rtkHoldMs;
        fp.useRtkYaw       = _cfg.rtkYawFromRtk;
        fp.calibStaticHoldMs = _cfg.rtkCalibStaticHoldMs;
        fp.calibPointGapM  = _cfg.rtkCalibPointGapM;
        fp.calibStaticAvgFrames = _cfg.rtkCalibStaticAvgFrames;
        fp.calibStaticSpreadM   = _cfg.rtkCalibStaticSpreadM;
        fp.syncOnlyOnTask       = _cfg.rtkSyncOnlyOnTask;
        fp.offStaticSpeed  = _cfg.rtkOffStaticSpeed;
        fp.offSlowSpeed    = _cfg.rtkOffSlowSpeed;
        fp.offSlowYawRateDeg = _cfg.rtkOffSlowYawRateDeg;
        fp.offStaticTauSec = _cfg.rtkOffStaticTauSec;
        fp.offSlowTauSec   = _cfg.rtkOffSlowTauSec;
        fp.autoZeroYaw     = _cfg.rtkAutoZero;
        fp.zeroHoldMs      = _cfg.rtkZeroHoldMs;
        fp.zeroMinFrames   = _cfg.rtkZeroMinFrames;
        fp.calibKMin       = _cfg.rtkCalibKMin;
        fp.calibKMax       = _cfg.rtkCalibKMax;
        fp.calibAlphaMaxDeg = _cfg.rtkCalibAlphaMaxDeg;
        fp.zeroFromLine    = _cfg.rtkZeroFromLine;
        fp.zeroLineSegM    = _cfg.rtkZeroLineSegM;
        fp.zeroLineMinM    = _cfg.rtkZeroLineMinM;
        fp.zeroLineSpreadDeg = _cfg.rtkZeroLineSpreadDeg;
        _fusion.setParams(fp);
        if (_cfg.rtkEnabled)
        {
            QString msg;
            if (_fusion.loadCalib(_rtk->calibPath(), &msg))
                qInfo().noquote() << "[Rtk]" << msg;
            else
                qInfo().noquote() << "[Rtk]" << msg
                                  << "（标定改为停点式：走一段停 2s 采一个锚点，采到 4 个分散停点后自动解出）";
            if (fp.useForNav)
                qInfo().noquote() << "[Rtk] 融合位姿已接管导航与上报（UseForPose=true）";
            qInfo().noquote()
                << QStringLiteral("[Rtk] 静止门控: 速度<%1m/s 且角速度<%2°/s 才采信差分更新偏移；"
                                  "运动中偏移冻结、位姿由里程计增量推进（规避差分 %3ms 滞后）")
                       .arg(fp.offStaticSpeed, 0, 'f', 2).arg(fp.offSlowYawRateDeg, 0, 'f', 1)
                       .arg(500);
            if (!fp.useRtkYaw)
                qInfo().noquote() << "[Rtk] 航向不采信差分（YawFromRtk=false），朝向只走里程计 + 标定转角";
            if (fp.autoZeroYaw)
                qInfo().noquote()
                    << QStringLiteral("[Rtk] 零位策略：建参考后自动取差分航向零位 b（静止 %1ms、%2 帧；"
                                      "b 只存内存，每次建参考/判定模块重启时重取）")
                           .arg(fp.zeroHoldMs).arg(fp.zeroMinFrames);
            qInfo().noquote()
                << QStringLiteral("[Rtk] 停点标定判据：k 需在 %1~%2、|α| ≤ %3°，越界不采用并提示回查 mapcalib")
                       .arg(fp.calibKMin, 0, 'f', 2).arg(fp.calibKMax, 0, 'f', 2)
                       .arg(fp.calibAlphaMaxDeg, 0, 'f', 1);
            if (fp.zeroFromLine && fp.useRtkYaw)
                qInfo().noquote()
                    << QStringLiteral("[Rtk] 零位口径：建参考先取读数当初始值，随后用直线段（每段 ≥%1m、"
                                      "累计 ≥%2m）反算 b 归到地图东——建参考时不必把狗摆正朝东")
                           .arg(fp.zeroLineSegM, 0, 'f', 1).arg(fp.zeroLineMinM, 0, 'f', 1);
        }
    }
}

// ③ 信号与定时器：北向/南向/导航/姿态连接，状态、心跳、看门狗、死手、申请、节拍
void MainController::initSignals()
{
    // ---- 北向信号 ----
    connect(_mq, &MqLink::connected, this, &MainController::onMqConnected);
    connect(_mq, &MqLink::disconnected, this, &MainController::onMqDisconnected);
    connect(_mq, &MqLink::messageReceived, this, &MainController::onMqMessage);
    connect(_mq, &MqLink::errorText, this, &MainController::onMqError);

    // ---- 南向信号 ----
    connect(_dog, &DogClientBase::connectedChanged, this, &MainController::onDogConnectedChanged);
    connect(_dog, &DogClientBase::stateDataChanged, this, &MainController::onDogStateData);

    // ---- 导航信号 ----
    connect(_navigator, &Navigator::remoteCommand, this, &MainController::onNavRemote);
    connect(_navigator, &Navigator::stopRequested, this, &MainController::onNavStop);
    connect(_navigator, &Navigator::finished, this, &MainController::onNavFinished);

    // ---- 姿态执行器 ----
    connect(_attitude, &AttitudeExecutor::cmdCode, this, [this](quint32 code) {
        _dog->sendCmd(code);
    });
    connect(_attitude, &AttitudeExecutor::unknownAction, this, [this](int action) {
        qWarning() << "[Main] 未知姿态动作:" << action;
    });

    // ---- 定时器 ----
    _stateTimer = new QTimer(this);
    _stateTimer->setInterval(qMax(1, 1000 / _cfg.stateRateHz));
    connect(_stateTimer, &QTimer::timeout, this, &MainController::onStateReportTick);

    _hbTimer = new QTimer(this);
    _hbTimer->setInterval(qMax(1, _cfg.heartbeatSec * 1000));
    connect(_hbTimer, &QTimer::timeout, this, &MainController::onHeartbeatTick);

    _watchdogTimer = new QTimer(this);
    _watchdogTimer->setInterval(1000);
    connect(_watchdogTimer, &QTimer::timeout, this, &MainController::onServerWatchdog);

    _velocityTimer = new QTimer(this);
    _velocityTimer->setSingleShot(true);
    _velocityTimer->setInterval(qMax(100, _cfg.velocityTimeoutMs));
    connect(_velocityTimer, &QTimer::timeout, this, &MainController::onVelocityTimeout);

    _acquireTimer = new QTimer(this);
    _acquireTimer->setInterval(_cfg.acquireRetryMs);
    connect(_acquireTimer, &QTimer::timeout, this, &MainController::onControlAcquireTick);

    // 控制节拍：与状态包到达率解耦。旧实现把导航推进和遥控下发都挂在
    // onDogStateData() 上，控制刷新率等于状态包到达率（名义 1Hz，空洞时 3~4s），
    // 狗每拍收到一帧摇杆量后随即失去刷新 → 走走停停。这里用独立 20Hz 定时器
    // 以最近一次状态连续推进导航并续发遥控量。
    _ctrlTimer = new QTimer(this);
    _ctrlTimer->setInterval(qMax(10, _cfg.navCtrlTickMs));
    connect(_ctrlTimer, &QTimer::timeout, this, &MainController::onControlTick);
}

// ④ 运行时对象：驱动节拍与下行指令路由（都依赖 ③ 建好的定时器与子对象）
void MainController::initEngines()
{
    // 驱动节拍：借走两个定时器与运行标志，直控续发缓存归它独占
    _nav = new NavBridge(_cfg, *_navigator, *_rtk, _dog, _st, *_report,
                         _acquireTimer, _velocityTimer, _running,
                         _controlAcquired, _velCtrlActive);

    // 下行指令：执行器由本类持有，路由器只借用
    _cmd = new CommandRouter(_cfg, *_navigator, *_attitude, _map, *_yaw, _dog,
                             _controlAcquired, _velCtrlActive, *_report,
                             _nav->velFwdRef(), _nav->velTurnRef(), _nav->velShiftRef(), *this);
}
