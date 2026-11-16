#ifndef RTK_POSE_TRACKER_H
#define RTK_POSE_TRACKER_H

#include <QByteArray>
#include <QString>
#include <QtGlobal>

#include "Control/RtkJumpGate.h"
#include "Convert/RtkAnchor.h"
#include "Convert/RtkFrameReader.h"
#include "Convert/RtkFusion.h"   // RtkFusion::SrcNone/SrcRtk/SrcHold（对外返回值）

struct MotionState;
class TerminalConfig;
class MapTransform;
class CalibPanel;

// ============================================================
// 差分(RTK)位姿跟踪器
//
// 职责（且只有这一件事）：把"服务器/差分模块给的绝对位姿"变成终端可用的位姿源。
//   · 收帧：取读取器读出的位姿，做接收周期/航向范围统计、是否断流判定
//   · 观测：位置/航向与里程计的偏差统计、Δψ 增量校核、首帧基线日志
//   · 融合：与里程计增量融合出局部位姿与地图位姿（RtkFusion 负责数学）
//   · 标定：k/α 与零位（b）的在线求解与 rtkcalib_<id>.ini 持久化
//   · 面板：1Hz 快照送 CalibPanel
//
// 已拆出的协作类（各自单一职责，不反向依赖本类）：
//   · RtkFrameReader：载荷(JSON/二进制) → AbsPoseCmd，含 ID 过滤与前几帧原样打印
//   · RtkJumpGate   ：单帧跳变门控（野值丢弃 / 真转向 / 模块重启 / 轴向错）
//
// 明确不负责：导航推进、上报打包、命令下发、地图参考的建立（只会在参考变更时被通知）。
// 依赖以引用注入，本类不持有也不销毁这些对象。
// ============================================================
class RtkPoseTracker
{
public:
    RtkPoseTracker(const TerminalConfig &cfg, MapTransform &map,
                   RtkFusion &fusion, MotionState &state);
    ~RtkPoseTracker() = default;

    RtkPoseTracker(const RtkPoseTracker &) = delete;
    RtkPoseTracker &operator=(const RtkPoseTracker &) = delete;

    // 标定面板（可空）：[Rtk] Enabled=false 时仍会创建，用于显示"纯里程计基线"
    void setPanel(CalibPanel *panel) { _panel = panel; }

    // 断流告警（只告警不干预；阈值 = max(配置值, 3 个实测平均周期)）
    void checkStale();

    // 收一帧差分位姿（MQ 通道载荷不带本终端协议头，故无时间戳参数）
    void onFrame(const QByteArray &body);

    // 融合位姿：局部坐标（导航用）/ 地图坐标（上报用）
    bool fusedLocalPose(double *lx, double *ly, double *lyaw, int *src) const;
    bool fusedMapPose(double *mx, double *my, double *myaw, int *src) const;

    // 当前导航位姿来源（RtkFusion::SrcNone / SrcRtk / SrcHold）
    int poseSrc() const { return _navPoseSrc; }
    void setPoseSrc(int src) { _navPoseSrc = src; }

    // 地图参考变更：融合偏移属旧参考，必须清空；重启判据的里程计基准也一并作废
    void onMapRefReset();

    // 任务边界同步：导航开始/结束时各喂一次，融合只在边沿吸附（段内不跟踪）
    void setNavActive(bool active);

    // 标定持久化路径（rtkcalib_<id>.ini，与 mapcalib 同目录）
    QString calibPath() const;

    // ---- 静置锚点（[Rtk] AnchorMode=rest）----
    // 只产"停稳后的差位置中值"，不产生位姿流；要不要修正、怎么换算成局部
    // 由 MainController 决定（见 MainControllerPose::applyRtkAnchor）。
    bool takeAnchor(double *x, double *y, int *frames, double *spread);
    QString anchorText() const { return _anchor.describe(); }
    qint64 lastAnchorFrameMs() const { return _anchor.lastFrameMs(); }

    // ---- 最近一帧差分位姿（已通过跳变门控）----
    // 给"差分位置跟随"用：数值变了才代表跨了格点，没变就是同格冻结。
    // 返回 false 表示还没收到过可用的帧（或被门控挡住，值保持在上一帧）。
    // yawRad：可选出参，地图系航向（0=正东，顺时针为正，rad）；不需要传 nullptr。
    bool lastFix(double *x, double *y, qint64 *recvMs, double *yawRad = nullptr) const;

    // 诊断：是否已收到过差分帧 / 最近一帧时刻（上报与打印用）
    bool seen() const { return _rtkSeen; }
    qint64 lastRecvMs() const { return _rtkRecvMs; }
    quint32 frames() const { return _rtkFrames; }
    quint32 badJumps() const { return _jump.badJumps(); }

private:
    const TerminalConfig &_cfg;
    MapTransform &_map;
    RtkFusion &_fusion;
    MotionState &_st;
    CalibPanel *_panel = nullptr;
    RtkFrameReader _reader;   // 载荷 → AbsPoseCmd
    RtkJumpGate _jump;        // 单帧跳变/重启门控
    RtkAnchor _anchor;        // 静置窗口取中值 → 锚点（只产点，不改位姿）

    // ---- 观测状态 ----
    bool   _rtkSeen = false;
    bool   _rtkOriginValid = false;
    quint32 _rtkFrames = 0;
    double _rtkX = 0.0;
    double _rtkY = 0.0;
    double _rtkYaw = 0.0;
    qint64 _rtkRecvMs = 0;
    qint64 _rtkStartMs = 0;
    qint64 _rtkLastInterMs = 0;
    qint64 _rtkMaxGapMs = 0;
    qint64 _lastRtkPrintMs = 0;
    bool   _rtkStaleWarned = false;
    double _rtkBiasSumDeg = 0.0;
    int    _rtkBiasCount = 0;
    // 常值偏差的低通值：只反映"当前"偏差。累计均值（_rtkBiasSumDeg 那一路）
    // 会被历史稀释——重新标定后正常了也长期停在高位，所以告警看这一路。
    double _rtkBiasLpfDeg = 0.0;
    bool   _rtkBiasWarned = false;   // α 松动告警的迟滞标志（回落一半以上才允许再报）
    int    _navPoseSrc = 0;
    double _rtkYawMinDeg = 1e9;
    double _rtkYawMaxDeg = -1e9;

    // ---- Δψ 增量校核（差分航向增量 vs 里程计航向增量）----
    double _rtkPrevRtkYawRad = 0.0;
    double _rtkPrevMapYawRad = 0.0;
    qint64 _rtkPrevCheckMs = 0;
    qint64 _rtkDpsiWarnMs = 0;
};

#endif  // RTK_POSE_TRACKER_H
