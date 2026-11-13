#ifndef CALIB_PANEL_H
#define CALIB_PANEL_H

#include <QObject>
#include <QString>
#include <QStringList>

class MqLink;

// ============================================================
// 差分 / 标定状态面板（现场"长显示"用）
//
// 现场问题：终端控制台是滚动的，几秒就刷走，人得翻日志找 b、k、锚点、对差；
// 服务端若也按日志式显示，同样一直在滚。这里把"人在现场真正要看的少数几个量
// + 下一步该干什么"压成**固定字段、固定行数**的一份快照，按秒级周期重发：
//
//   · 同一条消息原地覆盖（服务端每个路由键只保留最后一条即可），所以不滚动；
//   · lines[] 固定 8 行、顺序固定，服务端逐行贴到 8 个 label 就是一块常驻面板；
//   · verdict.action 是"下一步做什么"的人话指令，由终端当场判定，
//     服务端不需要懂标定逻辑，只显示；
//   · fields 分区（rtk / zero / calib / pose）给需要做判据/报警的服务端用。
//
// 只发不控：不进位姿、不进导航、不改任何现有上报。MQ 未连接时 publish 为 no-op。
// ============================================================
class CalibPanel : public QObject
{
    Q_OBJECT
public:
    // 一次快照：全部都是终端里已有的量，不做任何推测
    struct Snapshot
    {
        qint64 nowMs = 0;
        int    terminalId = 0;

        // 差分链路
        bool    autoCalib = true;     // 在线自动解 k/α（false 时"未解"不是缺点数，而是没开自动解）
        bool    rtkEnabled = false;
        bool    rtkSeen = false;
        qint64  ageMs = 0;
        qint64  periodMs = 0;
        qint64  gapMaxMs = 0;
        quint32 frames = 0;
        quint32 badJumps = 0;
        int     restarts = 0;
        double  yawRangeDeg = 0.0;
        double  biasAvgDeg = 0.0;
        double  rtkX = 0.0, rtkY = 0.0, rtkYawDeg = 0.0;

        // 航向零位 b
        bool   zeroValid = false;
        bool   zeroPending = false;
        bool   zeroFromLine = false;
        double bDeg = 0.0;
        double bSpreadDeg = 0.0;

        // k / α 标定
        bool   calibValid = false;
        double k = 1.0;
        double alphaDeg = 0.0;
        double alphaYawDeg = 0.0;
        double rmsM = 0.0;
        double pathM = 0.0;
        int    anchors = 0;

        // 位姿
        bool   hasMap = false;
        bool   hasFused = false;
        double mapX = 0.0, mapY = 0.0, mapYawDeg = 0.0;
        double fusedX = 0.0, fusedY = 0.0, fusedYawDeg = 0.0;
        double fusedVsRtkM = 0.0;
        double fusedVsRtkYawDeg = 0.0;
        int    src = 0;               // RtkFusion::Source
        bool   useForPose = false;
        double speedMps = 0.0;
        double staticSec = 0.0;
        double gyroBiasDps = 0.0;
        double dpsiDeg = 0.0;
        bool   dpsiValid = false;

        // 判据阈值（取自 [Rtk]，保证面板与终端日志同口径）
        int    calibMinSamples = 4;
        double calibMinPathM = 0.8;
        double calibMaxRmsM = 0.12;
        int    holdMs = 2000;
    };

    struct Verdict
    {
        QString code;     // 机器可判的分支码
        QString head;     // 一句话结论
        QString action;   // 下一步该做什么（人话）
        QString gate;     // 硬闸（必须在做下一步之前处理的事）
    };

    explicit CalibPanel(QObject *parent = nullptr);

    // base：已知的配置态快照（没收到差分帧时也能发一块"等待差分"的面板）
    void configure(MqLink *mq, const QString &routingKey, int minPeriodMs,
                   const Snapshot &base);

    // 有新汇总数据时调用（内部按 minPeriodMs 节流，默认 1Hz）
    void feed(const Snapshot &s);

    // 没有新数据时保活：刷新断流时长，必要时切成"差分断流"判定。
    // 由状态上报节拍调用（内部同样按 minPeriodMs 节流）。
    void pollStale(qint64 nowMs);

signals:
    // 判定发生变化（仅变化时发一次），供终端打印一行，不刷屏
    void verdictChanged(const QString &head, const QString &action);

private:
    void publish(Snapshot s);
    void updateDrift(const Snapshot &s);
    Verdict judge(const Snapshot &s) const;   // 读 _driftDps 判硬闸
    QByteArray buildJson(const Snapshot &s, const Verdict &v) const;
    QStringList buildLines(const Snapshot &s, const Verdict &v) const;

    MqLink *_mq = nullptr;
    QString _key;
    int     _minPeriodMs = 1000;
    Snapshot _base;
    Snapshot _last;
    bool   _hasBase = false;
    bool   _hasLast = false;
    qint64 _lastFeedMs = 0;
    qint64 _lastPubMs = 0;
    quint64 _seq = 0;
    QString _lastVerdictCode;

    // 静止期间地图 yaw 的漂移速率（°/s）：靠相邻两次静止快照差分得到
    bool   _driftPrevValid = false;
    bool   _driftHas = false;
    double _driftPrevYawDeg = 0.0;
    qint64 _driftPrevMs = 0;
    double _driftDps = 0.0;
};

#endif  // CALIB_PANEL_H
