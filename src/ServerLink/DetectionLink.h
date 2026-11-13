#ifndef DETECTION_LINK_H
#define DETECTION_LINK_H

#include <QObject>
#include <QByteArray>
#include <QHash>
#include <QString>

class QUdpSocket;
class QTimer;

// ============================================================
// 目标检测（YOLO26）UDP 接收 → 北向上行
//
// 上行链路：188 的检测服务把每帧推理结果以
//   robotdog.detection               完整帧（JSON）
//   robotdog.detection.fragment      大结果分片（base64-json-utf8，
//                                    每分片 ≤600B、每个 UDP 包 ≤1200B）
// 两种形式发到本机端口。本模块负责：
//   收包 → 按 (来源地址, message_id) 归组 → 按 part_index 升序拼接
//        → Base64 解码 → UTF-8 JSON → 校验 → 原样交给上层
//
// 设计约束：
//   - 不改写报文内容：转发出去的就是 188 发出的那份 JSON（字段由服务端按
//     协议文档消费），终端只补"哪个终端"（体现在 MQ 路由键 detection.<Id>）。
//   - 3s 内不完整的整帧丢弃；重复分片、重复帧不重复提交。
//   - 内存有上限：挂起帧数超过上限时丢最老的，避免异常对端把内存吃满。
//   - 本模块只负责"收 + 重组 + 统计"，是否发 MQ 由 MainController 决定，
//     因此可以先只开日志观察链路，不影响任何现有上报。
// ============================================================
class DetectionLink : public QObject
{
    Q_OBJECT
public:
    explicit DetectionLink(QObject *parent = nullptr);
    ~DetectionLink() override;

    // bindAddr 为空或 "0.0.0.0" = 监听所有网卡
    // maxHz<=0 不限流；skipEmpty=true 时不转发 detections 为空的帧
    void configure(const QString &bindAddr, quint16 port,
                   int maxHz = 10, bool skipEmpty = true, bool verbose = false);

    bool start(QString *errMsg);
    void stop();
    bool isRunning() const { return _running; }

    QString bindText() const;

signals:
    // payloadUtf8：重组后的原始 JSON 字节（与 188 发出的内容一致，未改写）
    // detectionCount：detections 数组长度（0 = 该帧无目标）
    // peer：来源 IP（用于多设备同网段时排查）
    void frameReady(const QByteArray &payloadUtf8, int detectionCount, const QString &peer);
    void errorText(const QString &what);

private slots:
    void onReadyRead();
    void onStatsTick();
    void onStallTick();

private:
    struct Assembly
    {
        qint64 atMs;
        int count;
        QHash<int, QByteArray> parts;
        Assembly() : atMs(0), count(0) {}
    };

    void handleDatagram(const QByteArray &dg, const QString &peer, qint64 nowMs);
    // 校验并取出 detections 个数；head 为给日志用的摘要（首帧打印）
    bool parseFrameMeta(const QByteArray &raw, int *count, QString *head) const;
    void emitFrame(const QByteArray &raw, int count, const QString &peer, qint64 nowMs);
    void expire(qint64 nowMs);
    void noteBadDatagram(const QString &why);
    qint64 nowMs() const;

    QUdpSocket *_sock;
    QTimer *_statsTimer;
    QTimer *_stallTimer;

    QString _bindAddr;
    quint16 _port;
    int _maxHz;
    bool _skipEmpty;
    bool _verbose;
    bool _running;

    QHash<QString, Assembly> _pending;    // key = peer + '|' + message_id
    QHash<QString, qint64> _completed;    // 同上，3s 内去重

    qint64 _startedMs;
    qint64 _lastRecvMs;
    qint64 _lastEmitMs;
    qint64 _lastBadWarnMs;
    bool _anyRecv;
    bool _stallWarned;
    bool _firstFrameLogged;

    // 统计（累计）
    quint64 _frames;
    quint64 _emitted;
    quint64 _emptySkipped;
    quint64 _rateSkipped;
    quint64 _badDatagrams;
    quint64 _expired;
    quint64 _bytes;
    int _maxDetections;
    // 统计（5s 窗口）
    quint64 _winFrames;
    quint64 _winBytes;
    quint64 _winDetections;
    quint64 _winEmitted;
};

#endif  // DETECTION_LINK_H
