#include "DetectionLink.h"

#include <QAbstractSocket>
#include <QDateTime>
#include <QDebug>
#include <QHostAddress>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>
#include <QMutableHashIterator>
#include <QStringList>
#include <QTimer>
#include <QUdpSocket>

namespace {

// 协议上限（与 188 发送端、tools/receive_detection_udp.py 一致）
const int    kMaxPacketBytes = 1200;   // 单个 UDP 包上限
const int    kMaxPartBytes   = 600;    // 单分片解码后上限
const int    kMaxParts       = 219;    // 单帧最多分片数（219 * 600B ≈ 131KB）
const qint64 kAssemblyMs     = 3000;   // 3s 不完整即丢弃整帧
const int    kMaxPendingSets = 64;     // 挂起帧上限（约 8MB 最坏情况）
const int    kStatsSec       = 5;

// 严格 Base64：Qt 的 fromBase64 会静默忽略非法字符，这里按发送端编码规则先校验
bool isBase64Strict(const QByteArray &s)
{
    if (s.isEmpty() || (s.size() % 4) != 0)
        return false;
    for (int i = 0; i < s.size(); ++i)
    {
        const char c = s.at(i);
        const bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')
                        || (c >= '0' && c <= '9') || c == '+' || c == '/' || c == '=';
        if (!ok)
            return false;
    }
    return true;
}

// QJson 的数字统一为 double，这里要求"数字且为整数"
bool intField(const QJsonObject &o, const char *key, int *out)
{
    const QJsonValue v = o.value(QString::fromLatin1(key));
    if (!v.isDouble())
        return false;
    const double d = v.toDouble();
    const int i = static_cast<int>(d);
    if (static_cast<double>(i) != d)
        return false;
    *out = i;
    return true;
}

}  // namespace

DetectionLink::DetectionLink(QObject *parent)
    : QObject(parent)
    , _sock(new QUdpSocket(this))
    , _statsTimer(new QTimer(this))
    , _stallTimer(new QTimer(this))
    , _bindAddr(QStringLiteral("0.0.0.0"))
    , _port(5006)
    , _maxHz(10)
    , _skipEmpty(true)
    , _verbose(false)
    , _running(false)
    , _startedMs(0)
    , _lastRecvMs(0)
    , _lastEmitMs(0)
    , _lastBadWarnMs(0)
    , _anyRecv(false)
    , _stallWarned(false)
    , _firstFrameLogged(false)
    , _frames(0)
    , _emitted(0)
    , _emptySkipped(0)
    , _rateSkipped(0)
    , _badDatagrams(0)
    , _expired(0)
    , _bytes(0)
    , _maxDetections(0)
    , _winFrames(0)
    , _winBytes(0)
    , _winDetections(0)
    , _winEmitted(0)
{
    connect(_sock, &QUdpSocket::readyRead, this, &DetectionLink::onReadyRead);

    _statsTimer->setInterval(kStatsSec * 1000);
    connect(_statsTimer, &QTimer::timeout, this, &DetectionLink::onStatsTick);

    _stallTimer->setInterval(2000);
    connect(_stallTimer, &QTimer::timeout, this, &DetectionLink::onStallTick);
}

DetectionLink::~DetectionLink()
{
    stop();
}

void DetectionLink::configure(const QString &bindAddr, quint16 port,
                              int maxHz, bool skipEmpty, bool verbose)
{
    _bindAddr = bindAddr.trimmed();
    if (_bindAddr.isEmpty())
        _bindAddr = QStringLiteral("0.0.0.0");
    if (port > 0)
        _port = port;
    _maxHz = maxHz;
    _skipEmpty = skipEmpty;
    _verbose = verbose;
}

QString DetectionLink::bindText() const
{
    return _bindAddr.isEmpty() ? QStringLiteral("0.0.0.0") : _bindAddr;
}

bool DetectionLink::start(QString *errMsg)
{
    if (_running)
        return true;

    QHostAddress addr;
    if (_bindAddr.isEmpty() || _bindAddr == QLatin1String("0.0.0.0"))
    {
        addr = QHostAddress(QHostAddress::AnyIPv4);
    }
    else if (!addr.setAddress(_bindAddr))
    {
        if (errMsg)
            *errMsg = QStringLiteral("监听地址无效: %1").arg(_bindAddr);
        return false;
    }

    if (!_sock->bind(addr, _port))
    {
        if (errMsg)
            *errMsg = QStringLiteral("绑定 UDP %1:%2 失败: %3")
                          .arg(bindText()).arg(_port).arg(_sock->errorString());
        return false;
    }

    // 10Hz、约 6KB/帧 会有突发：放大内核接收缓冲，减少瞬时丢包
    _sock->setSocketOption(QAbstractSocket::ReceiveBufferSizeSocketOption, 1 << 20);

    _running = true;
    _startedMs = nowMs();
    _lastRecvMs = 0;
    _lastEmitMs = 0;
    _anyRecv = false;
    _stallWarned = false;
    _firstFrameLogged = false;
    _pending.clear();
    _completed.clear();
    _winFrames = _winBytes = _winDetections = _winEmitted = 0;

    _statsTimer->start();
    _stallTimer->start();

    qInfo().noquote()
        << QStringLiteral("[Det] UDP 检测接收已启动 %1:%2（每包 ≤%3B，分片 ≤%4B，最多 %5 片，超时 %6ms）")
               .arg(bindText()).arg(_port).arg(kMaxPacketBytes)
               .arg(kMaxPartBytes).arg(kMaxParts).arg(kAssemblyMs);
    return true;
}

void DetectionLink::stop()
{
    if (!_running)
        return;

    _statsTimer->stop();
    _stallTimer->stop();
    _sock->close();
    _running = false;
    _pending.clear();
    _completed.clear();

    qInfo().noquote()
        << QStringLiteral("[Det] UDP 检测接收已停止（累计 帧=%1 转发=%2 空帧跳过=%3 坏包=%4）")
               .arg(_frames).arg(_emitted).arg(_emptySkipped).arg(_badDatagrams);
}

qint64 DetectionLink::nowMs() const
{
    return QDateTime::currentMSecsSinceEpoch();
}

void DetectionLink::onReadyRead()
{
    const qint64 t = nowMs();
    while (_sock->hasPendingDatagrams())
    {
        const qint64 sz = _sock->pendingDatagramSize();
        if (sz <= 0)
        {
            QByteArray dummy;
            dummy.resize(1);
            _sock->readDatagram(dummy.data(), 1);
            continue;
        }

        QByteArray dg;
        dg.resize(static_cast<int>(sz));
        QHostAddress from;
        quint16 fromPort = 0;
        const qint64 got = _sock->readDatagram(dg.data(), dg.size(), &from, &fromPort);
        if (got <= 0)
            break;
        dg.resize(static_cast<int>(got));

        _lastRecvMs = t;
        if (!_anyRecv)
            _anyRecv = true;
        if (_stallWarned)
        {
            _stallWarned = false;
            qInfo().noquote() << QStringLiteral("[Det] UDP 检测数据已恢复（来源 %1:%2）")
                                     .arg(from.toString()).arg(fromPort);
        }

        handleDatagram(dg, from.toString(), t);
    }
    expire(t);
}

void DetectionLink::handleDatagram(const QByteArray &dg, const QString &peer, qint64 t)
{
    if (dg.size() > kMaxPacketBytes)
    {
        noteBadDatagram(QStringLiteral("UDP 包超过 %1 字节（来自 %2，%3B）")
                            .arg(kMaxPacketBytes).arg(peer).arg(dg.size()));
        return;
    }

    QJsonParseError pe;
    const QJsonDocument doc = QJsonDocument::fromJson(dg, &pe);
    if (pe.error != QJsonParseError::NoError || !doc.isObject())
    {
        noteBadDatagram(QStringLiteral("不是合法 JSON（来自 %1：%2）")
                            .arg(peer).arg(pe.errorString()));
        return;
    }

    const QJsonObject o = doc.object();
    int schema = 0;
    if (!intField(o, "schema", &schema) || schema != 1)
    {
        noteBadDatagram(QStringLiteral("schema 非 1（来自 %1）").arg(peer));
        return;
    }

    const QString type = o.value(QStringLiteral("type")).toString();

    // 完整帧：小结果可能不分片，直接当一帧提交
    if (type == QLatin1String("robotdog.detection"))
    {
        int count = 0;
        if (!parseFrameMeta(dg, &count, nullptr))
        {
            noteBadDatagram(QStringLiteral("完整检测帧无效（来自 %1）").arg(peer));
            return;
        }
        emitFrame(dg, count, peer, t);
        return;
    }

    if (type != QLatin1String("robotdog.detection.fragment"))
    {
        noteBadDatagram(QStringLiteral("不支持的报文类型 \"%1\"（来自 %2）").arg(type).arg(peer));
        return;
    }

    // ---- 分片 ----
    const QString mid = o.value(QStringLiteral("message_id")).toString();
    int index = 0;
    int count = 0;
    if (mid.isEmpty()
        || !intField(o, "part_index", &index)
        || !intField(o, "part_count", &count)
        || index < 0 || index >= count || count > kMaxParts)
    {
        noteBadDatagram(QStringLiteral("分片头无效（来自 %1 message_id=%2）").arg(peer).arg(mid));
        return;
    }
    if (o.value(QStringLiteral("encoding")).toString() != QLatin1String("base64-json-utf8"))
    {
        noteBadDatagram(QStringLiteral("分片编码不支持（来自 %1）").arg(peer));
        return;
    }

    const QByteArray b64 = o.value(QStringLiteral("data")).toString().toLatin1();
    if (!isBase64Strict(b64))
    {
        noteBadDatagram(QStringLiteral("分片 Base64 无效（来自 %1 第 %2/%3 片）")
                            .arg(peer).arg(index + 1).arg(count));
        return;
    }
    const QByteArray part = QByteArray::fromBase64(b64);
    if (part.isEmpty() || part.size() > kMaxPartBytes)
    {
        noteBadDatagram(QStringLiteral("分片长度无效（%1B，来自 %2）").arg(part.size()).arg(peer));
        return;
    }

    // key = 来源地址 + message_id（发送端源端口由系统分配，不参与归组，避免端口变化后丢帧）
    const QString key = peer + QLatin1Char('|') + mid;
    if (_completed.contains(key))
        return;   // 重复帧：已提交过

    if (!_pending.contains(key) && _pending.size() >= kMaxPendingSets)
    {
        // 挂起过多：丢最老的一帧，防止异常对端把内存吃满
        qint64 oldest = 0;
        QString oldestKey;
        bool firstSel = true;
        for (auto it = _pending.constBegin(); it != _pending.constEnd(); ++it)
        {
            if (firstSel || it.value().atMs < oldest)
            {
                oldest = it.value().atMs;
                oldestKey = it.key();
                firstSel = false;
            }
        }
        if (!oldestKey.isEmpty())
        {
            _pending.remove(oldestKey);
            ++_expired;
        }
    }

    Assembly &a = _pending[key];
    if (a.parts.isEmpty())
    {
        a.atMs = t;
        a.count = count;
    }
    if (a.count != count)
    {
        _pending.remove(key);
        noteBadDatagram(QStringLiteral("分片数量不一致（message_id=%1）").arg(mid));
        return;
    }
    a.parts.insert(index, part);
    if (a.parts.size() != count)
        return;   // 还缺片

    QByteArray raw;
    raw.reserve(count * kMaxPartBytes);
    for (int i = 0; i < count; ++i)
        raw.append(a.parts.value(i));
    _pending.remove(key);
    _completed.insert(key, t);

    int detCount = 0;
    if (!parseFrameMeta(raw, &detCount, nullptr))
    {
        noteBadDatagram(QStringLiteral("重组后报文无效（message_id=%1，%2 片）").arg(mid).arg(count));
        return;
    }
    emitFrame(raw, detCount, peer, t);
}

bool DetectionLink::parseFrameMeta(const QByteArray &raw, int *count, QString *head) const
{
    QJsonParseError pe;
    const QJsonDocument doc = QJsonDocument::fromJson(raw, &pe);
    if (pe.error != QJsonParseError::NoError || !doc.isObject())
        return false;

    const QJsonObject o = doc.object();
    int schema = 0;
    if (!intField(o, "schema", &schema) || schema != 1)
        return false;
    if (o.value(QStringLiteral("type")).toString() != QLatin1String("robotdog.detection"))
        return false;

    const QJsonArray dets = o.value(QStringLiteral("detections")).toArray();
    if (count)
        *count = dets.size();

    if (head)
    {
        const QJsonArray isz = o.value(QStringLiteral("image_size")).toArray();
        QString sizeText = QStringLiteral("?");
        if (isz.size() >= 2)
            sizeText = QStringLiteral("%1x%2")
                           .arg(static_cast<int>(isz.at(0).toDouble()))
                           .arg(static_cast<int>(isz.at(1).toDouble()));

        QStringList labels;
        for (int i = 0; i < dets.size() && i < 3; ++i)
        {
            const QJsonObject d = dets.at(i).toObject();
            labels << QStringLiteral("%1 %2%")
                          .arg(d.value(QStringLiteral("label")).toString())
                          .arg(QString::number(d.value(QStringLiteral("confidence")).toDouble()
                                               * 100.0, 'f', 0));
        }

        *head = QStringLiteral("session=%1 model=%2 图像=%3 目标=%4%5")
                    .arg(o.value(QStringLiteral("session")).toString())
                    .arg(o.value(QStringLiteral("model")).toString())
                    .arg(sizeText)
                    .arg(dets.size())
                    .arg(labels.isEmpty()
                             ? QString()
                             : QStringLiteral(" [") + labels.join(QStringLiteral(", ")) + QLatin1Char(']'));
    }
    return true;
}

void DetectionLink::emitFrame(const QByteArray &raw, int count, const QString &peer, qint64 t)
{
    ++_frames;
    ++_winFrames;
    _bytes += static_cast<quint64>(raw.size());
    _winBytes += static_cast<quint64>(raw.size());
    _winDetections += static_cast<quint64>(count);
    if (count > _maxDetections)
        _maxDetections = count;

    if (!_firstFrameLogged)
    {
        _firstFrameLogged = true;
        QString head;
        int c = 0;
        if (parseFrameMeta(raw, &c, &head))
            qInfo().noquote() << QStringLiteral("[Det] 首帧到达：%1（%2B，来自 %3）")
                                     .arg(head).arg(raw.size()).arg(peer);
    }

    // 限流（0=不限）：188 推理上限 10FPS，这里只防"上游改大后把 MQ 打满"
    if (_maxHz > 0 && _lastEmitMs > 0 && (t - _lastEmitMs) < (1000 / _maxHz))
    {
        ++_rateSkipped;
        return;
    }

    if (count == 0 && _skipEmpty)
    {
        ++_emptySkipped;
        return;
    }

    _lastEmitMs = t;
    ++_emitted;
    ++_winEmitted;

    if (_verbose)
    {
        qInfo().noquote() << QStringLiteral("[Det] 帧 目标=%1 %2B 来自 %3")
                                 .arg(count).arg(raw.size()).arg(peer);
    }

    emit frameReady(raw, count, peer);
}

void DetectionLink::noteBadDatagram(const QString &why)
{
    ++_badDatagrams;
    const qint64 t = nowMs();
    // 10Hz 坏包会刷屏：1s 内只打一条
    if (t - _lastBadWarnMs < 1000)
        return;
    _lastBadWarnMs = t;
    qWarning().noquote() << QStringLiteral("[Det] 忽略无效报文（累计 %1）：%2").arg(_badDatagrams).arg(why);
}

void DetectionLink::expire(qint64 t)
{
    QMutableHashIterator<QString, Assembly> it(_pending);
    while (it.hasNext())
    {
        it.next();
        if (t - it.value().atMs > kAssemblyMs)
        {
            it.remove();
            ++_expired;
        }
    }

    QMutableHashIterator<QString, qint64> ci(_completed);
    while (ci.hasNext())
    {
        ci.next();
        if (t - ci.value() > kAssemblyMs)
            ci.remove();
    }
}

void DetectionLink::onStatsTick()
{
    expire(nowMs());

    if (_winFrames == 0 && _pending.isEmpty())
        return;   // 没数据就不刷屏

    const double avg = _winFrames ? double(_winBytes) / double(_winFrames) : 0.0;
    qInfo().noquote()
        << QStringLiteral("[Det] %1s: 帧=%2 目标=%3(峰值%4) 转发=%5 空帧跳过=%6 限流=%7 坏包=%8 超时=%9 平均=%10B 挂起=%11")
               .arg(kStatsSec)
               .arg(_winFrames).arg(_winDetections).arg(_maxDetections)
               .arg(_winEmitted)
               .arg(_emptySkipped).arg(_rateSkipped).arg(_badDatagrams).arg(_expired)
               .arg(static_cast<int>(avg))
               .arg(_pending.size());

    _winFrames = 0;
    _winBytes = 0;
    _winDetections = 0;
    _winEmitted = 0;
}

void DetectionLink::onStallTick()
{
    if (!_running)
        return;

    const qint64 t = nowMs();

    if (!_anyRecv)
    {
        if (t - _startedMs > 30000 && !_stallWarned)
        {
            _stallWarned = true;
            qWarning().noquote()
                << QStringLiteral("[Det] 启动 30s 未收到任何 UDP 检测数据：核对 188 网页「UDP结果发送」里的接收 IP 是否为本机、"
                                  "端口是否为 %1，以及本机防火墙是否放行该端口").arg(_port);
        }
        return;
    }

    if (t - _lastRecvMs > 8000 && !_stallWarned)
    {
        _stallWarned = true;
        qWarning().noquote()
            << QStringLiteral("[Det] 已 %1s 未收到 UDP 检测数据（检查 188 的 GPU 检测是否在跑、UDP 发送是否被关掉）")
                   .arg((t - _lastRecvMs) / 1000);
    }
}
