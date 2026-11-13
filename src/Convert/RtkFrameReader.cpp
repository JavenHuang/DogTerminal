// ============================================================
// 差分载荷读取器 —— 实现
// 解析逻辑整体来自 RtkPoseTracker::onFrame 的"收帧/解析"段，行为不变：
// 同一套候选取键名、同一套合理性阈值、同样的日志文本。
// ============================================================
#include "Convert/RtkFrameReader.h"

#include <cmath>
#include <QDataStream>
#include <QDebug>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QJsonValue>
#include <QStringList>

namespace
{

// 按多个候选键名取数值（不同服务端可能写 yaw / heading / angle / 航向）
bool rtkJsonNumber(const QJsonObject &o, const QStringList &keys, double *out)
{
    for (const QString &k : keys)
    {
        if (!o.contains(k))
            continue;
        const QJsonValue v = o.value(k);
        if (v.isDouble()) { *out = v.toDouble(); return true; }
        if (v.isString())
        {
            bool ok = false;
            const double d = v.toString().trimmed().toDouble(&ok);
            if (ok) { *out = d; return true; }
        }
    }
    return false;
}

// 数值合理性：差分坐标不该超过 100 km，高程不该超过 10 km，航向不该超过 720°
bool absPoseSane(const DogProtocol::AbsPoseCmd &c)
{
    if (!std::isfinite(c.x) || !std::isfinite(c.y) || !std::isfinite(c.z) || !std::isfinite(c.yawDeg))
        return false;
    return std::fabs(c.x) <= 1e5 && std::fabs(c.y) <= 1e5
           && std::fabs(c.z) <= 1e4 && std::fabs(c.yawDeg) <= 720.0;
}

// 十六进制转储：二进制报文直接打印会显示成空白，核对字段只能看 hex
QString hexDump(const QByteArray &b, int maxBytes)
{
    const int n = qMin(b.size(), maxBytes);
    QString s;
    for (int i = 0; i < n; ++i)
    {
        if (i % 16 == 0)
            s += QStringLiteral("\n  %1: ").arg(i, 4, 10, QLatin1Char('0'));
        s += QString::number(static_cast<unsigned char>(b.at(i)), 16)
                 .rightJustified(2, QLatin1Char('0')) + QLatin1Char(' ');
    }
    if (b.size() > maxBytes)
        s += QStringLiteral("... (共 %1B)").arg(b.size());
    return s;
}

}  // namespace

bool RtkFrameReader::parse(const QByteArray &body, qint32 expectedDiffId,
                           DogProtocol::AbsPoseCmd *cmd)
{
    QByteArray trimmed = body.trimmed();
    if (trimmed.isEmpty())
        return false;
    // 去掉可能存在的 UTF-8 BOM（部分服务端序列化会带）
    if (trimmed.startsWith("\xEF\xBB\xBF"))
        trimmed = trimmed.mid(3).trimmed();

    DogProtocol::AbsPoseCmd c;
    if (trimmed.startsWith('{') || trimmed.startsWith('['))
    {
        QJsonParseError err;
        const QJsonDocument doc = QJsonDocument::fromJson(trimmed, &err);
        QJsonObject o;
        if (doc.isObject())
            o = doc.object();
        else if (doc.isArray() && doc.array().size() > 0 && doc.array().at(0).isObject())
            o = doc.array().at(0).toObject();   // 兼容"数组包一层"的服务端写法
        if (o.isEmpty())
        {
            qWarning().noquote() << "[Rtk] JSON 解析失败:" << err.errorString()
                                 << "原文:" << QString::fromUtf8(trimmed.left(160));
            return false;
        }

        double x = 0.0, y = 0.0, yawDeg = 0.0, z = 0.0;
        const bool hasX = rtkJsonNumber(o, QStringList() << QStringLiteral("x")
                                                         << QStringLiteral("posX")
                                                         << QStringLiteral("east"), &x);
        const bool hasY = rtkJsonNumber(o, QStringList() << QStringLiteral("y")
                                                         << QStringLiteral("posY")
                                                         << QStringLiteral("south"), &y);
        rtkJsonNumber(o, QStringList() << QStringLiteral("z")
                                       << QStringLiteral("posZ")
                                       << QStringLiteral("alt"), &z);
        const bool hasYaw = rtkJsonNumber(o, QStringList() << QStringLiteral("yaw")
                                                           << QStringLiteral("heading")
                                                           << QStringLiteral("angle")
                                                           << QStringLiteral("azimuth")
                                                           << QStringLiteral("dir")
                                                           << QStringLiteral("航向")
                                                           << QStringLiteral("方向"), &yawDeg);
        if (!hasX || !hasY || !hasYaw)
        {
            qWarning().noquote() << "[Rtk] JSON 缺少必要字段(x/y/yaw)，原文:"
                                 << QString::fromUtf8(trimmed.left(200));
            return false;
        }

        // 差分 ID 为整数；服务端若给字符串数字也能取到，取不到按 0（不校验）
        double idVal = 0.0;
        c.diffId = rtkJsonNumber(o, QStringList() << QStringLiteral("diffId")
                                                  << QStringLiteral("diff_id")
                                                  << QStringLiteral("rtkId")
                                                  << QStringLiteral("deviceId")
                                                  << QStringLiteral("id")
                                                  << QStringLiteral("差分id"), &idVal)
                       ? static_cast<qint32>(idVal) : 0;
        c.x = x;
        c.y = y;
        c.z = z;
        c.yawDeg = yawDeg;
    }
    else
    {
        // 服务端用 QDataStream 序列化 AbsPoseCmd（Qt_5_12，大端），这里按同一版本反序列化
        QDataStream in(body);
        in.setVersion(QDataStream::Qt_5_12);
        in >> c;
        if (in.status() != QDataStream::Ok)
        {
            qWarning().noquote() << "[Rtk] 载荷既非 JSON 也不符合 AbsPoseCmd 格式，原文:"
                                 << QString::fromUtf8(trimmed.left(120));
            return false;
        }
        if (!absPoseSane(c))
        {
            // 解出天文数字说明服务端的结构体布局/字段顺序与本端不一致，
            // 宁可丢弃这一帧，也不能把假值锚进地图与上报坐标
            qWarning().noquote() << "[Rtk] 二进制解析结果不合理，已丢弃 id=" << c.diffId
                                 << "x=" << c.x << "y=" << c.y
                                 << "yaw=" << c.yawDeg << "（字段布局与服务端不一致）";
            return false;
        }
    }

    // 差分 ID 过滤：同一 la 主题下可能有多台设备，只取本机
    if (expectedDiffId != 0 && c.diffId != 0 && c.diffId != expectedDiffId)
    {
        if (!_idWarned)
        {
            _idWarned = true;
            qWarning().noquote() << "[Rtk] 差分ID不匹配，已忽略:" << c.diffId
                                 << "（本机期望" << expectedDiffId << "）";
        }
        return false;
    }

    // 原始报文 + 解析结果：现场核对服务端字段名、单位与字节序，只打前几帧避免刷屏
    if (_rawPrinted < 3)
    {
        ++_rawPrinted;
        const bool looksText = trimmed.startsWith('{') || trimmed.startsWith('[');
        if (looksText)
            qInfo().noquote() << QStringLiteral("[Rtk] 原始报文(%1B): %2")
                .arg(trimmed.size()).arg(QString::fromUtf8(trimmed.left(240)));
        else
            qInfo().noquote() << QStringLiteral("[Rtk] 原始报文 hex(%1B): %2")
                .arg(body.size()).arg(hexDump(body, 64));
        qInfo().noquote() << QStringLiteral("[Rtk] 服务端给值 id=%1 x=%2 y=%3 z=%4 yaw=%5°")
            .arg(c.diffId).arg(c.x, 0, 'f', 3).arg(c.y, 0, 'f', 3)
            .arg(c.z, 0, 'f', 3).arg(c.yawDeg, 0, 'f', 2);
    }

    if (cmd)
        *cmd = c;
    return true;
}
