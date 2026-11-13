#include "DogProtocol.h"
#include <QDateTime>

namespace DogProtocol {

QDataStream &operator<<(QDataStream &s, const MessageHeader &h)
{
    s << h.version << h.msgType << h.terminalId << h.timestampMs;
    return s;
}

QDataStream &operator>>(QDataStream &s, MessageHeader &h)
{
    s >> h.version >> h.msgType >> h.terminalId >> h.timestampMs;
    return s;
}

QDataStream &operator<<(QDataStream &s, const DogState &st)
{
    s << st.dogConnected << st.controlAcquired << st.status
      << st.x << st.y << st.z
      << st.yaw << st.pitch << st.roll
      << st.vWorldX << st.vWorldY << st.vWorldZ
      << st.bodyVx << st.bodyVy << st.bodyVz
      << st.yawRate
      << st.batteryPercent << st.batteryVolt << st.temperature
      << st.functionMode << st.controlMode << st.speedLevel
      << st.faultCount << st.calibrated
      << st.localX << st.localY << st.localYaw;
    return s;
}

QDataStream &operator>>(QDataStream &s, DogState &st)
{
    s >> st.dogConnected >> st.controlAcquired >> st.status
      >> st.x >> st.y >> st.z
      >> st.yaw >> st.pitch >> st.roll
      >> st.vWorldX >> st.vWorldY >> st.vWorldZ
      >> st.bodyVx >> st.bodyVy >> st.bodyVz
      >> st.yawRate
      >> st.batteryPercent >> st.batteryVolt >> st.temperature
      >> st.functionMode >> st.controlMode >> st.speedLevel
      >> st.faultCount >> st.calibrated
      >> st.localX >> st.localY >> st.localYaw;
    return s;
}

QDataStream &operator<<(QDataStream &s, const StateReport &m)
{
    s << m.state;
    return s;
}

QDataStream &operator>>(QDataStream &s, StateReport &m)
{
    s >> m.state;
    return s;
}

QDataStream &operator<<(QDataStream &s, const HeartbeatMsg &m)
{
    s << m.status;
    return s;
}

QDataStream &operator>>(QDataStream &s, HeartbeatMsg &m)
{
    s >> m.status;
    return s;
}

QDataStream &operator<<(QDataStream &s, const AttitudeCmd &m)
{
    s << m.action;
    return s;
}

QDataStream &operator>>(QDataStream &s, AttitudeCmd &m)
{
    s >> m.action;
    return s;
}

QDataStream &operator<<(QDataStream &s, const MoveCmd &m)
{
    s << m.taskId << m.x << m.y << m.maxSpeed << m.arriveRadius << m.timeoutMs;
    return s;
}

QDataStream &operator>>(QDataStream &s, MoveCmd &m)
{
    s >> m.taskId >> m.x >> m.y >> m.maxSpeed >> m.arriveRadius >> m.timeoutMs;
    return s;
}

QDataStream &operator<<(QDataStream &s, const RotateCmd &m)
{
    s << m.taskId << m.angleRad << m.maxTurnRate << m.toleranceRad << m.timeoutMs;
    return s;
}

QDataStream &operator>>(QDataStream &s, RotateCmd &m)
{
    s >> m.taskId >> m.angleRad >> m.maxTurnRate >> m.toleranceRad >> m.timeoutMs;
    return s;
}

QDataStream &operator<<(QDataStream &s, const MapCalibCmd &m)
{
    s << m.action << m.taskId << m.targetX << m.targetY;
    return s;
}

QDataStream &operator>>(QDataStream &s, MapCalibCmd &m)
{
    s >> m.action >> m.taskId >> m.targetX >> m.targetY;
    return s;
}

QDataStream &operator<<(QDataStream &s, const AbsPoseCmd &m)
{
    s << m.diffId << m.x << m.y << m.z << m.yawDeg;
    return s;
}

QDataStream &operator>>(QDataStream &s, AbsPoseCmd &m)
{
    s >> m.diffId >> m.x >> m.y >> m.z >> m.yawDeg;
    return s;
}

QDataStream &operator<<(QDataStream &s, const VelocityCmd &m)
{
    s << m.vx << m.vy << m.yawRate;
    return s;
}

QDataStream &operator>>(QDataStream &s, VelocityCmd &m)
{
    s >> m.vx >> m.vy >> m.yawRate;
    return s;
}

QDataStream &operator<<(QDataStream &s, const StopCmd &m)
{
    s << m.emergency;
    return s;
}

QDataStream &operator>>(QDataStream &s, StopCmd &m)
{
    s >> m.emergency;
    return s;
}

QDataStream &operator<<(QDataStream &s, const CalibrationData &m)
{
    s << m.originX << m.originY << m.originZ << m.originYaw;
    return s;
}

QDataStream &operator>>(QDataStream &s, CalibrationData &m)
{
    s >> m.originX >> m.originY >> m.originZ >> m.originYaw;
    return s;
}

QDataStream &operator<<(QDataStream &s, const ParamCmd &m)
{
    s << m.speedLevel;
    return s;
}

QDataStream &operator>>(QDataStream &s, ParamCmd &m)
{
    s >> m.speedLevel;
    return s;
}

QDataStream &operator<<(QDataStream &s, const TaskFeedback &m)
{
    s << m.taskId << m.result
      << m.currentX << m.currentY << m.remainingDist << m.message;
    return s;
}

QDataStream &operator>>(QDataStream &s, TaskFeedback &m)
{
    s >> m.taskId >> m.result
      >> m.currentX >> m.currentY >> m.remainingDist >> m.message;
    return s;
}

QByteArray packMessage(const MessageHeader &header, const QByteArray &body)
{
    QByteArray raw;
    QDataStream stream(&raw, QIODevice::WriteOnly);
    // 版本固定 Qt_5_12：保证 Qt5.12/5.15 两端字节布局一致
    stream.setVersion(QDataStream::Qt_5_12);
    MessageHeader h = header;
    h.timestampMs = QDateTime::currentMSecsSinceEpoch();
    stream << h;
    raw.append(body);
    return raw;
}

bool unpackMessage(const QByteArray &raw, MessageHeader &header, QByteArray &body)
{
    QDataStream stream(raw);
    stream.setVersion(QDataStream::Qt_5_12);
    stream >> header;
    if (stream.status() != QDataStream::Ok)
        return false;
    if (header.version != PROTOCOL_VERSION)
        return false;
    // 头部之后剩余部分为消息体
    const int headerSize = sizeof(quint8) + sizeof(quint16) + sizeof(quint32) + sizeof(qint64);
    if (raw.size() < headerSize)
        return false;
    body = raw.mid(headerSize);
    return true;
}

double radStd(double rad)
{
    // 归一化到 (-PI, PI]，必须与弧度PI比较
    const double PI = 3.14159265358979323846;
    while (rad > PI)
        rad -= 2.0 * PI;
    while (rad <= -PI)
        rad += 2.0 * PI;
    return rad;
}

}  // namespace DogProtocol
