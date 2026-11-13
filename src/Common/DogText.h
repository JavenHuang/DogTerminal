#ifndef DOG_TEXT_H
#define DOG_TEXT_H

// ============================================================
// 文本化与角度归一化（跨编译单元共用的小工具）
//
// 这些函数原本在 MainController*.cpp 里各写一份（拆分时整段复制），
// 既重复又触发 -Wunused-function。这里统一成 inline：用到才展开，
// 没用到不产生告警，也不必再为每个 TU 维护一份副本。
//
// 只放"无状态、无副作用"的纯函数；带状态的工具不要往这里加。
// ============================================================

#include <QtGlobal>
#include <QDateTime>
#include <QString>
#include <cmath>

// 单调毫秒时间戳（各处节流/超时判断共用）
inline qint64 nowMs()
{
    return QDateTime::currentMSecsSinceEpoch();
}

// 弧度 → 度
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

// feedback 枚举转可读文本；固件实际取值未完全确认，故打印时同时保留数值便于核对
inline QString functionModeText(int v)
{
    switch (v)
    {
    case 1: return QStringLiteral("remote");
    case 2: return QStringLiteral("trace");
    case 3: return QStringLiteral("sdk");
    case 4: return QStringLiteral("general_sdk");
    case 5: return QStringLiteral("roamer");
    default: return QStringLiteral("unknown");
    }
}

inline QString controlModeText(int v)
{
    switch (v)
    {
    case 1: return QStringLiteral("stand_up");
    case 2: return QStringLiteral("stand_down");
    case 3: return QStringLiteral("lock");
    case 4: return QStringLiteral("emergency");
    case 5: return QStringLiteral("move");
    case 6: return QStringLiteral("stay");
    case 7: return QStringLiteral("init");
    default: return QStringLiteral("unknown");
    }
}

inline QString motionModeText(int v)
{
    switch (v)
    {
    case 1: return QStringLiteral("running");
    case 2: return QStringLiteral("rest");
    case 3: return QStringLiteral("forbid");
    default: return QStringLiteral("unknown");
    }
}

inline QString speedLevelText(int v)
{
    switch (v)
    {
    case 1: return QStringLiteral("low");
    case 2: return QStringLiteral("mid");
    case 3: return QStringLiteral("high");
    default: return QStringLiteral("unknown");
    }
}

// 路由键：终端只发这三个上行主题（下行/检测/标定各有来源，不在此列）
inline QString routingKeyState(quint32 id)     { return QStringLiteral("state.%1").arg(id); }
inline QString routingKeyHeartbeat(quint32 id) { return QStringLiteral("heartbeat.%1").arg(id); }
inline QString routingKeyFeedback(quint32 id)  { return QStringLiteral("feedback.%1").arg(id); }

// 角度归一化到 [0, 360)（地图角约定：0=正东、顺时针为正）
inline double degNorm360(double deg)
{
    deg = std::fmod(deg, 360.0);
    if (deg < 0.0)
        deg += 360.0;
    // 控制台按 0.1° 打印：359.95° 以上四舍五入后会显示成 360.0°，这里折回 0
    deg = std::round(deg * 10.0) / 10.0;
    if (deg >= 360.0)
        deg = 0.0;
    return deg;
}

#endif  // DOG_TEXT_H
