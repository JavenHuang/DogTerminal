// ============================================================
// 抗跳变局部位置（死推算）—— 实现
// 只做一件事：把原始 odom.position 的"合理增量"累加成连续位姿。
// 不含任何地图/朝向逻辑，换算仍在 MapTransform。
// ============================================================
#include "Convert/DrPose.h"

#include <cmath>

void DrPose::reset(double x, double y)
{
    _x = x;
    _y = y;
    _has = true;   // 位姿已有效；raw 基准若尚未建立则由 feed() 首帧补上
}

void DrPose::alignRaw(double rawX, double rawY, qint64 tMs)
{
    _rawX = rawX;
    _rawY = rawY;
    _rawMs = tMs;
}

bool DrPose::feed(double rawX, double rawY, double vBody, qint64 tMs)
{
    // 首帧：没有历史基准，只能记录；位姿保持调用方已设定的值（默认 0,0）
    if (!_has)
    {
        _has = true;
        _x = rawX;
        _y = rawY;
        alignRaw(rawX, rawY, tMs);
        return false;
    }

    // 上一帧还没进过（例如 alignRaw 之后首次）：只对齐
    if (_rawMs == 0)
    {
        alignRaw(rawX, rawY, tMs);
        return false;
    }

    const double dx = rawX - _rawX;
    const double dy = rawY - _rawY;
    const qint64 dtMs = tMs - _rawMs;
    const double dt = static_cast<double>(dtMs) / 1000.0;

    // 基准先推进：无论本帧是否采信，下一帧都相对最新 raw 求增量，
    // 这样一次跳变只影响本帧（被丢弃），不会污染后续所有帧。
    alignRaw(rawX, rawY, tMs);

    if (dtMs <= 0)
        return false;                      // 同一时刻重复帧
    if (dt > _p.maxDtSec)
        return false;                      // 断流/暂停：这一段位移无从核对，不积分

    const double step = std::sqrt(dx * dx + dy * dy);
    const double v = (std::fabs(vBody) < 5.0) ? std::fabs(vBody) : 5.0;

    // 判据的时间基准必须取"自上次位置变化以来的间隔"，不是包间隔。
    // 2026-11-16 现场实测：odom.position 是成块刷新的（同一值重复若干包，
    // 真正变化那一帧一步 0.12~0.45m，约 1s 一次），而包间隔只有 8~11ms。
    // 用包间隔算 allow≈0.11m，于是"真实位移"整帧被判成跳变丢掉：
    // 行程显示 0.43m，而差分/原始 odom 都是 1.1~1.2m，死推算也因此少走 0.72m。
    const bool changed = (dx != 0.0 || dy != 0.0);
    const qint64 sinceMs = (changed && _lastChangeMs > 0) ? (tMs - _lastChangeMs) : dtMs;
    if (changed)
        _lastChangeMs = tMs;
    double dtEf = static_cast<double>(sinceMs) / 1000.0;
    if (dtEf < dt)
        dtEf = dt;                         // 位置每包都变时退回包间隔
    if (dtEf > _p.maxDtSec)
        dtEf = _p.maxDtSec;                // 长时间无变化后的一次大步：按断流上限估

    double allow = _p.jumpK * v * dtEf + _p.jumpMinM;
    if (allow < _p.jumpMinM)
        allow = _p.jumpMinM;

    if (step > allow)
    {
        // 只剩"物理上不可能"的步长会走到这里：静止时挪一步，或走动中
        // 一跳超出 maxDtSec×速度上限。成块刷新的真实位移已被 dtEf 放过。
        ++_nRej;
        _rejM += step;
        return false;                      // 跳变帧：整帧丢弃
    }

    _x += dx;
    _y += dy;
    _travel += step;
    return true;
}

QString DrPose::describe() const
{
    return QStringLiteral("dr=(%1, %2) 累计%3m 剔除跳变%4帧/%5m")
        .arg(_x, 0, 'f', 2)
        .arg(_y, 0, 'f', 2)
        .arg(_travel, 0, 'f', 2)
        .arg(_nRej)
        .arg(_rejM, 0, 'f', 2);
}
