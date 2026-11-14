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
    double allow = _p.jumpK * v * dt + _p.jumpMinM;
    if (allow < _p.jumpMinM)
        allow = _p.jumpMinM;

    if (step > allow)
    {
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
