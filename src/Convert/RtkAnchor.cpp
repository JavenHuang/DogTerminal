// ============================================================
// 差分静置锚点 —— 实现
// 窗口取中值（不是均值）：格点量化误差是台阶分布，中值对野值不敏感，
// 且 15 帧中值的标准差约为单帧的 1/√15（若误差近独立）。
// ============================================================
#include "Convert/RtkAnchor.h"

#include <algorithm>
#include <cmath>

void RtkAnchor::clearWindow()
{
    _wx.clear();
    _wy.clear();
}

void RtkAnchor::onFrame(double x, double y, double speedMps, double yawRateRad, qint64 tMs)
{
    _lastMs = tMs;

    if (!_p.enabled)
        return;
    if (_p.windowFrames < 3)
        return;                          // 少于 3 帧的中值没有意义

    const bool still = (std::fabs(speedMps) < _p.maxSpeedMps)
                    && (std::fabs(yawRateRad) < _p.maxYawRateRad);

    if (!still)
    {
        // 动了：清窗口、重新计时、重新武装
        _stillSinceMs = 0;
        _armed = true;
        clearWindow();
        return;
    }

    if (!_armed)
        return;                          // 本次停稳已经产过点，等下次移动

    if (_stillSinceMs == 0)
        _stillSinceMs = tMs;

    // 从"刚开始停稳"就往窗口里攒，而不是等够 minStillMs 才开始攒：
    // 差分实测 3.8Hz（周期 263ms），窗口 12~15 帧本身就要 3.2~3.9s，
    // 若再叠加 1.2s 的等待门槛，现场每到一点要站 5s 以上，节奏撑不住。
    // 这些帧确实都是"已经停稳后"的帧（_stillSinceMs 之前的运动帧没有进窗）。
    _wx.append(x);
    _wy.append(y);
    while (_wx.size() > _p.windowFrames)
    {
        _wx.removeFirst();
        _wy.removeFirst();
    }
    if ((tMs - _stillSinceMs) < _p.minStillMs)
        return;                          // 还没停够，继续攒
    if (_wx.size() < _p.windowFrames)
        return;                          // 窗口未满，继续攒

    // 中值：排序后取中间（偶数取两中间的平均）
    QVector<double> sx = _wx;
    QVector<double> sy = _wy;
    std::sort(sx.begin(), sx.end());
    std::sort(sy.begin(), sy.end());
    const int n = sx.size();
    const double mx = (n % 2 == 1) ? sx[n / 2]
                                   : 0.5 * (sx[n / 2 - 1] + sx[n / 2]);
    const double my = (n % 2 == 1) ? sy[n / 2]
                                   : 0.5 * (sy[n / 2 - 1] + sy[n / 2]);

    // 散布：各帧到中值的最大距离（欧氏），用来判"格点跳了 / 其实在挪"
    double spread = 0.0;
    for (int i = 0; i < n; ++i)
    {
        const double dx = _wx[i] - mx;
        const double dy = _wy[i] - my;
        const double d = std::sqrt(dx * dx + dy * dy);
        if (d > spread)
            spread = d;
    }

    if (_p.maxSpreadM > 0.0 && spread > _p.maxSpreadM)
    {
        ++_rejSpread;
        clearWindow();
        _stillSinceMs = tMs;             // 重新计时，避免连续判失败
        return;
    }

    _rx = mx;
    _ry = my;
    _rn = n;
    _rspread = spread;
    _ready = true;
    ++_produced;
    _armed = false;                      // 一次停稳只产一个点
    clearWindow();
}

bool RtkAnchor::take(double *x, double *y, int *frames, double *spread)
{
    if (!_ready)
        return false;
    _ready = false;
    if (x) *x = _rx;
    if (y) *y = _ry;
    if (frames) *frames = _rn;
    if (spread) *spread = _rspread;
    return true;
}

bool RtkAnchor::freshAt(qint64 tMs) const
{
    if (_lastMs <= 0)
        return false;
    if (_p.staleMs <= 0)
        return true;
    return (tMs - _lastMs) <= static_cast<qint64>(_p.staleMs);
}

QString RtkAnchor::describe() const
{
    // 带实时状态：现场要能看出"为什么不锚"——
    // 是没停稳、窗口没攒满、还是散布超限被丢了
    const qint64 stillMs = (_stillSinceMs > 0 && _lastMs > 0) ? (_lastMs - _stillSinceMs) : 0;
    return QStringLiteral("产点%1个 弃窗%2次 窗口%3/%4帧 停稳%5/门槛%6ms 散布上限%7m")
        .arg(_produced)
        .arg(_rejSpread)
        .arg(_wx.size())
        .arg(_p.windowFrames)
        .arg(stillMs)
        .arg(_p.minStillMs)
        .arg(_p.maxSpreadM, 0, 'f', 2);
}
