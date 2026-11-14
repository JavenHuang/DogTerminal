#ifndef RTK_ANCHOR_H
#define RTK_ANCHOR_H

#include <QString>
#include <QVector>
#include <QtGlobal>

// ============================================================
// 差分静置锚点（只做锚定，不做位姿流）
//
// 为什么只做静置锚点：
//   差分输出被量化在 0.61×0.42m 格点上（九点闭式复算 k=1.083、RMS=0.298m，
//   半格量化下限 √((0.61²+0.42²)/12)≈0.214m）。把它当高频位姿源，位置会
//   一格一格地台阶跳；动态卡尔曼也压不住，因为台阶不服从高斯分布。
//   但在**同一位置静止重复采样**时，量化误差零均值，取窗口中值即可把
//   标准差压到 0.05~0.1m —— 这是突破格点精度的唯一办法。
//
// 产出规则（缺一不可）：
//   1) 狗确实停稳：|vBody| < maxSpeedMps 且 |yawRate| < maxYawRateRad，
//      持续 minStillMs（默认 1.2s，等差分那 200~560ms 滞后把"停稳位置"送全）；
//   2) 窗口满 windowFrames 帧（默认 15 帧 ≈ 3~7s @ 2~5Hz）；
//   3) 窗口内散布 max|v-中值| ≤ maxSpreadM（默认 0.30m，超限说明格点跳了，
//      或者狗其实在挪，整窗丢弃、重新计时）；
//   4) 一次停稳只产出一个锚点：产出后需重新检测到"动过"，才再武装。
//
// 本类只负责"产点"，不碰位姿；把锚点换算进地图/局部、决定要不要修正，
// 由调用方（MainControllerPose）做，避免锚定逻辑与位姿源耦合。
// ============================================================
class RtkAnchor
{
public:
    struct Params
    {
        bool   enabled;        // [Rtk] AnchorMode=rest 时置 true
        int    windowFrames;   // 取中值的静止窗口帧数（默认 15）
        double maxSpreadM;     // 窗口内允许的最大散布 m（默认 0.45）
        int    minStillMs;     // 停稳持续多久才开始取窗 ms（默认 1200）
        double maxSpeedMps;    // 判"停稳"的速度上限 m/s（默认 0.05）
        double maxYawRateRad;  // 判"停稳"的角速度上限 rad/s（默认 5°/s）
        int    staleMs;        // 锚点新鲜度上限 ms（默认 1500，超时不采信）

        Params()
            : enabled(false)
            , windowFrames(15)
            , maxSpreadM(0.45)
            , minStillMs(1200)
            , maxSpeedMps(0.05)
            , maxYawRateRad(5.0 * 3.14159265358979323846 / 180.0)
            , staleMs(1500) {}
    };

    void setParams(const Params &p) { _p = p; }
    const Params &params() const { return _p; }
    bool enabled() const { return _p.enabled; }

    // 每收到一帧"通过跳变门控"的差分帧调用一次。
    // speedMps / yawRateRad 取终端自己的运动状态（不是差分给的）。
    void onFrame(double x, double y, double speedMps, double yawRateRad, qint64 tMs);

    // 取出一个已就绪的锚点（取走即清空）。返回 false 表示当前没有新锚点。
    bool take(double *x, double *y, int *frames, double *spread);

    qint64 lastFrameMs() const { return _lastMs; }
    bool   freshAt(qint64 tMs) const;
    int    produced() const { return _produced; }   // 累计产出锚点数（统计）
    int    rejectedSpread() const { return _rejSpread; }
    QString describe() const;

private:
    void clearWindow();

    Params _p;
    QVector<double> _wx;
    QVector<double> _wy;
    qint64 _stillSinceMs = 0;
    qint64 _lastMs = 0;
    bool   _armed = true;      // 一次停稳只产一个点：产出后等"动过"再武装
    bool   _ready = false;
    double _rx = 0.0;
    double _ry = 0.0;
    int    _rn = 0;
    double _rspread = 0.0;
    int    _produced = 0;
    int    _rejSpread = 0;
};

#endif  // RTK_ANCHOR_H
