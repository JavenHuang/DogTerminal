#ifndef DR_POSE_H
#define DR_POSE_H

#include <QString>
#include <QtGlobal>

// ============================================================
// 抗跳变局部位置（死推算 / Dead Reckoning）
//
// 为什么要有它：
//   狗端 odom 的 position 不是"平滑累积"，而是会整体重置的：
//   run9 实测 8 次单帧跳变 0.63~3.12m（包间隔只有 8~11ms，物理上最多走 9mm），
//   原因是 SDK 会话/控制权重置后局部坐标系重新起算。
//   这个 position 直接进导航 → Pure Pursuit 的方位角 α 瞬间变几十度 →
//   狗原地画圈（run7 14:51 连续 17s forwardJoy=1、|Δpos|=0.00）。
//
// 做法：只积分"合理步长"，跳变帧整帧丢弃。
//   允许步长 = jumpK × |vBody| × dtEf + jumpMinM（下限 jumpMinM）。
//   dtEf 取"自上次位置变化以来的间隔"，不是包间隔：odom.position 是成块
//   刷新的（2026-11-16 实测单帧一步 0.12~0.45m、约 1s 一次），用包间隔会把
//   真实位移判成跳变（行程 0.43m vs 差分 1.22m）。dtEf 上钳到 maxDtSec。
//   跳变帧不改变位姿，也不改变 raw 基准（下一帧继续相对最新 raw 求增量）。
//   断流超过 maxDtSec 时只对齐基准、不积分（避免把 3~5s 的位移当成一步）。
//
// 与 MapTransform 的关系：DrPose 只在**局部坐标系**里累积，
//   局部 → 地图 的换算仍然由 MapTransform 做（refYaw/scale/origin 全不变）。
//   因此地图显示、导航、上报三处坐标口径完全统一，导航代码一行不用改。
// ============================================================
class DrPose
{
public:
    struct Params
    {
        bool   enabled;      // 总开关（[Pose] PosSource=dr 时置 true）
        double jumpK;        // 允许步长系数（默认 3.0）
        double jumpMinM;     // 允许步长下限 m（默认 0.10）
        double maxDtSec;     // 超过该间隔视为断流，只对齐不积分（默认 2.0）

        Params()
            : enabled(true), jumpK(3.0), jumpMinM(0.10), maxDtSec(2.0) {}
    };

    void setParams(const Params &p) { _p = p; }
    const Params &params() const { return _p; }
    bool enabled() const { return _p.enabled; }

    // 直接设定当前位姿（锚定修正、建参考时用）；raw 基准保持不变，
    // 下一帧增量仍相对最新 raw 计算，所以不会产生"重置后再补一步"的错位。
    void reset(double x, double y);

    // 只对齐 raw 基准，不积分、不改位姿（参考重建/坐标系已知重置时用）
    void alignRaw(double rawX, double rawY, qint64 tMs);

    // 每收到一帧原始里程计调用一次。返回 true 表示本帧被接受（已累加）。
    bool feed(double rawX, double rawY, double vBody, qint64 tMs);

    double x() const { return _x; }
    double y() const { return _y; }
    int    rejected() const { return _nRej; }      // 被丢弃的跳变帧数
    double rejectedM() const { return _rejM; }     // 被丢弃的跳变累计米数
    double travel() const { return _travel; }      // 累计接受的位移长度
    void   resetTravel() { _travel = 0.0; }

    QString describe() const;

private:
    Params _p;
    bool   _has = false;
    double _x = 0.0;
    double _y = 0.0;
    double _rawX = 0.0;
    double _rawY = 0.0;
    qint64 _rawMs = 0;
    qint64 _lastChangeMs = 0;   // 上一次"raw 位置真的变了"的时刻（dtEf 用）
    int    _nRej = 0;
    double _rejM = 0.0;
    double _travel = 0.0;
};

#endif  // DR_POSE_H
