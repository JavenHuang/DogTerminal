#ifndef RTK_JUMP_GATE_H
#define RTK_JUMP_GATE_H

#include <QtGlobal>

struct TerminalConfig;
class RtkFusion;

// ============================================================
// 差分单帧跳变门控
//
// 只做一件事：给定本帧差分位姿与"上一帧被接受的基准"，判这一帧该不该进融合。
// 位移超限 → 野值丢弃；航向超限时再分三种成因：
//   A 狗真转了（里程计同期同号同量级）→ 接受，且绝不清零位；
//   B 狗没转、差分航向连跳同一角度簇 ≥600ms → 判模块重启，重取零位后接受；
//   C 狗转了但差分反向/量级差 >60° → 轴向或符号错，丢弃并告警。
// 只判、不改变位姿基准；基准由 RtkPoseTracker 在本帧被接受后更新。
// ============================================================
class RtkJumpGate
{
public:
    RtkJumpGate(const TerminalConfig &cfg, RtkFusion &fusion);

    // 返回 false 表示本帧应丢弃（调用方直接 return）
    bool accept(double cx, double cy, double yawRad, qint64 recvMs,
                double baseX, double baseY, double baseYawRad,
                bool hasOdom, double omyawRad);

    // 本帧被接受后记录里程计地图 yaw，作为下一帧判"真转向/假重启"的基准
    void noteAccepted(bool hasOdom, double omyawRad);

    // 地图参考变更：旧参考下的 yaw 基准全部作废，重启判据从头开始
    void reset();

    quint32 badJumps() const { return _badJumps; }
    int restarts() const { return _restarts; }

private:
    const TerminalConfig &_cfg;
    RtkFusion &_fusion;

    int    _yawJumpRun = 0;          // 同一角度簇已连续多少帧
    qint64 _jumpSinceMs = 0;         // 该角度簇起始时刻（0 = 未开始）
    bool   _rejectYawValid = false;  // 上一次被拒的航向是否可比较
    double _rejectYawRad = 0.0;
    quint32 _badJumps = 0;
    int _restarts = 0;
    bool _rebaseExhausted = false;   // 零位重取已达上限：此后航向跳变只丢弃，不再改写基准

    double _odomYawRad = 0.0;        // 最近一次被接受帧的里程计地图 yaw
    bool   _odomYawValid = false;
};

#endif  // RTK_JUMP_GATE_H
