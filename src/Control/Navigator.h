#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <QObject>
#include "Protocol/DogProtocol.h"

// ============================================================
// 点到点导航控制器（机动控制）
// 策略（已确认）：
//   1. 转向对准阶段：原地转向使机头对准目标航向（±容差）+ 最小前进量
//      （MATRiX walk 在 forward=0 时不执行转向，必须边走边转）
//   2. 直行阶段：固定 maxSpeed 巡航，两种速度策略可切换（见 SpeedMode）
//   3. 软到达：dist ≤ SoftArriveRadiusMul × arriveRadius 发 MR_ARRIVED
//      feedback 但不停车，给服务器留出无缝下发下一个航点的时间窗
//   4. 硬到达：dist ≤ arriveRadius 必停车（即使已发过软到达 feedback）
//   5. 超时/跑飞 硬到达（finishHard + stopRequested）
// 输出为归一化遥控量(joystick)：forward/turn ∈ [-1,1]
// ============================================================
class Navigator : public QObject
{
    Q_OBJECT
public:
    // 速度策略：两种可切换模式，通过 ini 配置 Nav/SpeedMode 选择
    // - Cruise：匀速巡航，vDes 直接硬切 maxSpeed；收油段仍启用（防过冲）
    //   起步猛冲、收油靠 MATRiX 原生
    // - Profile：三段式，起步加速度过渡 → 匀速巡航 → 收油减速度约束减速
    //   更像真车，起步不猛冲；收油段物理约束下减速，避免冲过终点反向大转向
    // 【强制】两种模式都启用收油衰减（stopArriveRadiusMul），否则
    // 快速巡航到终点狗冲过目标后目标方向反向、大转向、站不稳
    enum SpeedMode
    {
        SpeedMode_Cruise  = 0,
        SpeedMode_Profile = 1
    };

    // 转向律：腿足平台"原地转向"是最大的打滑源（实测里程计单帧跳 1~2m），
    // 所以默认走 PurePursuit —— 不停车、边走边转、连续曲率跟踪。
    // AlignThenGo 保留用于回归对比。
    enum SteeringMode
    {
        Steering_AlignThenGo = 0,
        Steering_PurePursuit = 1
    };

    struct Params
    {
        // 速度策略选择（0=Cruise 匀速硬切，1=Profile 三段式）
        SpeedMode speedMode;
        double headingToleranceRad;   // 对准容差 rad
        double reAlignThresholdRad;   // 超出此误差重新对准 rad
        double kpAlign;               // 转向P增益（输出 rad/s）
        // 转向D增益：用可信的 yawRate 做阻尼，抑制真狗约 38° 的停转惯性过冲
        // 输出 = Kp*err - Kd*yawRate
        double kdAlign;
        // 接近对准容差时的"减速带"：|err| < alignSlowBandRad 时把转向角速度
        // 限制到 alignSlowMaxTurnRate 以内，让惯性收敛在容差里而不是冲过去反向
        double alignSlowBandRad;
        double alignSlowMaxTurnRate;
        double kpHoldYaw;             // 直行中航向保持增益
        double kpDist;                // 距离→期望速度增益 (1/s)（当前未使用，保留）
        double kpVel;                 // 速度闭环P增益（输出 m/s 修正）
        double maxTurnRate;           // 最大转向角速度 rad/s
        double maxLinearSpeed;        // 最大线速度 m/s（终端上限）
        double joyLinearScale;        // joystick=1.0 对应的线速度 m/s
        double joyTurnScale;          // joystick=1.0 对应的角速度 rad/s
        bool allowBackward;           // 允许倒退接近
        // MATRiX walk 需 forward≠0 才执行转向，对准阶段的最小前进速度
        double alignCreepSpeed;       // 对准阶段最小前进速度 m/s（默认 0.25）
        // MATRiX/UE 摇杆轴死区：|axis|<0.2 的输入被仿真器直接丢弃
        double minAxisOut;            // 最小有效摇杆量（默认 0.22）
        // 收油段（两种模式都强制启用）
        double stopArriveRadiusMul;   // 到达半径多少倍内开始 vDes 线性衰减（默认 6.0）
        double minCreepSpeed;         // 收油段衰减到的最小速度 m/s（默认 0.25）
        double stopBodyVxDeadzone;    // 判定"狗已静止"的机体系前向速度阈值 m/s
        double holdYawOutDeadzoneRad; // 航向保持输出死区：航向误差小于此值直接输出 0
        double arriveRadiusDefault;   // 服务器未下发 arriveRadius 时的默认到达半径 m
        // 软到达半径倍数：dist <= mul × arriveRadius 时先发一次 MR_ARRIVED
        // feedback 但继续走，给服务器留出无缝下发下一个航点的时间窗；
        // 真走到 arriveRadius 则硬到达停车。mul <= 1 表示关闭软到达。
        double softArriveRadiusMul;

        // Profile 模式专用（Cruise 模式忽略）
        double maxAccel;              // 最大加速度 m/s²（默认 1.0，起步 2m/s 需 2s）
        double maxDecel;              // 最大减速度 m/s²（默认 2.0，收油比加速快）

        // ---- Pure Pursuit 转向律 ----
        SteeringMode steeringMode;    // 0=AlignThenGo(旧) 1=PurePursuit(默认)
        double ppLookaheadMin;        // 前视距离下限 m（必须 > 到达半径，否则曲率病态）
        double ppLookaheadMax;        // 前视距离上限 m
        double ppLookaheadK;          // 前视距离 = K * 巡航速度（秒）

        // ---- 原地空转脱困 ----
        // 曲率律假设狗按指令速度前进；狗一旦原地打滑（真实 bodyVx≈0 而
        // turnRate 很大），方位角不变、指令不变，就会一直原地画圈。
        // 判据：|turnRate|>=ppStuckTurnMin 且 期望前进速度>=ppStuckFwdMin 且
        //       |bodyVx|<=ppStuckVxMax，持续 ppStuckSec 秒即判定空转，
        //       改走"前进有效值归零 + 方向饱和转向"的对准段，直到朝向误差
        //       回到 ppStuckExitDeg 以内再交还 Pure Pursuit。
        double ppStuckSec;            // 判定空转所需持续秒数（<=0 关闭）
        double ppStuckTurnMin;        // 触发空转的转向角速率门限 rad/s
        double ppStuckFwdMin;         // 触发空转的期望前进速度门限 m/s
        double ppStuckVxMax;          // 触发空转的实际前向速度上限 m/s
        double ppStuckExitDeg;        // 退出对准段的朝向误差（度）

        // ---- 速度闭环：摇杆归一化值 → 实际 m/s 的自适应增益 ----
        // SetRemote 传的是归一化量，"1.0 摇杆 = 多少 m/s"随地面材质、
        // 电量、速度档位漂移。开启后用实测 bodyVx 反算真实增益并慢速跟随，
        // 替代固定的 joyLinearScale，避免水泥地冲过头 / 地毯上跑不动。
        bool joyGainAdaptEnable;
        double joyGainMin;            // 自适应下限 m/s per joy
        double joyGainMax;            // 自适应上限 m/s per joy
        double joyGainTau;            // 一阶低通系数（越小越稳、越大越快）

        // 诊断日志：5Hz 打印 [NvDbg] 行（默认关闭，排查航向/坐标一致性时打开）
        bool debugLog;

        Params()
            : speedMode(SpeedMode_Cruise)
            , headingToleranceRad(20.0 * 3.14159265358979323846 / 180.0)
            , reAlignThresholdRad(30.0 * 3.14159265358979323846 / 180.0)
            , kpAlign(1.5)
            , kdAlign(0.4)
            , alignSlowBandRad(25.0 * 3.14159265358979323846 / 180.0)
            , alignSlowMaxTurnRate(0.4)
            , kpHoldYaw(1.0)
            , kpDist(0.6)
            , kpVel(0.2)
            , maxTurnRate(1.2)
            , maxLinearSpeed(0.8)
            , joyLinearScale(1.0)
            , joyTurnScale(1.5)
            , allowBackward(true)
            , alignCreepSpeed(0.25)
            , minAxisOut(0.22)
            , stopArriveRadiusMul(6.0)
            , minCreepSpeed(0.25)
            , stopBodyVxDeadzone(0.05)
            , holdYawOutDeadzoneRad(5.0 * 3.14159265358979323846 / 180.0)
            , arriveRadiusDefault(1.5)
            , softArriveRadiusMul(2.0)
            , maxAccel(1.0)
            , maxDecel(2.0)
            , steeringMode(Steering_PurePursuit)
            , ppLookaheadMin(0.6)
            , ppLookaheadMax(1.5)
            , ppLookaheadK(0.5)
            , ppStuckSec(4.0)
            , ppStuckTurnMin(0.30)
            , ppStuckFwdMin(0.25)
            , ppStuckVxMax(0.08)
            , ppStuckExitDeg(25.0)
            , joyGainAdaptEnable(true)
            , joyGainMin(0.3)
            , joyGainMax(3.0)
            , joyGainTau(0.05)
            , debugLog(false) {}
    };

    explicit Navigator(QObject *parent = nullptr);

    void setParams(const Params &p) { _params = p; }
    const Params &params() const { return _params; }

    // 启动导航任务（目标为狗局部坐标）
    void start(quint32 taskId, double localX, double localY,
               double maxSpeed, double arriveRadius, qint64 timeoutMs);
    // 启动原地旋转：absolute=false 时 angleRad 为相对增量（正左负右），
    // absolute=true 时 angleRad 为狗局部坐标系的目标 yaw。
    void startRotate(quint32 taskId, double currentYaw, double angleRad, bool absolute,
                     double maxTurnRate, double toleranceRad, qint64 timeoutMs);
    void cancel();  // 取消当前任务（发 MR_FAILED + stopRequested，硬停车）
    bool isActive() const { return _phase != Phase_Idle; }
    bool isRotationActive() const { return _phase == Phase_Rotate; }
    // 软到达：已发过一次 MR_ARRIVED，仍在前进等下一个任务。
    // 抢占这种任务时不再补发 MR_CANCELLED，避免同一任务出现两次结束事件。
    bool isSoftArrived() const { return _phase == Phase_SoftArrived; }
    quint32 taskId() const { return _taskId; }
    double remainingDist() const { return _remainingDist; }

public slots:
    // 由主控制器按狗状态更新周期调用（局部坐标）
    // localYaw 直接采用原始 imu.rpy[2]；yawRate 仅作转向闭环阻尼项。
    void update(double localX, double localY, double localYaw,
                double bodyVx, double yawRate);

signals:
    // 遥控量输出（joystick 归一化值）
    void remoteCommand(double forwardJoy, double turnJoy);
    // 请求停车（零速遥控）
    void stopRequested();
    // 任务结束（含到达/超时；取消与失败由外部处理）
    void finished(quint32 taskId, DogProtocol::MoveResult result,
                  double remainingDist, const QString &message);

private:
    enum Phase
    {
        Phase_Idle,
        Phase_Rotate,       // 原地旋转到相对/绝对航向
        Phase_Align,        // 转向对准
        Phase_Move,         // 直行巡航
        Phase_SoftArrived   // 已发 MR_ARRIVED feedback，继续前进等下一个任务抢占
                            // （等待新任务抢占 start() 把 phase 重置）
    };

    double clamp(double v, double lo, double hi) const;
    double applyAxisDeadzone(double v) const;
    // 原地旋转闭环：只根据原始 yaw 计算任务误差，不修改测量值。
    void updateRotation(double localYaw, double yawRate, qint64 now);
    // Pure Pursuit：由目标向量直接算出连续 (速度, 角速度) 指令。
    // ω = 2·v·sin(α)/Ld，并用可达曲率 ωmax 反解线速度上限，
    // 保证"转不过来时先减速"而不是满速画大圈。
    void computePurePursuit(double dx, double dy, double yaw,
                            double dist, double &vDesRaw, double &turnRate);
    // 收油衰减：进入 stopArriveRadiusMul × arriveRadius 后线性降速，
    // 保留符号（倒车时也是往 0 收），并保证不低于 minCreepSpeed。
    void applyArriveDecay(double &vDes, double dist) const;
    // 统一的速度平滑出口：两种速度模式走这里做最终 vDes 处理
    // Cruise 模式：直接返回 vDesRaw（硬切，不加减速约束）
    // Profile 模式：用 maxAccel/maxDecel * dt 限制每帧变化量
    double smoothVDes(double vDesRaw, qint64 now);
    // 硬到达：Phase_Idle + emit stopRequested + emit finished（触发停车链路）
    void finishHard(DogProtocol::MoveResult result, const QString &message);
    // 软到达：保持 Phase_SoftArrived、继续前进、只 emit finished（仅发 feedback）
    void finishSoft(DogProtocol::MoveResult result, const QString &message);

    Params _params;
    Phase _phase;
    quint32 _taskId;
    double _targetX;
    double _targetY;
    double _maxSpeed;
    double _arriveRadius;
    qint64 _timeoutMs;
    qint64 _startMs;
    double _remainingDist;

    // 原地旋转任务状态：相对模式累计原始 yaw 的帧间最短差，支持跨 ±π 和多圈旋转。
    bool _rotateAbsolute = false;
    double _rotateTargetYaw = 0.0;
    double _rotateDelta = 0.0;
    double _rotateAccum = 0.0;
    double _rotatePrevRawYaw = 0.0;
    double _rotateMaxTurnRate = 0.0;
    double _rotateTolerance = 0.0;

    // 防跑飞：起始距离 + 持续计时（方向判反时无此保护会满速跑出几十米）
    double _startDist = 0.0;
    qint64 _escapeSinceMs = 0;
    bool _softArrivedReported = false;  // 防止软到达后穿出/再穿入重复发 feedback

    // Profile 模式专用：加速度/减速度限制状态
    double _vDesPrev = 0.0;
    qint64 _lastUpdateMs = 0;

    // 摇杆增益自适应状态：_joyGain = 归一化 1.0 摇杆实际对应的 m/s
    double _joyGain = 1.0;
    bool _joyGainInit = false;   // 首次启动时用 JoyLinearScale 作初值
    double _lastFwdJoy = 0.0;  // 上一帧实际下发的归一化前进量（用于反算）

    // 诊断日志限流（5Hz）：定位"目标在前方却转向"时打印坐标/航向基准
    qint64 _dbgLastMs = 0;
    // 大角度掉头方向锁：±1=锁定的转向方向，0=未锁定。
    // yawErr 在 ±π 处翻符号会让 P 控制来回反向，锁定后误差连续展开
    int _turnDirLock = 0;

    // 原地空转脱困：空转计时 + 是否处于"强制对准"段 + 已打印过提示
    qint64 _stuckSinceMs = 0;
    qint64 _stuckAlignSinceMs = 0;
    bool   _stuckAlign = false;
};

#endif  // NAVIGATOR_H
