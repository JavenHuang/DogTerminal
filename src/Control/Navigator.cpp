#include "Navigator.h"

#include <QDateTime>
#include <QDebug>
#include <cmath>

using DogProtocol::radStd;

static const double PI = 3.14159265358979323846;

Navigator::Navigator(QObject *parent)
    : QObject(parent)
    , _phase(Phase_Idle)
    , _taskId(0)
    , _targetX(0)
    , _targetY(0)
    , _maxSpeed(0)
    , _arriveRadius(0)
    , _timeoutMs(0)
    , _startMs(0)
    , _remainingDist(0)
{
}

double Navigator::clamp(double v, double lo, double hi) const
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

double Navigator::applyAxisDeadzone(double v) const
{
    // v==0 表示控制器确实不想动（停车/到位），必须保持 0，不能抬成最小量
    if (std::fabs(v) < 1e-4)
        return 0.0;
    if (std::fabs(v) >= _params.minAxisOut)
        return v;
    return v > 0.0 ? _params.minAxisOut : -_params.minAxisOut;
}

// ============================================================
// 统一速度平滑出口：两种模式只在这里区分
// - Cruise：硬切，直接返回 vDesRaw；更新 _vDesPrev/时间戳让 Profile 模式不炸
// - Profile：每帧变化量不超过 maxAccel * dt（加速）或 maxDecel * dt（减速）
//   起步从 0 逐步加速到 vDesRaw，收油逐步衰减
// ============================================================
double Navigator::smoothVDes(double vDesRaw, qint64 now)
{
    double dt = 0.05;  // 第一帧或时间戳异常时默认 30Hz 周期
    if (_lastUpdateMs > 0)
    {
        dt = (now - _lastUpdateMs) / 1000.0;
        if (dt < 0.01) dt = 0.05;
        if (dt > 1.0)  dt = 1.0;
    }
    _lastUpdateMs = now;

    double vDesSmoothed;
    if (_params.speedMode == SpeedMode_Cruise)
    {
        // 匀速硬切：vDesRaw 想变多少就变多少
        vDesSmoothed = vDesRaw;
    }
    else  // SpeedMode_Profile
    {
        // 三段式：加速用 maxAccel，减速用 maxDecel（收油比加速快）
        const double absDelta = (vDesRaw > _vDesPrev)
                                    ? _params.maxAccel * dt
                                    : _params.maxDecel * dt;
        vDesSmoothed = _vDesPrev + clamp(vDesRaw - _vDesPrev, -absDelta, absDelta);
    }
    _vDesPrev = vDesSmoothed;
    return vDesSmoothed;
}

void Navigator::start(quint32 taskId, double localX, double localY,
                      double maxSpeed, double arriveRadius, qint64 timeoutMs)
{
    _taskId = taskId;
    _targetX = localX;
    _targetY = localY;
    _maxSpeed = (maxSpeed > 0 && maxSpeed < _params.maxLinearSpeed)
                    ? maxSpeed : _params.maxLinearSpeed;
    _arriveRadius = arriveRadius > 0 ? arriveRadius : _params.arriveRadiusDefault;
    _timeoutMs = timeoutMs;
    _startMs = QDateTime::currentMSecsSinceEpoch();
    _remainingDist = 0;
    _startDist = 0.0;
    _escapeSinceMs = 0;
    _softArrivedReported = false;
    // 状态机收敛：新任务抢占时统一重置平滑器缓存，避免上一任务的
    // 加减速状态残留到新方向上（典型 bug：反向大转向瞬间速度仍沿旧方向）
    _vDesPrev = 0.0;
    _lastUpdateMs = 0;
    // 摇杆增益：首次启动用配置值作初值，之后跨任务保留学到的值（平台属性）
    if (!_joyGainInit)
    {
        _joyGain = _params.joyLinearScale > 1e-3 ? _params.joyLinearScale : 1.0;
        _joyGainInit = true;
    }
    _lastFwdJoy = 0.0;
    _turnDirLock = 0;
    _stuckSinceMs = 0;
    _stuckAlign = false;
    _phase = Phase_Align;  // 先对准
    qInfo() << "[Navigator] 启动任务" << taskId
            << QString("目标(%1, %2) 限速%3 到达半径%4 超时%5ms SpeedMode=%6 SteeringMode=%7")
               .arg(localX, 0, 'f', 2).arg(localY, 0, 'f', 2)
               .arg(_maxSpeed, 0, 'f', 2).arg(_arriveRadius, 0, 'f', 2)
               .arg(_timeoutMs).arg(_params.speedMode == SpeedMode_Cruise ? "Cruise" : "Profile")
               .arg(_params.steeringMode == Steering_PurePursuit ? "PurePursuit" : "AlignThenGo");
}

void Navigator::startRotate(quint32 taskId, double currentYaw, double angleRad,
                            bool absolute, double maxTurnRate,
                            double toleranceRad, qint64 timeoutMs)
{
    _taskId = taskId;
    _rotateAbsolute = absolute;
    _rotateTargetYaw = angleRad;
    _rotateDelta = angleRad;
    _rotateAccum = 0.0;
    _rotatePrevRawYaw = currentYaw;
    _rotateMaxTurnRate = maxTurnRate > 0.0
        ? qMin(maxTurnRate, _params.maxTurnRate) : _params.maxTurnRate;
    _rotateTolerance = toleranceRad > 0.0
        ? toleranceRad : _params.headingToleranceRad;
    _timeoutMs = timeoutMs;
    _startMs = QDateTime::currentMSecsSinceEpoch();
    _remainingDist = 0.0;
    _vDesPrev = 0.0;
    _lastUpdateMs = 0;
    _lastFwdJoy = 0.0;
    _phase = Phase_Rotate;

    qInfo() << "[Navigator] 启动旋转任务" << taskId
            << (absolute ? "绝对目标yaw=" : "相对增量=")
            << angleRad * 180.0 / PI << "° 当前原始yaw=" << currentYaw * 180.0 / PI
            << "° 限速=" << _rotateMaxTurnRate << "rad/s 容差="
            << _rotateTolerance * 180.0 / PI << "° 超时=" << _timeoutMs << "ms";
}

void Navigator::updateRotation(double localYaw, double yawRate, qint64 now)
{
    double remaining = 0.0;
    if (_rotateAbsolute)
    {
        remaining = radStd(_rotateTargetYaw - localYaw);
    }
    else
    {
        _rotateAccum += radStd(localYaw - _rotatePrevRawYaw);
        remaining = _rotateDelta - _rotateAccum;
    }
    _rotatePrevRawYaw = localYaw;
    _remainingDist = 0.0;

    constexpr double kRad2Deg = 57.29577951308232;
    const QString modeText = _rotateAbsolute ? QStringLiteral("绝对旋转")
                                             : QStringLiteral("相对旋转");
    if (std::fabs(remaining) <= _rotateTolerance)
    {
        finishHard(DogProtocol::MR_ARRIVED,
                   QStringLiteral("%1到位，原始yaw=%2°，剩余角度=%3°")
                       .arg(modeText)
                       .arg(localYaw * kRad2Deg, 0, 'f', 2)
                       .arg(remaining * kRad2Deg, 0, 'f', 2));
        return;
    }

    if (_timeoutMs > 0 && (now - _startMs) > _timeoutMs)
    {
        finishHard(DogProtocol::MR_TIMEOUT,
                   QStringLiteral("%1超时 %2ms，原始yaw=%3°，剩余角度=%4°")
                       .arg(modeText).arg(_timeoutMs)
                       .arg(localYaw * kRad2Deg, 0, 'f', 2)
                       .arg(remaining * kRad2Deg, 0, 'f', 2));
        return;
    }

    double turnRate = clamp(_params.kpAlign * remaining - _params.kdAlign * yawRate,
                            -_rotateMaxTurnRate, _rotateMaxTurnRate);
    if (std::fabs(remaining) < _params.alignSlowBandRad)
    {
        const double slowLimit = qMin(_rotateMaxTurnRate, _params.alignSlowMaxTurnRate);
        turnRate = clamp(turnRate, -slowLimit, slowLimit);
    }
    double turnJoy = clamp(turnRate / _params.joyTurnScale, -1.0, 1.0);
    turnJoy = applyAxisDeadzone(turnJoy);

    if (_params.debugLog && now - _dbgLastMs >= 200)
    {
        _dbgLastMs = now;
        qInfo().noquote() << QStringLiteral(
            "[NvRot] task=%1 mode=%2 rawYaw=%3° command=%4° accum=%5° "
            "remaining=%6° yawRate=%7°/s turnJoy=%8")
            .arg(_taskId)
            .arg(modeText)
            .arg(localYaw * kRad2Deg, 0, 'f', 2)
            .arg((_rotateAbsolute ? _rotateTargetYaw : _rotateDelta) * kRad2Deg, 0, 'f', 2)
            .arg(_rotateAccum * kRad2Deg, 0, 'f', 2)
            .arg(remaining * kRad2Deg, 0, 'f', 2)
            .arg(yawRate * kRad2Deg, 0, 'f', 2)
            .arg(turnJoy, 0, 'f', 3);
    }

    emit remoteCommand(0.0, turnJoy);
}

void Navigator::cancel()
{
    if (_phase == Phase_Idle)
        return;
    finishHard(DogProtocol::MR_FAILED, QStringLiteral("任务被取消"));
}

void Navigator::finishHard(DogProtocol::MoveResult result, const QString &message)
{
    const quint32 taskId = _taskId;
    const double remain = _remainingDist;
    _phase = Phase_Idle;
    emit stopRequested();
    emit finished(taskId, result, remain, message);
    qInfo() << "[Navigator] 任务" << taskId << "硬结束 result=" << result
            << "剩余距离=" << QString::number(remain, 'f', 3) << message;
}

void Navigator::finishSoft(DogProtocol::MoveResult result, const QString &message)
{
    const quint32 taskId = _taskId;
    const double remain = _remainingDist;
    _phase = Phase_SoftArrived;
    emit finished(taskId, result, remain, message);
    qInfo() << "[Navigator] 任务" << taskId << "软到达 result=" << result
            << "剩余距离=" << QString::number(remain, 'f', 3) << message
            << "(继续前进，等下一个任务抢占)";
}

// ============================================================
// 收油衰减（两种转向律共用）
// 进入 stopArriveRadiusMul × arriveRadius 后线性降速，保留符号，
// 并保证不低于 minCreepSpeed（避免接近目标时完全不动）。
// ============================================================
void Navigator::applyArriveDecay(double &vDes, double dist) const
{
    const double decayStart = _params.stopArriveRadiusMul * _arriveRadius;
    if (dist >= decayStart || dist <= 0.0)
        return;

    const double denom = qMax(decayStart - _arriveRadius, 0.01);
    const double decay = clamp((dist - _arriveRadius) / denom, 0.0, 1.0);
    const double sign  = (vDes >= 0.0) ? 1.0 : -1.0;
    double mag = std::fabs(vDes) * decay;
    if (mag < _params.minCreepSpeed)
        mag = _params.minCreepSpeed;
    vDes = sign * mag;
}

// ============================================================
// Pure Pursuit 转向律
// 核心：不设"原地对准"阶段，狗一直在前进中转向，消除腿足原地
// 旋转打滑（实测里程计单帧跳 1~2m 的根源）。
//
//   Ld    = clamp(K·vMax + LdMin, LdMin, LdMax)  且不超过当前距离
//   α     = 目标方位角 - 当前航向
//   ω     = 2·v·sin(α)/Ld                        （曲率 κ=2sinα/Ld）
//   v     ≤ ωmax·Ld/(2|sinα|)                    （转不过来先减速）
//
// 目标在后方时不再"掉头再走"，而是直接倒车接近（allowBackward 时），
// 倒车同样满足上述曲率约束（v 取负，ω 自动反号）。
// ============================================================
void Navigator::computePurePursuit(double dx, double dy, double yaw,
                                   double dist, double &vDesRaw, double &turnRate)
{
    // --- 前视距离：随目标巡航速度自适应，且不能超过实际目标距离 ---
    double ld = _params.ppLookaheadK * _maxSpeed + _params.ppLookaheadMin;
    ld = clamp(ld, _params.ppLookaheadMin, _params.ppLookaheadMax);
    ld = qMin(ld, qMax(dist, _params.ppLookaheadMin * 0.5));

    // --- 目标方位角（局部系，与 yaw 同向）---
    const double alpha = radStd(std::atan2(dy, dx) - yaw);

    // --- 正后方且禁止倒车：曲率律会退化成"sinα≈0 → ω≈0 → 一直往前跑"，
    //     必须先低速转向把目标拉回侧前方，再交给 Pure Pursuit ---
    if (!_params.allowBackward && std::fabs(alpha) > (135.0 * PI / 180.0))
    {
        vDesRaw  = _params.alignCreepSpeed;
        turnRate = (alpha >= 0.0 ? 1.0 : -1.0) * _params.maxTurnRate;
        return;
    }

    const double sinA  = std::sin(alpha);
    const double absSin = std::fabs(sinA);

    // --- 可达曲率反解线速度上限：ω = 2·v·|sinα|/Ld ≤ ωmax ---
    double vMaxCurv = _maxSpeed;
    if (absSin > 1e-3)
        vMaxCurv = _params.maxTurnRate * ld / (2.0 * absSin);

    double vNom = qMin(_maxSpeed, vMaxCurv);
    if (vNom < _params.minCreepSpeed)
        vNom = qMin(_params.minCreepSpeed, _maxSpeed);  // 至少给个能走起来的速度

    // --- 目标在后方：倒车接近（机尾对准），否则正向前进 ---
    const bool reverse = _params.allowBackward && std::fabs(alpha) > (PI / 2.0);
    const double vSigned = reverse ? -vNom : vNom;

    turnRate = clamp(2.0 * vSigned * sinA / ld,
                     -_params.maxTurnRate, _params.maxTurnRate);
    vDesRaw  = vSigned;

    // --- 大偏差额外收油：接近 90° 时纯曲率约束仍容易甩尾，主动降速 ---
    if (absSin > 0.85)
    {
        const double sign = (vDesRaw >= 0.0) ? 1.0 : -1.0;
        vDesRaw = sign * qMax(std::fabs(vDesRaw) * 0.6, _params.minCreepSpeed);
    }
}

void Navigator::update(double localX, double localY, double localYaw,
                       double bodyVx, double yawRate)
{
    if (_phase == Phase_Idle)
        return;

    const qint64 now = QDateTime::currentMSecsSinceEpoch();

    if (_phase == Phase_Rotate)
    {
        updateRotation(localYaw, yawRate, now);
        return;
    }

    // 点到点导航直接使用狗反馈的原始局部 yaw。
    const double yaw = localYaw;

    const double dx = _targetX - localX;
    const double dy = _targetY - localY;
    const double dist = std::sqrt(dx * dx + dy * dy);
    _remainingDist = dist;

    // ============================================================
    // 【硬到达】进入 arriveRadius 必须停车。
    // 放在软到达之前：软到达只是为了给服务器留出"提前下发下一个航点"的
    // 时间窗，一旦真的走到 arriveRadius（例如 0.5m）就必须停下来，
    // 否则目标点跑到脚下后方位角病态，狗会在原地反复转圈直到超时。
    // ============================================================
    if (dist <= _arriveRadius)
    {
        finishHard(DogProtocol::MR_ARRIVED,
                   QStringLiteral("进入到达半径(%1m)，剩余 %2m，已停车")
                       .arg(_arriveRadius, 0, 'f', 2).arg(dist, 0, 'f', 3));
        return;
    }

    // ============================================================
    // 【软到达】提前 softArriveRadiusMul × arriveRadius 发 MR_ARRIVED
    // feedback，不停车、不退导航状态。狗继续往前走（vDes 随收油段
    // 自然减速），等服务器收到 feedback 后下发下一个任务 →
    // Navigator::start() 抢占 → 无缝变向，中途狗不停。
    // 若始终没有新任务，则走到上面的硬到达半径时停车。
    // ============================================================
    const double softRadius = _params.softArriveRadiusMul * _arriveRadius;
    if (dist <= softRadius && !_softArrivedReported)
    {
        _softArrivedReported = true;
        finishSoft(DogProtocol::MR_ARRIVED,
                   QStringLiteral("进入软到达半径(%1m)，剩余 %2m")
                       .arg(softRadius, 0, 'f', 2).arg(dist, 0, 'f', 3));
        // 注意：不 return，继续往下算 fwdCmd/turnCmd，让狗自然收油往前走
    }

    // 超时判定（硬到达，必然停车）
    if (_timeoutMs > 0 && (now - _startMs) > _timeoutMs)
    {
        finishHard(DogProtocol::MR_TIMEOUT,
                   QStringLiteral("导航超时 %1ms").arg(_timeoutMs));
        return;
    }

    // 防跑飞（硬到达，必然停车）
    if (_startDist <= 0.0)
        _startDist = dist;
    const double escapeLimit = qMax(_startDist * 3.0, _startDist + 10.0);
    if (dist > escapeLimit)
    {
        if (_escapeSinceMs == 0)
            _escapeSinceMs = now;
        else if (now - _escapeSinceMs > 2000)
        {
            finishHard(DogProtocol::MR_FAILED,
                       QStringLiteral("偏离目标过远（%1m），已停车").arg(dist, 0, 'f', 1));
            return;
        }
    }
    else
    {
        _escapeSinceMs = 0;
    }

    const double targetYaw = std::atan2(dy, dx);
    const double yawErr = radStd(targetYaw - yaw);

    // 目标在侧后方且允许倒退时，改为机尾对准目标、后退接近
    const bool backward = _params.allowBackward && std::fabs(yawErr) > (PI / 2.0);
    const double headingErr = backward ? radStd(yawErr - PI) : yawErr;

    // ============================================================
    // 大角度掉头方向锁：
    //   |headingErr| 接近 180° 时，角度归一化会在 ±π 处翻符号，
    //   P 控制随之反向，现场表现为 turnJoy 在 -1/+1 间抖动、狗原地绕圈。
    //   这里在 |误差|>170° 时锁定初始转向方向并把误差连续展开，
    //   转到 |误差|<90° 后自动解除。
    // ============================================================
    if (!backward && std::fabs(headingErr) > 170.0 * PI / 180.0)
    {
        if (_turnDirLock == 0)
            _turnDirLock = (headingErr > 0.0) ? 1 : -1;
    }
    else if (std::fabs(headingErr) < 90.0 * PI / 180.0)
    {
        _turnDirLock = 0;
    }
    double headingErrCont = headingErr;
    if (_turnDirLock > 0 && headingErrCont < 0.0)
        headingErrCont += 2.0 * PI;
    else if (_turnDirLock < 0 && headingErrCont > 0.0)
        headingErrCont -= 2.0 * PI;

    // ============================================================
    // 诊断日志（5Hz 限流）：定位"目标在正前方却转向"。
    // 判读方法：
    //   headingErr ≈ 0      → 坐标/航向基准一致，转向来自别处（步态/输出层）
    //   headingErr 恒定偏移 → yaw 与 dx/dy 不在同一坐标基准（差量即偏移角）
    //   headingErr 反复跳变 → 原始局部 yaw 或里程计反馈跳变
    // ============================================================
    if (_params.debugLog && now - _dbgLastMs >= 200)
    {
        _dbgLastMs = now;
        constexpr double kRad2Deg = 57.29577951308232;
        qInfo().noquote() << QStringLiteral(
            "[NvDbg] task=%1 local(%2,%3) yaw=%4° dist=%5m "
            "dxy(%6,%7) targetYaw=%8° headingErr=%9° "
            "mode=%10 phase=%11 vx=%12 yawRate=%13°/s")
            .arg(_taskId)
            .arg(localX, 0, 'f', 2).arg(localY, 0, 'f', 2)
            .arg(yaw * kRad2Deg, 0, 'f', 1)
            .arg(dist, 0, 'f', 2)
            .arg(dx, 0, 'f', 2).arg(dy, 0, 'f', 2)
            .arg(targetYaw * kRad2Deg, 0, 'f', 1)
            .arg(headingErr * kRad2Deg, 0, 'f', 1)
            .arg(_params.steeringMode == Steering_PurePursuit
                     ? QStringLiteral("PP") : QStringLiteral("ATG"))
            .arg(static_cast<int>(_phase))
            .arg(bodyVx, 0, 'f', 2)
            .arg(yawRate * kRad2Deg, 0, 'f', 1);
    }

    // ============================================================
    // 第一步：各分支只产出"原始期望速度 vDesRaw"和转向 turnRate，
    // 不做速度平滑、不做闭环。所有 vDesRaw 最后统一过 smoothVDes()
    // —— 这是经验 100000659 的核心：单一决策出口，避免多分支隐式覆盖。
    // ============================================================
    double vDesRaw = 0.0;   // 原始期望速度（方向为正前方，backward 在最后统一取反）
    double turnRate = 0.0;

    if (_params.steeringMode == Steering_PurePursuit)
    {
        // ============================================================
        // 【Pure Pursuit】不停车、边走边转，直接输出连续 (v, ω)。
        // 腿足原地旋转打滑最严重，这里从根上取消了"先对准再走"阶段。
        // ============================================================
        if (_phase == Phase_Align)
            _phase = Phase_Move;   // 纯追踪没有独立的对准阶段

        computePurePursuit(dx, dy, yaw, dist, vDesRaw, turnRate);
        applyArriveDecay(vDesRaw, dist);   // 收油衰减（保留倒车符号）
    }
    else if (_phase == Phase_Align && std::fabs(headingErr) > _params.headingToleranceRad)
    {
        // 对准阶段：MATRiX walk 需 forward≠0 才执行转向，
        // 带一个最小 creep 速度，实现"边走边转"
        vDesRaw = _params.alignCreepSpeed;
        // P + D：yawRate 阻尼抑制真狗约 38° 的停转惯性过冲；
        // 误差用连续展开值，掉头时不会在 ±180° 处反向
        turnRate = clamp(_params.kpAlign * headingErrCont - _params.kdAlign * yawRate,
                         -_params.maxTurnRate, _params.maxTurnRate);
    }
    else
    {
        _phase = Phase_Move;
        // 若直行中航向偏差过大，退回对准阶段
        if (std::fabs(headingErr) > _params.reAlignThresholdRad)
        {
            _phase = Phase_Align;
            vDesRaw = _params.alignCreepSpeed;
            turnRate = clamp(_params.kpAlign * headingErrCont - _params.kdAlign * yawRate,
                             -_params.maxTurnRate, _params.maxTurnRate);
        }
        else
        {
            // ============================================================
            // 【固定速度巡航】vDesRaw = _maxSpeed
            // —— Cruise 模式硬切 2.0 m/s
            // —— Profile 模式由 smoothVDes 逐步加速到 2.0
            // ============================================================
            vDesRaw = _maxSpeed;

            // ============================================================
            // 【收油衰减 · 两种模式都强制启用】
            // 没有收油衰减的话，快速巡航到终点狗冲过目标后目标方向反向、
            // 大转向 180°、站不稳——这就是之前删掉收油段导致的 bug。
            // ============================================================
            applyArriveDecay(vDesRaw, dist);

            // 航向保持输出死区：航向误差在容差内直接输出 0，抵抗 IMU yaw 微抖动
            double yawHoldOut = clamp(_params.kpHoldYaw * headingErr - _params.kdAlign * yawRate,
                                      -_params.maxTurnRate, _params.maxTurnRate);
            if (std::fabs(headingErr) < _params.holdYawOutDeadzoneRad)
                yawHoldOut = 0.0;
            turnRate = yawHoldOut;
        }
    }

    // ============================================================
    // 【原地空转脱困】仅 Pure Pursuit。
    // 曲率律 ω=2v·sinα/Ld 的前提是"狗真的按 v 前进"：一旦打滑原地转，
    // 位置不动→α 不变→指令不变，就是现场看到的"原地画圈找不到去向"。
    // 判据：转向大 + 期望前进 + 实测几乎不动，持续 ppStuckSec。
    // 处置：前进量归零、方向饱和转向，先把朝向拧到 ppStuckExitDeg 以内；
    // 对准段最长 8s，避免狗本身转不动时永久卡死。
    // ============================================================
    if (_params.steeringMode == Steering_PurePursuit && _params.ppStuckSec > 0.0)
    {
        const bool spin = std::fabs(turnRate) >= _params.ppStuckTurnMin
                       && vDesRaw >= _params.ppStuckFwdMin
                       && std::fabs(bodyVx) <= _params.ppStuckVxMax;
        if (spin)
        {
            if (_stuckSinceMs == 0)
                _stuckSinceMs = now;
        }
        else if (!_stuckAlign)
        {
            _stuckSinceMs = 0;
        }

        if (!_stuckAlign && _stuckSinceMs != 0
            && (now - _stuckSinceMs) >= static_cast<qint64>(_params.ppStuckSec * 1000.0))
        {
            _stuckAlign = true;
            _stuckAlignSinceMs = now;
            qWarning().noquote() << QStringLiteral(
                "[Nav] 原地空转 %1s（turnJoy 有输出但位置不动）：前进量归零，先原地对准，"
                "朝向误差 %2°")
                .arg((now - _stuckSinceMs) / 1000.0, 0, 'f', 1)
                .arg(headingErr * 57.29577951308232, 0, 'f', 1);
        }

        if (_stuckAlign)
        {
            const bool aligned = std::fabs(headingErr) < (_params.ppStuckExitDeg * PI / 180.0);
            const bool giveUp = (now - _stuckAlignSinceMs) > 8000;
            if (aligned || giveUp)
            {
                qWarning().noquote() << QStringLiteral(
                    "[Nav] 对准段结束%1：朝向误差 %2°，恢复 Pure Pursuit")
                    .arg(giveUp ? QStringLiteral("（超时放行）") : QString())
                    .arg(headingErr * 57.29577951308232, 0, 'f', 1);
                _stuckAlign = false;
                _stuckSinceMs = 0;
            }
            else
            {
                vDesRaw = 0.0;   // 关键：前进量清零，不再画小圈
                turnRate = (headingErr >= 0.0 ? 1.0 : -1.0) * _params.maxTurnRate;
            }
        }
    }

    // ============================================================
    // 【接近容差减速带】仅 AlignThenGo 需要
    // |headingErr| 进入 alignSlowBandRad 后把转向角速度压到
    // alignSlowMaxTurnRate 以内：真狗撤舵后仍有 30~40° 惯性，
    // 满舵冲进 10/20° 容差必然过冲反向，于是来回摆。
    // PurePursuit 下 |α|<25° 是正常巡航区间，压到 0.4 rad/s 会导致
    // 曲率不足、切弯严重滞后，所以这里必须跳过。
    // ============================================================
    if (_params.steeringMode == Steering_AlignThenGo
        && std::fabs(headingErr) < _params.alignSlowBandRad)
        turnRate = clamp(turnRate, -_params.alignSlowMaxTurnRate,
                         _params.alignSlowMaxTurnRate);

    // backward 统一取反（放在收油衰减之后，避免与衰减率混淆）
    // 注意：PurePursuit 内部已经按 sin(α) 自动处理了倒车符号，不能再取反一次
    if (_params.steeringMode == Steering_AlignThenGo && backward)
        vDesRaw = -vDesRaw;

    // ============================================================
    // 第二步：统一平滑出口 —— 两种模式只在这里区分
    // ============================================================
    const double vDesSmoothed = smoothVDes(vDesRaw, now);

    // ============================================================
    // 第三步：速度闭环 + 摇杆转换 + 死区补偿 —— 统一出口
    // 速度闭环按 vDesSmoothed 符号限制在同向半边，避免 vDes=0 时反向刹车
    // ============================================================
    const double vFb = vDesSmoothed + _params.kpVel * (vDesSmoothed - bodyVx);
    double fwdCmd = (vDesSmoothed >= 0.0)
                        ? clamp(qMax(0.0, vFb), 0.0, _maxSpeed)
                        : clamp(qMin(0.0, vFb), -_maxSpeed, 0.0);

    // ---- 摇杆增益自适应 ----
    // SetRemote 收的是归一化量，"1.0 摇杆 = 多少 m/s"在同一台狗上也会随
    // 地面材质/电量/档位变化。用上一帧实际下发的摇杆量和本帧实测 bodyVx
    // 反算真实增益，慢速一阶跟随替代固定的 joyLinearScale。
    // 只在"指令与速度同号且幅度够大"时采样，避免倒车/打滑污染。
    if (_params.joyGainAdaptEnable && std::fabs(_lastFwdJoy) > 0.15
        && std::fabs(bodyVx) > 0.05 && _lastFwdJoy * bodyVx > 0.0)
    {
        const double observed = std::fabs(bodyVx) / std::fabs(_lastFwdJoy);
        if (observed > _params.joyGainMin * 0.5 && observed < _params.joyGainMax * 1.5)
        {
            _joyGain += (observed - _joyGain) * _params.joyGainTau;
            _joyGain = clamp(_joyGain, _params.joyGainMin, _params.joyGainMax);
        }
    }

    const double gain = (_params.joyGainAdaptEnable && _joyGain > 1e-3)
                            ? _joyGain : _params.joyLinearScale;
    double fwdJoy = clamp(fwdCmd / gain, -1.0, 1.0);
    double turnJoy = clamp(turnRate / _params.joyTurnScale, -1.0, 1.0);

    // MATRiX 死区补偿：想动的输出抬到 minAxisOut 之上
    fwdJoy = applyAxisDeadzone(fwdJoy);
    turnJoy = applyAxisDeadzone(turnJoy);

    _lastFwdJoy = fwdJoy;
    emit remoteCommand(fwdJoy, turnJoy);
}
