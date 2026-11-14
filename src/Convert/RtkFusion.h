#ifndef RTK_FUSION_H
#define RTK_FUSION_H

#include <QString>
#include <QVector>

// ============================================================
// 差分(RTK)绝对位姿融合与在线标定
//
// 为什么需要它：
//   差分定位 200 ms 一帧（5 Hz），控制节拍 20 Hz，且差分帧到达有抖动；
//   里程计 20 Hz 但会随距离累积尺度误差与航向漂移。
//   导航输出必须同时又稳又准，因此取"差分做锚点、里程计做增量"：
//       pose = 里程计位姿 + offset，offset 由每一帧差分测量出来并低通跟踪
//
// 坐标口径（与 MapTransform 的地图系一致，无需二次换算）：
//   差分：x 东、y 南，单位 m；航向 度，正东 0、顺时针 0~360
//   地图：同上，两者手性一致，只差一个"尺度 + 转角 + 平移"的相似变换
//
// 标定只解"尺度 k 与转角 alpha"，不保存平移：
//   平移每次都由当前差分帧与当前里程计位姿现场相减得到并吸收进 offset，
//   这样地图参考点被人为重设（AutoReference / mapcheck）后不会把平移解带坏，
//   重新标定量也因此只剩 k 与 alpha 两个自由度，样本需求少、抗噪声好。
//
// 航向零位 b（会话量，不进 ini）：
//   差分航向每次上电的零位是随机的，但转向增量准确。建地图参考的那一姿态就是
//   地图 0°（正东），所以 b = 那一姿态下的差分原值 yaw，所有换算统一用 yaw - b。
//   这与服务端是否已归零无关：已归零则 b≈0（扣了等于没扣），未归零就是随机角
//   （扣掉正好），不存在"两边各扣一次"的双扣。b 不写 ini——每次上电即过期；
//   它随 reset() 失效，由 requestYawZero() 在静止窗口内重取。
//
//   b 的两种口径（并存，不是二选一）：
//     · 参考口径（初值）：建参考那一静止帧的差分原值 yaw。等于把当时狗的姿态当
//       成了地图东，所以摆位有角误差时 yaw 就带一个常值偏差。
//     · 直线口径（终值）：b = 圆周均值(差分航向读数) − θ(差分位移方向)。停点
//       A→B 的差分位移方向就是狗在地图里的真实行进方向，所以 b 归到地图东，不再
//       依赖"建参考时狗正好朝东"，天线安装角一并合并进 b。
//     直线观测与 k/α 标定解耦（k/α 标定后 trySolve 直接返回、不再采点，若把 b 反算
//     挂进去，第二次以后的场次永远不会触发）。观测器只看"停点之间的位移"。
//
// 标定方向（重要）：把里程计映射到差分的绝对系，而不是把差分压进里程计尺度。
//   差分给出的米才是地图上真实的米；若反过来（fit rtk ≈ k·R(α)·odom 的逆），
//   位姿会继承 mapcalib 的残余尺度误差，导航到 10 m 处就会停在 9 m。
//   因此约定 p=差分绝对坐标、q=里程计地图坐标，解 p ≈ k·R(α)·q，
//   输出 pose = k·R(α)·odom + offset。
//
// 断流降级：超过 holdMs 没有新帧时 offset 冻结，位姿仍按里程计外推，
//   输出标记为 SrcHold；恢复后 offset 继续低通跟踪，不跳变。
// ============================================================
class RtkFusion
{
public:
    struct Params
    {
        bool   autoCalib;        // 在线自动解 k 与 alpha（默认 true）
        bool   useForNav;        // 融合位姿是否交给导航使用（默认 true）
        int    calibMinSamples;  // 参与解算的最少样本帧（默认 40，约 8 s 数据）
        double calibMinPathM;    // 参与解算的最少累计行走距离 m（默认 1.0）
        double calibMaxRmsM;     // 允许的最大拟合残差 RMS m（默认 0.12）
        double offsetTauSec;     // offset 一阶低通时间常数 s（默认 0.5）
        double snapM;            // offset 突变超过该值即直接吸附 m（默认 0.8）
        int    holdMs;           // 超过该值无新帧进入 hold 降级 ms（默认 2000）
        bool   useRtkYaw;        // 是否用差分航向做绝对朝向（默认 true；服务端航向恒 0 时置 false）
        // 差分 5Hz、里程计 20Hz，差分帧天生滞后 0.2~0.5s（移动时就是 0.1~0.5m 的错位），
        // 所以 offset 只在"够静止"的时刻采信；运动中冻结，靠里程计增量推进。
        double offStaticSpeed;   // 低于该速度视为静止，可快速重锚 m/s（默认 0.03）
        double offSlowSpeed;     // 低于该速度允许慢速微调 m/s（默认 0.2）
        double offSlowYawRateDeg;// 角速度低于该值才允许微调航向 度/s（默认 5）
        double offStaticTauSec;  // 静止时的 offset 低通时间常数 s（默认 0.3）
        double offSlowTauSec;    // 慢速时的 offset 低通时间常数 s（默认 2.0）
        int    calibStaticHoldMs;// 持续静止多久才算一个"静止锚点" ms（默认 1500）
        double calibPointGapM;   // 两个静止锚点的最小间距 m（默认 0.35，避免原地重复停）

        // ---- 差分航向零位 b（会话量，每次"建参考"重取一次）----
        // 差分航向每次上电的零位随机、转向增量准确。地图参考建立的那一姿态就是
        // 地图 0°（正东），因此 b = 那一姿态下的差分原值 yaw；归一化 yaw = yaw - b。
        // 这个定义与服务端是否已归零无关：归零了 b≈0（扣了等于没扣），
        // 没归零就是随机角（扣掉正好），不存在双扣。
        bool   autoZeroYaw;      // 是否在"建参考"时自动取零位（默认 true）
        int    zeroHoldMs;       // 静止持续多久才开始取零位 ms（默认 800）
        int    zeroMinFrames;    // 取多少帧做圆周均值（默认 3）

        // 直线反算零位：停点 A→B 的"差分位移方向"就是狗在服务端地图里的真实行进
        // 方向，这段路程里差分给自己的航向读数与它之差就是零位 b（含天线安装角，
        // 合并成一个量即可）。这样 b 归到"地图东"，不再依赖"建参考时狗正好朝东"。
        // 与 k/α 标定解耦：k/α 已标定、不再采样的场次照样能用。
        bool   zeroFromLine;     // 是否启用（默认 true；仅在 YawFromRtk=true 时生效）
        double zeroLineSegM;     // 单段最小位移 m（默认 0.6，太短则角度噪声被放大）
        double zeroLineMinM;     // 累计直线长度门槛 m（默认 2.5）
        double zeroLineSpreadDeg;// 各段观测之间允许的最大散布 度（默认 2.0）

        // 停点标定结果的合理性判据：越界不采用（避免错值持久化后不再重解）
        double calibKMin;        // k 下限（默认 0.95）
        double calibKMax;        // k 上限（默认 1.05）
        double calibAlphaMaxDeg; // |α| 上限 度（默认 5）

        Params()
            : autoCalib(true)
            , useForNav(true)
            , calibMinSamples(4)
            , calibMinPathM(0.8)
            , calibMaxRmsM(0.12)
            , offsetTauSec(0.5)
            , snapM(0.8)
            , holdMs(2000)
            , useRtkYaw(true)
            , offStaticSpeed(0.03)
            , offSlowSpeed(0.2)
            , offSlowYawRateDeg(5.0)
            , offStaticTauSec(0.3)
            , offSlowTauSec(2.0)
            , calibStaticHoldMs(1500)
            , calibPointGapM(0.35)
            , autoZeroYaw(true)
            , zeroHoldMs(800)
            , zeroMinFrames(3)
            , zeroFromLine(true)
            , zeroLineSegM(0.6)
            , zeroLineMinM(2.5)
            , zeroLineSpreadDeg(2.0)
            , calibKMin(0.95)
            , calibKMax(1.05)
            , calibAlphaMaxDeg(5.0) {}
    };

    struct Calib
    {
        bool   valid;        // 是否已有可用标定
        double k;            // 差分 -> 地图 的尺度（无标定时恒为 1）
        double alphaRad;     // 差分 -> 地图 的转角 rad（位置拟合解）
        double alphaYawRad;  // 差分 -> 地图 的转角（航向差均值，用于一致性校核）
        double rmsM;         // 位置拟合残差 RMS m
        double pathM;        // 参与解算的累计行走距离 m
        int    samples;      // 参与解算的样本帧数

        Calib()
            : valid(false), k(1.0), alphaRad(0.0), alphaYawRad(0.0),
              rmsM(0.0), pathM(0.0), samples(0) {}
    };

    // 位姿来源标记（写进日志/上报，便于现场判断当前用的是哪套位姿）
    enum Source
    {
        SrcNone = 0,   // 无差分，纯里程计
        SrcRtk  = 1,   // 差分新鲜，offset 在跟踪
        SrcHold = 2    // 差分断流，offset 冻结 + 里程计外推
    };

    void setParams(const Params &p) { _p = p; }
    const Params &params() const { return _p; }

    void setCalib(const Calib &c) { _calib = c; }
    const Calib &calib() const { return _calib; }

    // 清空观测与标定样本（地图参考被重设时必须调用）
    void reset();

    // 请求重取差分航向零位 b：建参考 / 判定模块重启时调用。
    // 真正的取点发生在"静止窗口"内（连续 ZeroHoldMs 静止后取 ZeroMinFrames 帧圆周均值），
    // 取到之前不建立/不更新 offset——否则会用一个随机零位把航向基准初始化错。
    // targetMapYawRad：取到零位后，归一化航向应等于的地图朝向（rad）。
    // 建参考时传 0（与旧行为完全等价：b = 该姿态读数）；模块重启时传当前里程计地图 yaw，
    // 这样重取 b 不会把"自参考建立以来转过的角"变成一条航向偏置。
    void requestYawZero(const QString &reason, double targetMapYawRad = 0.0);

    bool hasYawZero() const { return _zeroValid; }
    bool yawZeroPending() const { return _zeroPending; }
    double yawZeroRad() const { return _zeroYawRad; }
    double yawZeroSpreadDeg() const { return _zeroSpreadDeg; }
    bool yawZeroFromLine() const { return _zeroFromLineSet; }   // b 是直线段反算来的（否则取建参考姿态读数）

    bool hasOffset() const { return _offInit; }
    int  lastSource() const { return _lastSrc; }
    qint64 lastFrameMs() const { return _lastFrameMs; }
    int  staticPoints() const { return _n; }          // 已采到的静止锚点数
    // 实时累计锚点里程。注意与 calib().pathM 的区别：后者只在解算成功时写入，
    // 未解时恒 0；面板/诊断必须用这个，否则会显示"里程 0.00m"把人引向"再多走点"。
    double pathM() const { return _path; }
    bool offsetFrozen() const { return _offFrozen; }  // 当前是否冻结 offset（运动中）
    double lastSpeed() const { return _lastSpeed; }
    double frozenDriftM() const { return (_driftCount > 0) ? (_driftSum / _driftCount) : 0.0; } // 冻结期间差分-融合残差均值 m

    // 一帧差分：rtk* 为差分原始位姿（地图口径，已补偿天线安装偏置），
    // odom* 为"同一时刻"里程计推出的地图位姿（调用方传入当前时刻的姿态，
    // 差分的滞后由本模块用 speedMps / yawRateRad 做静止门控来规避）。
    void onRtkFrame(double rtkX, double rtkY, double rtkYaw,
                    double odomX, double odomY, double odomYaw, qint64 tMs,
                    double speedMps, double yawRateRad);

    // 尝试解算标定；返回 true 表示本次产生了被接受的新解
    bool trySolve(qint64 tMs);

    // 融合输出（地图系）：
    //   返回 false 表示尚未建立 offset（调用方退回纯里程计位姿）
    bool fusedMapPose(double odomX, double odomY, double odomYaw, qint64 tMs,
                      double *x, double *y, double *yaw, int *src) const;

    // 标定结果持久化（QSettings ini）
    bool loadCalib(const QString &path, QString *message);
    bool saveCalib(const QString &path, QString *message) const;

    QString describe() const;

private:
    struct Sample
    {
        double px, py;  // 差分绝对坐标（地图口径）
        double qx, qy;  // 同一帧的里程计地图位姿
        double dyaw;    // 差分航向 - 里程计地图航向
    };

    void addSample(const Sample &s, bool isStatic);
    void clearSamples();
    // 解不出来时把"卡在哪一条判据"打出来（限频 3s），返回 false 以便直连 return。
    // 历史教训：RMS 超限原本是静默 return，面板又错报成"里程不足"，现场只能靠猜。
    bool logReject(qint64 tMs, const QString &why);

    Params _p;
    Calib  _calib;

    // offset 平滑状态
    bool   _offInit = false;
    double _offX = 0.0, _offY = 0.0, _offYaw = 0.0;
    qint64 _offMs = 0;
    qint64 _lastFrameMs = 0;
    int    _lastSrc = SrcNone;
    bool   _bigWarned = false;
    bool   _snapNext = false;   // 标定刚更新：下一帧直接吸附，不用低通爬坡
    bool   _offFrozen = false;  // 当前 offset 是否处于冻结（运动中/转动中）
    double _lastSpeed = 0.0;    // 最近一帧差分时刻的线速度 m/s
    double _driftSum = 0.0;     // 冻结期间差分与融合位姿的累计残差（判里程计是否跑飞）
    int    _driftCount = 0;
    bool   _driftWarned = false; // 冻结残差告警只报一次
    qint64 _staticSinceMs = 0;  // 当前静止段的起始时刻
    bool   _staticStill = false;// 已静止足够久（calibStaticHoldMs）
    bool   _staticPointTaken = false; // 本静止段是否已采过锚点

    // 差分航向零位 b（会话量，不进 ini）
    double _zeroYawRad = 0.0;   // 参考姿态下的差分原值 yaw（rad，未归一化）
    double _zeroTargetYawRad = 0.0; // 本次取零位要对齐的地图朝向（rad），建参考时为 0
    bool   _zeroValid = false;  // 是否已取到
    bool   _zeroPending = false;// 是否在等待静止窗口取零
    qint64 _zeroSinceMs = 0;    // 本静止窗口起点
    int    _zeroN = 0;          // 窗口内已累计帧数
    double _zeroSin = 0.0;
    double _zeroCos = 0.0;
    double _zeroFirstYaw = 0.0; // 窗口首帧 yaw（算抖动）
    double _zeroMaxDev = 0.0;   // 窗口内相对首帧的最大偏差 度
    double _zeroSpreadDeg = 0.0;// 最终采用窗口的抖动 度
    qint64 _zeroMs = 0;
    bool   _zeroFromLineSet = false;  // 当前 b 是否由直线段反算得到
    bool   _bandRejectWarned = false; // k/α 越界只告警一次

    // 直线反算 b 的观测器（与 k/α 标定完全独立，k/α 已标定的场次也能用）
    bool   _lzActive = false;   // 是否在观测
    bool   _lzHasSeg = false;   // 是否已有线段起点
    double _lzSegPx = 0.0;      // 线段起点：差分 x
    double _lzSegPy = 0.0;      // 线段起点：差分 y
    double _lzSegYaw = 0.0;     // 线段起点：差分原始 yaw
    double _lzLen = 0.0;        // 已累计的有效直线长度 m
    QVector<double> _lzObs;     // 各段反算出的 b 观测（rad）
    bool   _lzWarned = false;   // 线段不合格只告警一次

    // 最近一帧原始输入，标定刚解出时用它立即重算 offset，避免出现一帧错位
    double _lastRtkX = 0.0, _lastRtkY = 0.0, _lastRtkYaw = 0.0;
    double _lastOdomX = 0.0, _lastOdomY = 0.0, _lastOdomYaw = 0.0;
    bool   _hasLastFrame = false;

    // 标定样本累积（对偶数求和，闭式解相似变换）
    QVector<Sample> _samples;
    int    _n = 0;
    double _spx = 0.0, _spy = 0.0, _sqx = 0.0, _sqy = 0.0;
    double _spxqx = 0.0, _spyqy = 0.0, _spxqy = 0.0, _spyqx = 0.0;
    double _sqq = 0.0, _sdyaw = 0.0, _path = 0.0;
    bool   _hasPrev = false;
    double _prevPx = 0.0, _prevPy = 0.0;
    qint64 _lastSolveMs = 0;
    qint64 _lastRejectLogMs = 0;   // "暂不可解"日志限频用

    // 里程计坐标系重锚检测：遥控接管/SLAM 重定位会让 odom 单帧整块平移，
    // 这类跳变之后采的锚点与之前的不在同一坐标系，混进同一次拟合必然发散。
    bool   _hasPrevOdom = false;
    double _prevOdomX = 0.0, _prevOdomY = 0.0;
    qint64 _prevOdomMs = 0;
};

#endif  // RTK_FUSION_H
