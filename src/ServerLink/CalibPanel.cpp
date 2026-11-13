#include "CalibPanel.h"

#include <QDateTime>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QStringList>
#include <cmath>

#include "MqLink.h"

namespace
{

double norm360(double deg)
{
    deg = std::fmod(deg, 360.0);
    if (deg < 0.0)
        deg += 360.0;
    return deg;
}

double norm180(double deg)
{
    deg = norm360(deg);
    if (deg > 180.0)
        deg -= 360.0;
    return deg;
}

QString srcText(int src)
{
    // 与 RtkFusion::Source 对应
    if (src == 1)
        return QStringLiteral("差分");
    if (src == 2)
        return QStringLiteral("差分保持");
    return QStringLiteral("里程计");
}

}   // namespace

CalibPanel::CalibPanel(QObject *parent)
    : QObject(parent)
{
}

void CalibPanel::configure(MqLink *mq, const QString &routingKey, int minPeriodMs,
                           const Snapshot &base)
{
    _mq = mq;
    _key = routingKey;
    _minPeriodMs = qMax(500, minPeriodMs);
    _base = base;
    _hasBase = true;
}

void CalibPanel::feed(const Snapshot &s)
{
    // 漂移速率要在"上一帧静止快照"还存在时先算，再更新基准点
    updateDrift(s);

    _last = s;
    _hasLast = true;
    _lastFeedMs = s.nowMs;

    if (s.nowMs - _lastPubMs < _minPeriodMs)
        return;

    Snapshot t = s;
    t.ageMs = 0;   // 刚收到帧
    publish(t);
}

void CalibPanel::pollStale(qint64 nowMs)
{
    // 没有任何差分帧时也要有一块面板：否则服务端会停在"上次的样子"，
    // 现场看不出"是断流了还是没人动"。
    if (!_hasBase)
        return;
    if (nowMs - _lastPubMs < _minPeriodMs)
        return;

    Snapshot s = _hasLast ? _last : _base;
    s.nowMs = nowMs;
    s.ageMs = _hasLast ? (nowMs - _lastFeedMs) : 0;
    publish(s);
}

void CalibPanel::updateDrift(const Snapshot &s)
{
    // 只在"确实静止"的样本之间算漂移，运动中的 yaw 变化是腿的功劳，不是零偏
    const bool still = s.hasMap && s.staticSec >= 5.0 && s.speedMps <= 0.05;
    if (!still)
    {
        _driftPrevValid = false;
        return;
    }
    if (_driftPrevValid)
    {
        const double dtSec = static_cast<double>(s.nowMs - _driftPrevMs) / 1000.0;
        if (dtSec >= 0.5 && dtSec <= 6.0)
        {
            const double rate = norm180(s.mapYawDeg - _driftPrevYawDeg) / dtSec;
            // 一步量化噪声能有 0.1°，做 EMA 抑制，取"持续漂"而不是单帧跳
            _driftDps = _driftHas ? (0.7 * _driftDps + 0.3 * rate) : rate;
            _driftHas = true;
        }
    }
    _driftPrevYawDeg = s.mapYawDeg;
    _driftPrevMs = s.nowMs;
    _driftPrevValid = true;
}

void CalibPanel::publish(Snapshot s)
{
    const Verdict v = judge(s);

    QByteArray json = buildJson(s, v);
    if (_mq)
        _mq->publish(_key, json, QStringLiteral("application/json"));

    _lastPubMs = s.nowMs;
    ++_seq;

    // 判定变化才打印一行：控制台不刷屏，人也不用翻日志找结论
    if (v.code != _lastVerdictCode)
    {
        _lastVerdictCode = v.code;
        emit verdictChanged(v.head, v.action);
    }
}

CalibPanel::Verdict CalibPanel::judge(const Snapshot &s) const
{
    Verdict v;

    if (!s.rtkEnabled)
    {
        v.code = QStringLiteral("RTK_OFF");
        v.head = QStringLiteral("差分链路未启用（Rtk/Enabled=false）");
        v.action = QStringLiteral("当前只是纯里程计基线；要做差分标定，先把 [Rtk] Enabled 打开（改 ini 不用重编）");
    }
    else if (!s.rtkSeen)
    {
        v.code = QStringLiteral("WAIT_RTK");
        v.head = QStringLiteral("未收到任何差分帧");
        v.action = QStringLiteral("查 188 的 LA 定位服务是否在发、服务端是否往 la 路由键发帧（终端只收不发）");
    }
    else if (s.ageMs > s.holdMs)
    {
        v.code = QStringLiteral("RTK_STALE");
        v.head = QStringLiteral("差分断流 %1ms（阈值 %2ms）").arg(s.ageMs).arg(s.holdMs);
        v.action = QStringLiteral("位姿已退回里程计外推：先恢复差分链路，标定与验收动作全部暂停");
    }
    else if (!s.zeroValid)
    {
        v.code = QStringLiteral("NO_B");
        v.head = s.zeroPending ? QStringLiteral("零位 b 正在取（等静止窗口凑够帧）")
                               : QStringLiteral("航向零位 b 未取");
        v.action = QStringLiteral("摆正东、完全静止 ≥1.5s 取 b；b 一取到会立刻显示在本面板");
    }
    else if (!s.calibValid)
    {
        const int need = qMax(0, s.calibMinSamples - s.anchors);
        if (!s.autoCalib)
        {
            v.code = QStringLiteral("CALIB_OFF");
            v.head = QStringLiteral("在线标定已关闭（AutoCalib=false），当前无 k/α");
            v.action = QStringLiteral("要么把 [Rtk] AutoCalib 打开让它自己解，要么用服务端下发的标定/rtkcalib_<id>.ini");
        }
        else if (need > 0)
        {
            v.code = QStringLiteral("KEEP_ANCHORS");
            v.head = QStringLiteral("标定未解：锚点 %1/%2，路径 %3/%4m")
                         .arg(s.anchors).arg(s.calibMinSamples)
                         .arg(s.pathM, 0, 'f', 2).arg(s.calibMinPathM, 0, 'f', 2);
            v.action = QStringLiteral("继续走停补锚点：还差 %1 个（每点停 ≥1.5s，点间距 ≥0.35m，不要原地重复停）")
                           .arg(need);
        }
        else if (s.pathM < s.calibMinPathM)
        {
            v.code = QStringLiteral("KEEP_PATH");
            v.head = QStringLiteral("标定未解：点数够、直线里程不足 %1/%2m")
                         .arg(s.pathM, 0, 'f', 2).arg(s.calibMinPathM, 0, 'f', 2);
            v.action = QStringLiteral("沿正东再走 %1m，并把停点拉开（点间距 ≥1m，走直、中途别纠偏）")
                           .arg(s.calibMinPathM - s.pathM, 0, 'f', 2);
        }
        else
        {
            v.code = QStringLiteral("SOLVE_FAIL");
            v.head = QStringLiteral("样本够了但 k/α 未解出（残差或散布超限被拒）");
            v.action = QStringLiteral("沿正东重走 3~5m 直线：走直、中途别纠偏、停点拉大，再让它解一次");
        }
    }
    else if (s.rmsM > s.calibMaxRmsM)
    {
        v.code = QStringLiteral("REDO_LINE");
        v.head = QStringLiteral("标定残差 RMS %1m 超限（> %2m）")
                     .arg(s.rmsM, 0, 'f', 3).arg(s.calibMaxRmsM, 0, 'f', 2);
        v.action = QStringLiteral("这个解不可信：重走 3~5m 直线（走直、速度稳），让它重新解");
    }
    else if (!s.zeroFromLine)
    {
        v.code = QStringLiteral("LINE_B");
        v.head = QStringLiteral("b=%1° 来自建参考姿态（含摆位角误差）").arg(s.bDeg, 0, 'f', 1);
        v.action = QStringLiteral("沿正东走 3~5m 停住：用停点 A→B 的差分位移方向反算 b，b 才归到地图东");
    }
    else if (s.yawRangeDeg < 1.0)
    {
        v.code = QStringLiteral("NO_YAW");
        v.head = QStringLiteral("服务端航向恒定（观测范围 %1°）").arg(s.yawRangeDeg, 0, 'f', 1);
        v.action = QStringLiteral("差分割向不可用：位置可锚定、航向只能用里程计，把 [Rtk] YawFromRtk 置 false");
    }
    else if (!s.useForPose)
    {
        v.code = QStringLiteral("BASELINE");
        v.head = QStringLiteral("UseForPose=false：上报仍走纯里程计");
        v.action = QStringLiteral("先按基线跑；差分侧检查通过后再改 true 开回融合，别边标边开");
    }
    else if (s.dpsiValid && s.dpsiDeg > 10.0)
    {
        v.code = QStringLiteral("DPSI_CHECK");
        v.head = QStringLiteral("Δψ 校核 %1°（> 10°）").arg(s.dpsiDeg, 0, 'f', 1);
        v.action = QStringLiteral("先做四方向核对（东0°/南90°/西180°/北270°）：确认差分航向的符号与轴向，再谈标定");
    }
    else if (s.hasFused && s.speedMps < 0.05 && s.fusedVsRtkM > 0.5)
    {
        v.code = QStringLiteral("ANCHOR_MISS");
        v.head = QStringLiteral("静止对差 %1m（融合位姿没吸住差分）").arg(s.fusedVsRtkM, 0, 'f', 2);
        v.action = QStringLiteral("原地静止 5s 让它重锚；仍大就核对地图原点（mapcheck）与尺度 k，别直接开走");
    }
    else
    {
        v.code = QStringLiteral("READY");
        v.head = QStringLiteral("各环节通过：定标已解、b 已归位、静止对差 %1m")
                     .arg(s.fusedVsRtkM, 0, 'f', 2);
        v.action = QStringLiteral("沿正东直行 10m 验收：到点位置误差应 <0.3m；超了回看 RMS 与 Δψ");
    }

    // 硬闸：与"当前该做哪一步"无关，但做航向/直线验收之前必须先清掉
    QStringList gates;
    if (s.hasMap && s.staticSec >= 20.0 && _driftHas && std::fabs(_driftDps) > 0.15)
        gates << QStringLiteral("硬闸①：地图 yaw 静止漂 %1°/s（60s≈%2°）→ 做 90°/直线航向验收前必须先修陀螺零偏")
                     .arg(_driftDps, 0, 'f', 2)
                     .arg(_driftDps * 60.0, 0, 'f', 0);
    if (s.dpsiValid && s.dpsiDeg > 10.0)
        gates << QStringLiteral("硬闸②：Δψ %1° > 10° → 先四方向核对符号/轴向")
                     .arg(s.dpsiDeg, 0, 'f', 1);
    if (s.rtkSeen && s.yawRangeDeg < 1.0)
        gates << QStringLiteral("硬闸③：服务端航向恒定（范围 %1°）→ 差分割向不可用")
                     .arg(s.yawRangeDeg, 0, 'f', 1);
    v.gate = gates.isEmpty() ? QStringLiteral("无") : gates.join(QStringLiteral("；"));

    return v;
}

namespace
{

QString hzText(const CalibPanel::Snapshot &s)
{
    if (s.periodMs <= 0)
        return QStringLiteral("n/a");
    return QString::number(1000.0 / static_cast<double>(s.periodMs), 'f', 1);
}

}   // namespace

QStringList CalibPanel::buildLines(const Snapshot &s, const Verdict &v) const
{
    QStringList L;

    // 固定 8 行、顺序固定：服务端逐行贴到 8 个 label，第一次设置好就不用再动
    L << QStringLiteral("判定｜%1　%2").arg(v.code).arg(v.head);
    L << QStringLiteral("下一步｜%1").arg(v.action);
    L << QStringLiteral("差分｜%1Hz 间隔%2ms 最长%3ms 帧%4 跳变%5 重启%6 偏差均值%7°")
              .arg(hzText(s)).arg(s.periodMs).arg(s.gapMaxMs)
              .arg(s.frames).arg(s.badJumps).arg(s.restarts)
              .arg(s.biasAvgDeg, 0, 'f', 1);
    L << QStringLiteral("零位｜b=%1°（%2） 抖动=%3° 航向范围=%4°")
              .arg(s.zeroValid ? QString::number(s.bDeg, 'f', 1) : QStringLiteral("未取"))
              .arg(!s.zeroValid ? QStringLiteral("—")
                                : (s.zeroFromLine ? QStringLiteral("直线") : QStringLiteral("参考")))
              .arg(s.bSpreadDeg, 0, 'f', 1)
              .arg(s.yawRangeDeg, 0, 'f', 1);
    L << QStringLiteral("标定｜%1 k=%2 α=%3° RMS=%4m 路径=%5m 锚点=%6/%7")
              .arg(s.calibValid ? QStringLiteral("已解") : QStringLiteral("未解"))
              .arg(s.k, 0, 'f', 3).arg(s.alphaDeg, 0, 'f', 2)
              .arg(s.rmsM, 0, 'f', 3).arg(s.pathM, 0, 'f', 2)
              .arg(s.anchors).arg(s.calibMinSamples);
    L << QStringLiteral("位置｜差分(%1, %2 %3°) 里程计(%4, %5 %6°)")
              .arg(s.rtkX, 0, 'f', 2).arg(s.rtkY, 0, 'f', 2)
              .arg(s.rtkSeen ? QString::number(norm360(s.rtkYawDeg), 'f', 1)
                             : QStringLiteral("—"))
              .arg(s.mapX, 0, 'f', 2).arg(s.mapY, 0, 'f', 2)
              .arg(s.hasMap ? QString::number(norm360(s.mapYawDeg), 'f', 1)
                            : QStringLiteral("—"));
    L << QStringLiteral("融合｜(%1, %2 %3°) 对差=%4m/%5° 源=%6 速度=%7m/s 零偏=%8°/s 静止=%9s")
              .arg(s.fusedX, 0, 'f', 2).arg(s.fusedY, 0, 'f', 2)
              .arg(s.hasFused ? QString::number(norm360(s.fusedYawDeg), 'f', 1)
                              : QStringLiteral("—"))
              .arg(s.fusedVsRtkM, 0, 'f', 2).arg(s.fusedVsRtkYawDeg, 0, 'f', 1)
              .arg(srcText(s.src))
              .arg(s.speedMps, 0, 'f', 2)
              .arg(s.gyroBiasDps, 0, 'f', 2)
              .arg(s.staticSec, 0, 'f', 0);
    L << QStringLiteral("闸门｜%1").arg(v.gate);

    return L;
}

QByteArray CalibPanel::buildJson(const Snapshot &s, const Verdict &v) const
{
    const QStringList lines = buildLines(s, v);

    QJsonObject o;
    o.insert(QStringLiteral("schema"), 1);
    o.insert(QStringLiteral("type"), QStringLiteral("robotdog.calib_panel"));
    o.insert(QStringLiteral("terminal_id"), s.terminalId);
    o.insert(QStringLiteral("ts"), static_cast<double>(s.nowMs));
    o.insert(QStringLiteral("ts_iso"),
             QDateTime::fromMSecsSinceEpoch(s.nowMs).toString(Qt::ISODate));
    o.insert(QStringLiteral("seq"), static_cast<double>(_seq));
    o.insert(QStringLiteral("hold"), true);   // 同键覆盖显示：服务端只留最后一条

    QJsonObject verdict;
    verdict.insert(QStringLiteral("code"), v.code);
    verdict.insert(QStringLiteral("head"), v.head);
    verdict.insert(QStringLiteral("action"), v.action);
    verdict.insert(QStringLiteral("gate"), v.gate);
    o.insert(QStringLiteral("verdict"), verdict);

    o.insert(QStringLiteral("lines"), QJsonArray::fromStringList(lines));
    o.insert(QStringLiteral("text"), lines.join(QLatin1Char('\n')));

    QJsonObject rtk;
    rtk.insert(QStringLiteral("enabled"), s.rtkEnabled);
    rtk.insert(QStringLiteral("seen"), s.rtkSeen);
    rtk.insert(QStringLiteral("age_ms"), static_cast<double>(s.ageMs));
    rtk.insert(QStringLiteral("hold_ms"), static_cast<double>(s.holdMs));
    rtk.insert(QStringLiteral("period_ms"), static_cast<double>(s.periodMs));
    rtk.insert(QStringLiteral("gap_max_ms"), static_cast<double>(s.gapMaxMs));
    rtk.insert(QStringLiteral("frames"), static_cast<double>(s.frames));
    rtk.insert(QStringLiteral("bad_jumps"), static_cast<double>(s.badJumps));
    rtk.insert(QStringLiteral("restarts"), s.restarts);
    rtk.insert(QStringLiteral("yaw_range_deg"), s.yawRangeDeg);
    rtk.insert(QStringLiteral("bias_avg_deg"), s.biasAvgDeg);
    rtk.insert(QStringLiteral("x"), s.rtkX);
    rtk.insert(QStringLiteral("y"), s.rtkY);
    rtk.insert(QStringLiteral("yaw_deg"), norm360(s.rtkYawDeg));
    o.insert(QStringLiteral("rtk"), rtk);

    QJsonObject zero;
    zero.insert(QStringLiteral("valid"), s.zeroValid);
    zero.insert(QStringLiteral("pending"), s.zeroPending);
    zero.insert(QStringLiteral("source"),
                !s.zeroValid ? QStringLiteral("none")
                             : (s.zeroFromLine ? QStringLiteral("line")
                                               : QStringLiteral("reference")));
    zero.insert(QStringLiteral("b_deg"), s.bDeg);
    zero.insert(QStringLiteral("spread_deg"), s.bSpreadDeg);
    o.insert(QStringLiteral("zero"), zero);

    QJsonObject calib;
    calib.insert(QStringLiteral("valid"), s.calibValid);
    calib.insert(QStringLiteral("auto"), s.autoCalib);
    calib.insert(QStringLiteral("k"), s.k);
    calib.insert(QStringLiteral("alpha_deg"), s.alphaDeg);
    calib.insert(QStringLiteral("alpha_yaw_deg"), s.alphaYawDeg);
    calib.insert(QStringLiteral("rms_m"), s.rmsM);
    calib.insert(QStringLiteral("path_m"), s.pathM);
    calib.insert(QStringLiteral("anchors"), s.anchors);
    calib.insert(QStringLiteral("min_samples"), s.calibMinSamples);
    calib.insert(QStringLiteral("min_path_m"), s.calibMinPathM);
    calib.insert(QStringLiteral("max_rms_m"), s.calibMaxRmsM);
    o.insert(QStringLiteral("calib"), calib);

    QJsonObject pose;
    pose.insert(QStringLiteral("has_map"), s.hasMap);
    pose.insert(QStringLiteral("map_x"), s.mapX);
    pose.insert(QStringLiteral("map_y"), s.mapY);
    pose.insert(QStringLiteral("map_yaw_deg"), norm360(s.mapYawDeg));
    pose.insert(QStringLiteral("fused_x"), s.fusedX);
    pose.insert(QStringLiteral("fused_y"), s.fusedY);
    pose.insert(QStringLiteral("fused_yaw_deg"), norm360(s.fusedYawDeg));
    pose.insert(QStringLiteral("fused_vs_rtk_m"), s.fusedVsRtkM);
    pose.insert(QStringLiteral("fused_vs_rtk_deg"), s.fusedVsRtkYawDeg);
    pose.insert(QStringLiteral("src"), srcText(s.src));
    pose.insert(QStringLiteral("use_for_pose"), s.useForPose);
    pose.insert(QStringLiteral("speed_mps"), s.speedMps);
    pose.insert(QStringLiteral("static_sec"), s.staticSec);
    pose.insert(QStringLiteral("gyro_bias_dps"), s.gyroBiasDps);
    pose.insert(QStringLiteral("static_yaw_drift_dps"), _driftDps);
    pose.insert(QStringLiteral("dpsi_deg"), s.dpsiDeg);
    pose.insert(QStringLiteral("dpsi_valid"), s.dpsiValid);
    o.insert(QStringLiteral("pose"), pose);

    return QJsonDocument(o).toJson(QJsonDocument::Compact);
}
