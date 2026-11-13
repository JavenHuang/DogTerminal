#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
# IMU 航向漂移诊断（真狗 UDP 链路，只读不控，安全）
# 依赖: 仅 Python3 标准库
#
# 用途:
#   狗完全静止时采集 rpy / quat / gyro / odom，用 60 秒数据回答三个问题:
#     1) 静止时 yaw 是否在漂？漂多快？
#     2) rpy[2] 与 quat 解算的 yaw 是否一致（判断固件 rpy 是否可信）
#     3) gyro_z / omega_world_z 是否与 yaw 漂移同源（判断是零偏积分还是融合算法）
#
# 用法:
#   python3 scripts/imu_yaw_diag.py                    # 读 dogterminal.ini 的 [Dog] 段，跑 60s
#   python3 scripts/imu_yaw_diag.py --seconds 120      # 跑 120s
#   python3 scripts/imu_yaw_diag.py --ip 192.168.168.168
#
# 注意事项:
#   - 运行前先关掉 DogTerminal（否则 8080 端口被占用）
#   - 全程不发任何运动指令，只发心跳 + 收状态
#   - 狗必须完全静止（不要碰、不要放在会晃的平台上）
# ============================================================

import argparse
import configparser
import json
import math
import os
import select
import socket
import statistics
import sys
import time

# 与 UdpDogClient 一致的合法狗消息类型
VALID_TYPES = ("heartbeat", "odom_info", "imu_info", "feedback")

RAD2DEG = 180.0 / math.pi


def load_dog_cfg(path):
    """读取 dogterminal.ini 的 [Dog] 段，文件不存在返回空 dict"""
    cfg = {}
    if not path or not os.path.isfile(path):
        return cfg
    cp = configparser.ConfigParser()
    try:
        cp.read(path, encoding="utf-8")
    except Exception:
        return cfg
    if not cp.has_section("Dog"):
        return cfg
    d = cp["Dog"]
    cfg = {
        "ip": (d.get("Ip", fallback="") or "").strip(),
        "send_port": d.get("SendPort", fallback="8081"),
        "recv_port": d.get("RecvPort", fallback="8080"),
        "role": (d.get("Role", fallback="sdk") or "sdk").strip().lower(),
        "heartbeat_ms": d.get("HeartbeatMs", fallback="50"),
    }
    if cp.has_section("Nav"):
        cfg["joy_turn_scale"] = cp["Nav"].get("JoyTurnScale", fallback="1.5")
    return cfg


def to_int(v, default):
    try:
        return int(str(v).strip())
    except (TypeError, ValueError):
        return default


def quat_yaw(q, order):
    """四元数转 yaw（只关心绕 z 轴的偏航角）
    order="xyzw" → q=[x,y,z,w]；order="wxyz" → q=[w,x,y,z]
    返回 None 表示数据无效
    """
    if len(q) < 4:
        return None
    if order == "xyzw":
        x, y, z, w = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
    else:
        w, x, y, z = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return None
    x, y, z, w = x / n, y / n, z / n, w / n
    # yaw = atan2(2(wz+xy), 1-2(y²+z²))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def unwrap_deg(series):
    """角度序列解卷绕（消除 ±180° 跳变）"""
    out = []
    prev = None
    offset = 0.0
    for v in series:
        if prev is not None:
            d = v - prev
            if d > 180.0:
                offset -= 360.0
            elif d < -180.0:
                offset += 360.0
        out.append(v + offset)
        prev = v
    return out


def slope_per_sec(times, vals):
    """最小二乘拟合斜率（值/秒），样本不足返回 None"""
    n = len(times)
    if n < 5:
        return None
    t0 = times[0]
    xs = [t - t0 for t in times]
    mx = sum(xs) / n
    my = sum(vals) / n
    num = sum((x - mx) * (y - my) for x, y in zip(xs, vals))
    den = sum((x - mx) ** 2 for x in xs)
    if den < 1e-9:
        return None
    return num / den


def mean_or_none(vals):
    return statistics.fmean(vals) if vals else None


def angular_diff(a, b):
    """两个角度之差，归一到 [-180, 180]"""
    return ((a - b + 180.0) % 360.0) - 180.0


def pick_quat_yaw(rpy_deg, q_xyzw, q_wxyz):
    """两种四元数顺序里挑与 rpy 更接近的那个（rpy 可读时才知道用哪种顺序）"""
    if rpy_deg is None:
        return q_xyzw if q_xyzw is not None else q_wxyz
    if q_xyzw is None:
        return q_wxyz
    if q_wxyz is None:
        return q_xyzw
    return q_wxyz if abs(angular_diff(rpy_deg, q_wxyz)) < abs(angular_diff(rpy_deg, q_xyzw)) else q_xyzw


def fmt(v, unit="", nd=2, sign=False):
    if v is None:
        return "N/A"
    if sign:
        return ("%+.*f%s" % (nd, v, unit))
    return ("%.*f%s" % (nd, v, unit))


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_ini = os.path.normpath(os.path.join(here, "..", "config", "dogterminal.ini"))

    ap = argparse.ArgumentParser(description="IMU 航向漂移诊断（只读，不发运动指令）")
    ap.add_argument("--config", default=default_ini, help="dogterminal.ini 路径")
    ap.add_argument("--ip", help="机器狗 IP（默认取 ini）")
    ap.add_argument("--send-port", type=int, help="控制口，默认 ini SendPort / 8081")
    ap.add_argument("--recv-port", type=int, help="状态口，默认 ini RecvPort / 8080")
    ap.add_argument("--role", help="心跳角色，默认 ini Role / sdk")
    ap.add_argument("--seconds", type=float, default=60.0, help="采集时长秒，默认 60（--turn 模式下忽略）")
    ap.add_argument("--print-hz", type=float, default=2.0, help="实时打印频率，默认 2Hz")
    ap.add_argument("--turn", type=float, default=0.0,
                    help="轴测试模式：下发的摇杆量（如 0.5），0 表示只做静止诊断")
    ap.add_argument("--axis", type=int, default=2, choices=(0, 1, 2, 3),
                    help="摇杆轴序号（0..3），默认 2。实测本狗 1=前后平移")
    ap.add_argument("--turn-sec", type=float, default=3.0, help="每段转向持续时间，默认 3s")
    ap.add_argument("--cal-sec", type=float, default=20.0, help="转向模式下的静止零偏标定时长，默认 20s")
    ap.add_argument("--no-acquire", action="store_true",
                    help="转向模式下不自动申请 SDK 控制权（默认会申请）")
    args = ap.parse_args()

    cfg = load_dog_cfg(args.config)
    ip = args.ip or cfg.get("ip") or "192.168.168.168"
    send_port = args.send_port or to_int(cfg.get("send_port"), 8081)
    recv_port = args.recv_port or to_int(cfg.get("recv_port"), 8080)
    role = args.role or cfg.get("role") or "sdk"
    hb_ms = to_int(cfg.get("heartbeat_ms"), 50)

    turn_val = max(0.0, float(args.turn))
    turn_sec = max(1.0, float(args.turn_sec))
    cal_sec = max(5.0, float(args.cal_sec))
    use_turn = turn_val > 0.0

    def phase_at(elapsed):
        """返回 (阶段名, 旋转摇杆量)。静止诊断模式只有一个阶段"""
        if not use_turn:
            return ("采集", 0.0)
        if elapsed < cal_sec:
            return ("标定", 0.0)
        t = elapsed - cal_sec
        if t < 2.0:
            return ("静置", 0.0)
        t -= 2.0
        if t < turn_sec:
            return ("正转", +turn_val)
        t -= turn_sec
        if t < 2.0:
            return ("静置", 0.0)
        t -= 2.0
        if t < turn_sec:
            return ("反转", -turn_val)
        t -= turn_sec
        return ("静置", 0.0)

    total_sec = (cal_sec + 2.0 + turn_sec + 2.0 + turn_sec + 2.0) if use_turn else args.seconds

    print("=" * 68)
    if use_turn:
        print(" IMU 零偏标定 + 摇杆轴功能测试（会下发运动指令）")
    else:
        print(" IMU 航向漂移诊断（只读，不发运动指令）")
    print(" 狗地址  : %s:%d(控制) -> 本机:%d(状态)" % (ip, send_port, recv_port))
    print(" 角色/心跳: role=%s  %dms" % (role, hb_ms))
    if use_turn:
        print(" 测试轴  : joystick[%d]，正段 %+.2f / 反段 %+.2f，每段 %.0fs"
              % (args.axis, turn_val, -turn_val, turn_sec))
        print(" 阶段    : 标定%.0fs → 静置2s → 轴正 %.0fs → 静置2s → 轴反 %.0fs → 静置2s"
              % (cal_sec, turn_sec, turn_sec))
        print(" 总时长  : %.0fs    控制权: %s"
              % (total_sec, "不申请(--no-acquire)" if args.no_acquire else "自动申请"))
        print(" 要求    : 标定阶段狗完全静止；测试阶段周围留出 1m 空间")
    else:
        print(" 采集时长: %.0fs    实时打印: %.1fHz" % (args.seconds, args.print_hz))
        print(" 要求    : 狗必须完全静止，且 DogTerminal 已关闭（释放 %d 端口）" % recv_port)
    print("=" * 68)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("0.0.0.0", recv_port))
    except OSError as e:
        print("[FAIL] 绑定本机 UDP %d 失败: %s" % (recv_port, e))
        print("       可能 DogTerminal / 其他上位机正在运行，先停掉再试")
        return 3
    sock.setblocking(False)

    hb = json.dumps({"type": "heartbeat", "role": role}, separators=(",", ":")).encode("utf-8")

    def send_json(obj):
        try:
            sock.sendto(json.dumps(obj, separators=(",", ":")).encode("utf-8"), (ip, send_port))
        except OSError:
            pass

    def send_cmd(code):
        send_json({"type": "cmd", "role": role, "cmd": code})

    def send_remote(amount):
        # 把指令量写到指定轴，其余轴置零
        js = [0.0, 0.0, 0.0, 0.0]
        js[int(args.axis)] = float(amount)
        send_json({"type": "remote", "role": role,
                   "joystick": js, "button": [0.0] * 14})

    if use_turn and not args.no_acquire:
        print("[INFO] 申请 SDK 控制权(0xB3) + 移动模式(0x8A) ...")
        for _ in range(3):
            send_cmd(0xB3)
            time.sleep(0.3)
        for _ in range(2):
            send_cmd(0x8A)
            time.sleep(0.3)

    # 采样序列
    imu_t, rpy_yaw, quat_yaw_xyzw, quat_yaw_wxyz, gyro_z = [], [], [], [], []
    rpy_roll, rpy_pitch = [], []
    odom_t, odom_pos, odom_omega_wz, odom_omega_bz, odom_vbody_x = [], [], [], [], []
    phase_samples = []          # (阶段名, t, rpy_yaw_deg, gyro_z)

    latest = {"rpy": None, "qxyzw": None, "qwxyz": None, "gyro": None,
              "pos": None, "owz": None, "obz": None, "vx": None}
    counts = 0
    first_rx = None
    start = time.time()
    next_hb = start
    next_remote = start
    next_print = start + 1.0 / max(args.print_hz, 0.1)
    last_phase = None
    remote_ms = 50

    try:
        while time.time() - start < total_sec:
            now = time.time()
            elapsed = now - start
            phase_name, phase_turn = phase_at(elapsed)
            if phase_name != last_phase:
                last_phase = phase_name
                print("[PHASE] t=%5.1fs → %s（axis%d=%+0.2f）"
                      % (elapsed, phase_name, args.axis, phase_turn))

            if use_turn and now >= next_remote:
                send_remote(phase_turn)
                next_remote = now + remote_ms / 1000.0

            if now >= next_hb:
                try:
                    sock.sendto(hb, (ip, send_port))
                except OSError as e:
                    print("[WARN] 心跳发送失败: %s" % e)
                next_hb = now + hb_ms / 1000.0

            rlist, _, _ = select.select([sock], [], [], 0.02)
            if rlist:
                while True:
                    try:
                        data, addr = sock.recvfrom(8192)
                    except OSError:
                        break
                    t = time.time() - start
                    if first_rx is None:
                        first_rx = t
                        print("[INFO] 首包来自 %s:%d，延迟 %dms" % (addr[0], addr[1], int(t * 1000)))
                    try:
                        obj = json.loads(data.decode("utf-8", "ignore"))
                        typ = obj.get("type", "")
                    except Exception:
                        continue
                    if typ not in VALID_TYPES:
                        continue
                    counts += 1

                    if typ == "imu_info":
                        rpy = obj.get("rpy") or []
                        q = obj.get("quat") or []
                        g = obj.get("gyro") or []
                        gv = float(g[2]) if len(g) >= 3 else None
                        if len(rpy) >= 3:
                            ry = float(rpy[2]) * RAD2DEG
                            imu_t.append(t)
                            rpy_yaw.append(ry)
                            rpy_roll.append(float(rpy[0]) * RAD2DEG)
                            rpy_pitch.append(float(rpy[1]) * RAD2DEG)
                            latest["rpy"] = ry
                            phase_samples.append((phase_name, t, ry, gv, latest["pos"]))
                        q1 = quat_yaw(q, "xyzw")
                        q2 = quat_yaw(q, "wxyz")
                        if q1 is not None:
                            quat_yaw_xyzw.append(q1 * RAD2DEG)
                            latest["qxyzw"] = q1 * RAD2DEG
                        if q2 is not None:
                            quat_yaw_wxyz.append(q2 * RAD2DEG)
                            latest["qwxyz"] = q2 * RAD2DEG
                        if gv is not None:
                            gyro_z.append(gv)
                            latest["gyro"] = gv

                    elif typ == "odom_info":
                        pos = obj.get("position") or []
                        ow = obj.get("omega_world") or []
                        ob = obj.get("omega_body") or []
                        vb = obj.get("v_body") or []
                        if len(pos) >= 2:
                            odom_t.append(t)
                            odom_pos.append((float(pos[0]), float(pos[1])))
                            latest["pos"] = (float(pos[0]), float(pos[1]))
                            odom_vbody_x.append(float(vb[0]) if vb else 0.0)
                            latest["vx"] = float(vb[0]) if vb else 0.0
                        if len(ow) >= 3:
                            odom_omega_wz.append(float(ow[2]))
                            latest["owz"] = float(ow[2])
                        if len(ob) >= 3:
                            odom_omega_bz.append(float(ob[2]))
                            latest["obz"] = float(ob[2])

            now = time.time()
            if now >= next_print:
                next_print = now + 1.0 / max(args.print_hz, 0.1)
                pos = latest["pos"]
                pos_txt = "(%.2f, %.2f)" % pos if pos else "N/A"
                qy = pick_quat_yaw(latest["rpy"], latest["qxyzw"], latest["qwxyz"])
                print("[t=%5.1fs|%s] rpy_yaw=%s  quat_yaw=%s  gyro_z=%s  "
                      "odom_wz=%s  vx=%s  pos=%s"
                      % (now - start, phase_name,
                         fmt(latest["rpy"], "°", 2, True),
                         fmt(qy, "°", 2, True),
                         fmt(latest["gyro"], "", 4),
                         fmt(latest["owz"], "", 4),
                         fmt(latest["vx"], "", 2, True),
                         pos_txt))

    except KeyboardInterrupt:
        print("[INFO] 用户中断，按已采集数据出结果")

    finally:
        # 退出前连发零速，确保狗不会保持最后一条转向指令
        if use_turn:
            for _ in range(5):
                send_remote(0.0)
                time.sleep(0.05)
        sock.close()

    elapsed = time.time() - start
    print("")
    print("-" * 68)
    print(" 统计（%.1fs，合法消息 %d 个，imu 样本 %d，odom 样本 %d）"
          % (elapsed, counts, len(imu_t), len(odom_t)))
    print("-" * 68)

    if not imu_t:
        print("[FAIL] 没收到 imu_info，无法分析（检查 IP/端口/狗是否在线）")
        return 1

    t0 = imu_t[0]
    ts = [t - t0 for t in imu_t]

    rpy_un = unwrap_deg(rpy_yaw)
    rpy_drift = slope_per_sec(ts, rpy_un)
    total_drift = rpy_un[-1] - rpy_un[0]

    q_un = unwrap_deg(quat_yaw_xyzw) if quat_yaw_xyzw else []
    q_drift = slope_per_sec(ts, q_un) if q_un else None
    w_un = unwrap_deg(quat_yaw_wxyz) if quat_yaw_wxyz else []
    w_drift = slope_per_sec(ts, w_un) if w_un else None

    # rpy 与 quat 的一致性（只用同长度的尾部样本，避免序列不同步）
    n_min = min(len(rpy_yaw), len(quat_yaw_xyzw))
    diff_xyzw = ([abs(((rpy_yaw[-n_min + i] - quat_yaw_xyzw[-n_min + i] + 180.0)
                       % 360.0) - 180.0) for i in range(n_min)] if n_min > 0 else [])
    n_min_w = min(len(rpy_yaw), len(quat_yaw_wxyz))
    diff_wxyz = ([abs(((rpy_yaw[-n_min_w + i] - quat_yaw_wxyz[-n_min_w + i] + 180.0)
                       % 360.0) - 180.0) for i in range(n_min_w)] if n_min_w > 0 else [])

    disp = 0.0
    if len(odom_pos) >= 2:
        for i in range(1, len(odom_pos)):
            dx = odom_pos[i][0] - odom_pos[i - 1][0]
            dy = odom_pos[i][1] - odom_pos[i - 1][1]
            disp += math.hypot(dx, dy)

    gyro_mean = mean_or_none(gyro_z)
    owz_mean = mean_or_none(odom_omega_wz)
    obz_mean = mean_or_none(odom_omega_bz)
    vx_mean = mean_or_none(odom_vbody_x)

    # 注意: rpy_yaw / quat_yaw_* 序列存的都是"度"，斜率单位就是 °/s
    rpy_drift_deg = rpy_drift
    rpy_drift_rad = (rpy_drift_deg * math.pi / 180.0) if rpy_drift_deg is not None else None

    print(" rpy yaw      : 起 %s → 终 %s   总漂移 %s   速率 %s (%s)"
          % (fmt(rpy_un[0], "°", 2, True), fmt(rpy_un[-1], "°", 2, True),
             fmt(total_drift, "°", 2, True),
             fmt(rpy_drift_deg, "°/s", 3, True) if rpy_drift_deg is not None else "N/A",
             fmt(rpy_drift_deg * 60.0, "°/min", 2, True) if rpy_drift_deg is not None else "N/A"))
    print(" quat yaw(xyzw): 速率 %s   quat yaw(wxyz): 速率 %s"
          % (fmt(q_drift, "°/s", 3, True) if q_drift is not None else "N/A",
             fmt(w_drift, "°/s", 3, True) if w_drift is not None else "N/A"))
    if diff_xyzw:
        print(" rpy↔quat(xyzw): 平均偏差 %.2f°  最大 %.2f°"
              % (statistics.fmean(diff_xyzw), max(diff_xyzw)))
    if diff_wxyz:
        print(" rpy↔quat(wxyz): 平均偏差 %.2f°  最大 %.2f°"
              % (statistics.fmean(diff_wxyz), max(diff_wxyz)))
    print(" gyro_z 均值  : %s    odom omega_world_z 均值: %s    omega_body_z 均值: %s"
          % (fmt(gyro_mean, " rad/s", 4, True),
             fmt(owz_mean, " rad/s", 4, True),
             fmt(obz_mean, " rad/s", 4, True)))
    print(" 位移总长     : %.3f m   机体系 vx 均值: %s"
          % (disp, fmt(vx_mean, " m/s", 3, True)))
    print(" roll/pitch 均值: %s / %s"
          % (fmt(mean_or_none(rpy_roll), "°", 2, True),
             fmt(mean_or_none(rpy_pitch), "°", 2, True)))

    results = {}
    if use_turn and phase_samples:
        cal = [s for s in phase_samples if s[0] == "标定" and s[3] is not None]
        bias_rad = statistics.fmean([s[3] for s in cal]) if cal else 0.0
        bias_deg = bias_rad * RAD2DEG
        base = unwrap_deg([s[2] - bias_deg * s[1] for s in phase_samples])
        print("")
        print("-" * 68)
        print(" 摇杆轴功能测试（axis=%d，标定零偏 %+.4f rad/s = %+.3f°/s）"
              % (args.axis, bias_rad, bias_deg))
        print("-" * 68)
        try:
            jts = float(cfg.get("joy_turn_scale") or 1.5)
        except (TypeError, ValueError):
            jts = 1.5
        print(" 参考: 若该轴是旋转轴，摇杆 %.2f × JoyTurnScale %.2f × %.1fs ≈ %+.1f°"
              % (turn_val, jts, turn_sec, turn_val * jts * turn_sec * RAD2DEG))
        for ph in ("正转", "反转"):
            idx = [i for i, s in enumerate(phase_samples) if s[0] == ph]
            if len(idx) < 5:
                print(" [WARN] 阶段 %s 样本不足" % ph)
                continue
            delta = base[idx[-1]] - base[idx[0]]
            dur = phase_samples[idx[-1]][1] - phase_samples[idx[0]][1]
            p0 = phase_samples[idx[0]][4]
            p1 = phase_samples[idx[-1]][4]
            dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1]) if (p0 and p1) else 0.0
            results[ph] = (delta, dist, dur)
            print(" %s: yaw变化 %+.2f°   位移 %.2f m   历时 %.2fs   平均转向率 %+.2f°/s"
                  % (ph, delta, dist, dur,
                     delta / dur if dur > 1e-6 else 0.0))
        print("")
        if "正转" in results and "反转" in results:
            (d1, s1, _), (d2, s2, _) = results["正转"], results["反转"]
            m = max(abs(d1), abs(d2))
            if m >= 3.0:
                gain = abs(d1) * math.pi / 180.0 / max(turn_sec, 0.1) / max(turn_val, 0.01)
                print(" [PASS] axis=%d 能产生转向（%+.1f° / %+.1f°）→ 这是旋转轴"
                      % (args.axis, d1, d2))
                print("        实测 JoyTurnScale ≈ %.2f（配置值 %.2f）" % (gain, jts))
            elif max(s1, s2) > 0.10:
                print(" [INFO] axis=%d 是平移轴：位移 %.2f/%.2f m，但 yaw 几乎不变（%+.1f° / %+.1f°）"
                      % (args.axis, s1, s2, d1, d2))
                print("        → 旋转轴应该是别的序号，继续用 --axis 试其余轴")
            else:
                print(" [FAIL] axis=%d 无位移也无转动 → 该轴未被狗执行"
                      % args.axis)

    print("")
    print("-" * 68)
    print(" 判读")
    print("-" * 68)

    ok = True
    if disp > 0.10 or abs(vx_mean or 0.0) > 0.05:
        ok = False
        print(" [WARN] 采集期间狗有位移（%.3f m）或速度非零，漂移估计不可信，请重测" % disp)
    else:
        print(" [PASS] 采集期间狗静止（位移 %.3f m，vx %.3f m/s）" % (disp, vx_mean or 0.0))

    if rpy_drift_deg is None:
        print(" [WARN] 样本太少，无法拟合漂移速率")
    else:
        rate = rpy_drift_deg
        if abs(rate) > 0.05:
            print(" [WARN] 静止时 yaw 漂移 %.3f°/s（1 分钟 %.1f°，5 分钟 %.1f°）→ 存在恒定零偏"
                  % (rate, rate * 60.0, rate * 300.0))
        else:
            print(" [PASS] 静止时 yaw 漂移 %.3f°/s，可忽略" % rate)

    # rpy 与 quat 一致性
    if diff_xyzw and diff_wxyz:
        if statistics.fmean(diff_xyzw) <= statistics.fmean(diff_wxyz):
            print(" [INFO] quat 顺序判定为 xyzw（与 rpy 平均偏差 %.2f°）" % statistics.fmean(diff_xyzw))
            best_diff = statistics.fmean(diff_xyzw)
        else:
            print(" [INFO] quat 顺序判定为 wxyz（与 rpy 平均偏差 %.2f°）" % statistics.fmean(diff_wxyz))
            best_diff = statistics.fmean(diff_wxyz)
        if best_diff > 2.0:
            print(" [WARN] rpy 与 quat 解算的 yaw 偏差 %.2f° > 2°，固件 rpy 解算可疑" % best_diff)
        else:
            print(" [PASS] rpy 与 quat 解算的 yaw 一致（偏差 %.2f°），字段格式无误" % best_diff)

    # gyro / odom 角速度与 yaw 漂移的同源性（统一换成 rad/s 比较）
    if gyro_mean is not None and rpy_drift_rad is not None and abs(rpy_drift_rad) > 1e-6:
        ratio = gyro_mean / rpy_drift_rad
        print(" [INFO] gyro_z ÷ yaw漂移 = %.2f"
              "（≈1 → gyro 为 rad/s 且 yaw 就是它的纯积分）" % ratio)
    if owz_mean is not None and rpy_drift_rad is not None:
        same = abs(owz_mean - rpy_drift_rad) < 0.005
        print(" [INFO] odom omega_world_z %+.4f rad/s vs yaw 漂移 %+.4f rad/s → %s"
              % (owz_mean, rpy_drift_rad,
                 "同源（三者都是同一路陀螺，积分 omega 同样会漂）" if same else
                 "不同源（omega 更可信，可用它积分航向）"))

    print("")
    print(" 建议:")
    bias_rad = gyro_mean if gyro_mean is not None else rpy_drift_rad
    if bias_rad is not None and abs(bias_rad) > 1e-5:
        bias_deg = bias_rad * RAD2DEG
        print("   零偏扣除: driftBias = %+.5f rad/s (= %+.3f°/s)" % (bias_rad, bias_deg))
        print("   最简实现(无需积分): yawCorr = rpy[2] - driftBias_deg * (t - t0)")
        print("   其中 driftBias_deg = %+.3f°/s，t0 取开机静止标定结束时刻" % bias_deg)
    print("   每个任务开始时把修正后的 yaw 对齐到当前值，只用任务内增量")
    if ok and rpy_drift_deg is not None and abs(rpy_drift_deg) > 0.05:
        print("   结论: yaw 是相对航向且带恒定零偏；扣除零偏后短时可用（10~30s 误差 <1°），")
        print("         长任务建议改用横向误差(cross-track)闭环，不依赖 yaw 绝对精度")
    return 0


if __name__ == "__main__":
    sys.exit(main())
