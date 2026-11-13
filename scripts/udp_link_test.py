#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================
# 机器狗 UDP 链路检测脚本（Linux ARM / x86_64 通用）
# 依赖: 仅 Python3 标准库（Ubuntu 自带，无需 pip）
#
# 用法:
#   python3 scripts/udp_link_test.py                    # 读 config/dogterminal.ini 的 [Dog] 段
#   python3 scripts/udp_link_test.py --ip 192.168.168.168 --seconds 10
#   python3 scripts/udp_link_test.py --listen-only      # 只监听，不发心跳（观察狗是否主动推送）
#   python3 scripts/udp_link_test.py --cmd 122          # 收到首包后再下发一条指令(十进制=0x7A 站立)
#
# 行为与 DogTerminal 的 UdpDogClient 完全一致:
#   向 <狗IP>:8081 发 {"type":"heartbeat","role":"sdk"}，周期默认 50ms
#   本地绑定 0.0.0.0:8080 收状态
# 退出码: 0=正常 1=无回包 2=回包但状态不全 3=端口占用/绑定失败
# ============================================================

import argparse
import collections
import configparser
import json
import os
import platform
import select
import socket
import sys
import time

# 与 UdpDogClient 一致的"合法狗消息"类型（用于连通性判定）
VALID_TYPES = ("heartbeat", "odom_info", "imu_info", "feedback")
# 其余类型仅统计，不参与连通性判定，但同样是有效回包
EXTRA_TYPES = ("dev_info", "dog_state", "fault_info", "speed_set",
               "version", "leg_joint_info", "wifi_hotspot_rename")

RED = "\033[0;31m"
GREEN = "\033[0;32m"
YELLOW = "\033[1;33m"
NC = "\033[0m"


def pass_msg(s):
    print(f"{GREEN}[PASS]{NC} {s}")


def fail_msg(s):
    print(f"{RED}[FAIL]{NC} {s}")


def warn_msg(s):
    print(f"{YELLOW}[WARN]{NC} {s}")


def info(s):
    print(f"[INFO] {s}")


def load_ini(path):
    """读取 [Dog] 段，返回 dict（文件不存在则返回空 dict）"""
    cfg = {}
    if not path or not os.path.isfile(path):
        return cfg
    cp = configparser.ConfigParser()
    try:
        cp.read(path, encoding="utf-8")
    except Exception as e:                      # ini 格式错误时不应中断测试
        warn_msg(f"配置文件解析失败({e})，改用命令行参数/默认值")
        return cfg
    if cp.has_section("Dog"):
        d = cp["Dog"]

        def g(key, default=None):
            return d.get(key, fallback=default)

        cfg = {
            "driver": (g("Driver", "udp") or "udp").strip().lower(),
            "ip": (g("Ip") or "").strip(),
            "send_port": g("SendPort"),
            "recv_port": g("RecvPort"),
            "role": (g("Role", "sdk") or "sdk").strip().lower(),
            "heartbeat_ms": g("HeartbeatMs"),
            "timeout_ms": g("TimeoutMs"),
        }
    return cfg


def to_int(value, default):
    try:
        return int(str(value).strip())
    except (TypeError, ValueError):
        return default


def local_ip_toward(dst_ip):
    """取本机到狗地址的出口 IP（不发包，仅内核选路）"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((dst_ip, 9))
        return s.getsockname()[0]
    except OSError:
        return "?"
    finally:
        s.close()


def fmt_rpy_deg(rpy):
    try:
        import math
        return "[roll=%.2f° pitch=%.2f° yaw=%.2f°]" % tuple(
            math.degrees(float(v)) for v in rpy[:3])
    except Exception:
        return ""


def brief_sample(typ, obj):
    """输出首包的关键字段摘要，便于肉眼核对"""
    if typ == "imu_info":
        return "rpy=" + fmt_rpy_deg(obj.get("rpy", []))
    if typ == "odom_info":
        pos = obj.get("position", [])
        try:
            return "position=[%.3f, %.3f, %.3f]" % tuple(float(v) for v in pos[:3])
        except Exception:
            return "position=" + str(pos)
    if typ == "feedback":
        return "function=%s control=%s motion=%s" % (
            obj.get("function"), obj.get("control_mode"), obj.get("motion_mode"))
    if typ == "heartbeat":
        return "role=%s" % obj.get("role")
    if typ == "dev_info":
        bat = obj.get("battery") if isinstance(obj.get("battery"), dict) else obj
        return "power=%s volt=%s" % (bat.get("power"), bat.get("volt"))
    return ""


def build_heartbeat(role):
    return json.dumps({"type": "heartbeat", "role": role},
                      separators=(",", ":")).encode("utf-8")


def build_cmd(role, code):
    return json.dumps({"type": "cmd", "role": role, "cmd": code},
                      separators=(",", ":")).encode("utf-8")


def print_hints(ip, send_port, recv_port):
    print("")
    print("排查建议:")
    print(f"  1) 网络连通: ping -c 3 {ip}           # ICMP 被禁时不通属正常")
    print(f"  2) 路由出口: ip route get {ip}        # 确认走的是连狗那块网卡")
    print(f"  3) 同网段ARP: ip neigh show {ip}")
    print(f"  4) 端口占用: sudo ss -lunp | grep {recv_port}   # 有 dogterminal 在跑就先停掉")
    print(f"  5) 狗端状态: 狗是否上电、是否处于可通信模式、是否与终端在同一网段")
    print("  6) 边缘设备中转时需 ip_forward+NAT，否则跨网段 UDP 回包到不了本机")


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    default_ini = os.path.normpath(os.path.join(here, "..", "config", "dogterminal.ini"))

    ap = argparse.ArgumentParser(
        description="机器狗 UDP 链路检测（心跳 + 状态回传），默认读取 dogterminal.ini 的 [Dog] 段",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="示例:\n"
               "  python3 scripts/udp_link_test.py\n"
               "  python3 scripts/udp_link_test.py --ip 192.168.168.168 --seconds 10\n"
               "  python3 scripts/udp_link_test.py --listen-only\n"
               "  python3 scripts/udp_link_test.py --cmd 122   # 0x7A 站立（注意: 会真实控制狗）")
    ap.add_argument("--config", default=default_ini, help="dogterminal.ini 路径")
    ap.add_argument("--ip", help="机器狗 IP（默认取 ini 的 [Dog] Ip）")
    ap.add_argument("--send-port", type=int, help="控制口，默认 ini SendPort / 8081")
    ap.add_argument("--recv-port", type=int, help="状态口，默认 ini RecvPort / 8080")
    ap.add_argument("--role", help="心跳角色，默认 ini Role / sdk")
    ap.add_argument("--seconds", type=float, default=8.0, help="测试时长秒，默认 8")
    ap.add_argument("--hz", type=float, default=20.0, help="心跳频率 Hz，默认 20 (=50ms)")
    ap.add_argument("--bind", default="", help="本地绑定地址，默认 0.0.0.0")
    ap.add_argument("--listen-only", action="store_true", help="只监听，不发心跳")
    ap.add_argument("--cmd", action="append", type=lambda x: int(x, 0), default=[],
                    help="收到首包后下发指令码(支持 0x7A 写法)，可多次指定；会真实控制狗")
    ap.add_argument("--acquire", action="store_true",
                    help="收到首包后先申请 SDK 控制权(0xB3)，再下发 --cmd")
    args = ap.parse_args()

    cfg = load_ini(args.config)
    ip = args.ip or cfg.get("ip") or "192.168.168.168"
    send_port = args.send_port or to_int(cfg.get("send_port"), 8081)
    recv_port = args.recv_port or to_int(cfg.get("recv_port"), 8080)
    role = args.role or cfg.get("role") or "sdk"
    timeout_ms = to_int(cfg.get("timeout_ms"), 5000)
    hb_ms = to_int(cfg.get("heartbeat_ms"), 50)
    hz = args.hz if args.hz > 0 else (1000.0 / hb_ms if hb_ms > 0 else 20.0)

    print("=" * 60)
    print(" 机器狗 UDP 链路检测")
    print(f" 配置文件: {args.config}{'' if os.path.isfile(args.config) else ' (不存在，使用默认值)'}")
    if cfg.get("driver") and cfg["driver"] != "udp":
        warn_msg(f"ini 中 [Dog] Driver={cfg['driver']}；本脚本只验证 UDP 链路，"
                 f"请确认狗端按 UDP 模式通信")
    print(f" 狗地址  : {ip}:{send_port}(控制) -> 本机:{recv_port}(状态)")
    print(f" 本机出口: {local_ip_toward(ip)}  (uname -m={platform.machine()})")
    print(f" 角色/频率: role={role}  心跳={'关闭(listen-only)' if args.listen_only else f'{hz:g}Hz'}")
    print(f" 测试时长: {args.seconds:g}s  断连阈值: {timeout_ms}ms")
    print("=" * 60)

    # ---- 绑定状态口 ----
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind((args.bind, recv_port))
    except OSError as e:
        fail_msg(f"绑定本机 UDP {recv_port} 失败: {e}")
        info("可能原因: 已有 dogterminal/其他程序占用该端口，或权限不足")
        print_hints(ip, send_port, recv_port)
        return 3
    sock.setblocking(False)

    counts = collections.Counter()
    samples = {}          # type -> 首包摘要
    raw_first = {}        # type -> 首个原始 JSON（便于排查字段）
    sent = 0
    first_rx = None
    last_rx = None
    pending_cmds = list(args.cmd)
    acquired = not args.acquire

    def send(data):
        nonlocal sent
        try:
            sock.sendto(data, (ip, send_port))
            sent += 1
        except OSError as e:
            warn_msg(f"发送失败: {e}")

    start = time.time()
    next_hb = start
    try:
        while time.time() - start < args.seconds:
            now = time.time()
            if not args.listen_only and now >= next_hb:
                send(build_heartbeat(role))
                next_hb = now + 1.0 / hz

            rlist, _, _ = select.select([sock], [], [], 0.02)
            if not rlist:
                continue

            data, addr = sock.recvfrom(4096)
            now = time.time()
            last_rx = now
            if first_rx is None:
                first_rx = now
                info(f"首包来自 {addr[0]}:{addr[1]}，延迟 {int((now - start) * 1000)}ms")

            try:
                obj = json.loads(data.decode("utf-8", "ignore"))
                typ = obj.get("type", "unknown") if isinstance(obj, dict) else "unknown"
            except Exception:
                typ = "非JSON"
                obj = {}

            counts[typ] += 1
            if typ not in samples:
                samples[typ] = brief_sample(typ, obj) if isinstance(obj, dict) else ""
                raw_first[typ] = data[:300].decode("utf-8", "ignore")
                tail = ("  " + samples[typ]) if samples[typ] else ""
                print(f"[RX] 首次收到 {typ}{tail}")

            # 收到合法狗消息后，按需下发控制权/指令（只发一次）
            if typ in VALID_TYPES:
                if not acquired:
                    send(build_cmd(role, 0xB3))
                    info("已申请 SDK 控制权 (CMD_SDK_CONTROL_RIGHT=0xB3)")
                    acquired = True
                elif pending_cmds:
                    code = pending_cmds.pop(0)
                    send(build_cmd(role, code))
                    info(f"已下发指令 0x{code:02X}({code})")
    except KeyboardInterrupt:
        info("用户中断，提前输出结果")
    finally:
        sock.close()

    elapsed = time.time() - start
    total_rx = sum(counts.values())
    valid_rx = sum(counts[t] for t in VALID_TYPES)

    # ---- 汇总 ----
    print("")
    print("-" * 60)
    print(f"发包 {sent} 个，收包 {total_rx} 个，时长 {elapsed:.1f}s")
    if counts:
        stat = "  ".join(f"{k}={v}" for k, v in sorted(counts.items()))
        print(f"类型统计: {stat}")
    if first_rx is not None:
        print(f"首包延迟: {int((first_rx - start) * 1000)}ms   "
              f"末包距今: {int((time.time() - last_rx) * 1000)}ms")
    for typ in ("imu_info", "odom_info", "feedback", "dev_info"):
        if typ in raw_first:
            print(f"  {typ:<10} 首包原文: {raw_first[typ]}")
    print("-" * 60)

    # ---- 判定 ----
    if valid_rx == 0:
        fail_msg("未收到任何合法的狗状态消息（heartbeat/imu_info/odom_info/feedback）")
        print_hints(ip, send_port, recv_port)
        return 1

    missing = [t for t in ("imu_info", "odom_info", "feedback") if counts.get(t, 0) == 0]
    age_ms = int((time.time() - last_rx) * 1000)
    if age_ms > timeout_ms:
        warn_msg(f"最后一次回包已 {age_ms}ms 前，超过断连阈值 {timeout_ms}ms")
    else:
        pass_msg(f"链路连通，末包距今 {age_ms}ms（阈值 {timeout_ms}ms）")

    if missing:
        for t in missing:
            warn_msg(f"缺少 {t}（{ {'imu_info': '姿态/IMU', 'odom_info': '坐标/里程计', 'feedback': '模式反馈'}[t] }）")
        return 2

    pass_msg("IMU / 里程计 / 模式反馈 均已收到，状态回传完整")
    if samples.get("feedback"):
        print(f"        模式反馈: {samples['feedback']}")
    if samples.get("odom_info"):
        print(f"        坐标: {samples['odom_info']}")
    if samples.get("imu_info"):
        print(f"        姿态: {samples['imu_info']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
