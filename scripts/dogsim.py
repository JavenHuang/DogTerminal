#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dogsim.py — 机器狗 SDK 模拟器（UDP + JSON）

模拟真实机器狗的 UDP JSON 协议，用于验证 DogTerminal 终端的 6 项功能：
  1. 终端连接服务器（RabbitMQ）
  2. 终端连接机器狗（UDP）
  3. 状态转换→服务器
  4. 姿态控制→狗
  5. 机动控制（坐标点）导航
  6. 多终端多狗

协议对齐 Zsibot L1 SDK：
  - 终端发送：{"type":"heartbeat","role":"sdk"} 到控制口（默认 8081）
  - 终端发送：{"type":"cmd","role":"sdk","cmd":122} 到控制口
  - 终端发送：{"type":"remote","role":"sdk","joystick":[sh,f,rot,h],"button":[...]} 到控制口
    （轴序按真狗实测：axis0 横向(+右)、axis1 前后(+后退)、axis2 旋转(+右转)）
  - 本模拟器发送：{"type":"imu_info",...} / {"type":"odom_info",...} 等到状态口（默认 8080）
  - 心跳 50ms，状态 50Hz，断连 5s

用法：
  python dogsim.py --send-port 8081 --recv-port 8080

  # 多狗验证（第二只狗）
  python dogsim.py --send-port 8083 --recv-port 8082 --name dog_2
"""

import asyncio
import json
import math
import socket
import time
import argparse
import sys
from dataclasses import dataclass, field
from typing import Optional, Tuple, List

# ------------------------------------------------------------
# 命令码常量（对齐 ZsibotDefs.h）
# ------------------------------------------------------------
CMD_NULL = 0x00
CMD_EMERGENCY_STOP = 0x5A      # 90  紧急停止
CMD_STAND_UP = 0x7A            # 122 站立
CMD_SIT_DOWN = 0x6A            # 106 趴下
CMD_MOVE_MODE = 0x8A           # 138 移动模式
CMD_BALANCE_STAND_MODE = 0x9A  # 154 平衡站立
CMD_LOCK_MODE = 0x9B           # 155 锁定
CMD_SLOW_SPEED = 0xAE          # 174 慢速
CMD_NORMAL_SPEED = 0xAF        # 175 正常
CMD_FAST_SPEED = 0xB0          # 176 快速
CMD_SDK_CONTROL_RIGHT = 0xB3   # 179 申请SDK控制权

CMD_NAMES = {
    CMD_NULL: "NULL",
    CMD_EMERGENCY_STOP: "EMERGENCY_STOP",
    CMD_STAND_UP: "STAND_UP",
    CMD_SIT_DOWN: "SIT_DOWN",
    CMD_MOVE_MODE: "MOVE_MODE",
    CMD_BALANCE_STAND_MODE: "BALANCE_STAND_MODE",
    CMD_LOCK_MODE: "LOCK_MODE",
    CMD_SLOW_SPEED: "SLOW_SPEED",
    CMD_NORMAL_SPEED: "NORMAL_SPEED",
    CMD_FAST_SPEED: "FAST_SPEED",
    CMD_SDK_CONTROL_RIGHT: "SDK_CONTROL_RIGHT",
}


@dataclass
class DogState:
    """模拟狗内部状态（局部坐标系，上电原点）"""
    # 位置
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    # 速度（世界系 = 局部系，简化）
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    # 姿态
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    # 角速度
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    # 电池
    battery: float = 85.0
    # 模式
    control_mode: int = 0      # 0=无控制, 1=remote, 2=sdk
    function_mode: int = 0     # 0=init, 1=move, 2=stand
    speed_level: int = 1       # 1=low, 2=mid, 3=high
    # 协议上报字段（对齐真狗 feedback 的 function / control_mode 取值）
    sdk_granted: bool = False  # 是否已授予 SDK 控制权 → function=sdk
    posture: str = "init"      # 姿态 → control_mode: stand_up/sit_down/move/lock/...
    fault_count: int = 0
    # 连接状态
    connected: bool = False
    last_heartbeat: float = 0.0
    # 导航目标
    target_x: Optional[float] = None
    target_y: Optional[float] = None
    # 遥控输入
    remote_fwd: float = 0.0
    remote_turn: float = 0.0
    remote_shift: float = 0.0
    # 统计
    cmd_count: int = 0
    msg_count: int = 0


class DogSimProtocol(asyncio.DatagramProtocol):
    """UDP 协议处理器：接收终端指令"""

    def __init__(self, sim: "DogSim"):
        self.sim = sim

    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, data: bytes, addr: Tuple[str, int]):
        try:
            msg = json.loads(data.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            print(f"[DogSim] JSON 解析失败: {e}, raw={data[:80]}")
            return

        msg_type = msg.get("type", "")
        self.sim.state.msg_count += 1

        if msg_type == "heartbeat":
            self.sim.on_heartbeat(msg, addr)
        elif msg_type == "cmd":
            self.sim.on_cmd(msg)
        elif msg_type == "remote":
            print(f"[DogSim] 收到 remote: {msg}")
            self.sim.on_remote(msg)
        else:
            print(f"[DogSim] 未知消息类型: {msg_type}, keys={list(msg.keys())}")
        #print(f"[DogSim] 消息类型: {msg_type}, keys={list(msg.keys())}")
        # 回复 response（简单确认）
        resp = {
            "type": "response",
            "code": 0,
            "timestamp": int(time.time() * 1000),
        }
        self.transport.sendto(json.dumps(resp).encode(), addr)


class DogSim:
    """机器狗模拟器主类"""

    def __init__(self, name: str, send_port: int, recv_port: int,
                 ip: str = "127.0.0.1", terminal_ip: str = "127.0.0.1"):
        self.name = name
        self.ip = ip
        self.send_port = send_port      # 终端发到这里（控制口）
        self.recv_port = recv_port      # 终端从这里收（状态口）
        self.terminal_ip = terminal_ip  # 终端 IP（发送状态的目标）
        self.state = DogState()
        self.state.last_heartbeat = time.time()
        self._running = False
        self._terminal_addr = None  # 终端地址（ip, port），从收到的心跳学习

    # --------------------------------------------------------
    # 指令处理（终端 → 狗）
    # --------------------------------------------------------

    def on_heartbeat(self, msg: dict, addr: Tuple[str, int] = None):
        """收到终端心跳"""
        self.state.last_heartbeat = time.time()
        if addr and not self._terminal_addr:
            self._terminal_addr = addr
            print(f"[{self.name}] 记录终端地址: {addr}")
        if not self.state.connected:
            self.state.connected = True
            print(f"[{self.name}] 终端已连接 (heartbeat)")

    def on_cmd(self, msg: dict):
        """收到离散命令"""
        cmd = int(msg.get("cmd", 0))
        role = msg.get("role", "unknown")
        self.state.cmd_count += 1
        cmd_name = CMD_NAMES.get(cmd, f"UNKNOWN({cmd})")

        print(f"[{self.name}] 收到命令 #{self.state.cmd_count}: "
              f"0x{cmd:02X} ({cmd_name}) role={role}")

        if cmd == CMD_SDK_CONTROL_RIGHT:
            self.state.control_mode = 2  # sdk
            self.state.function_mode = 3  # FM_SDK，终端靠这个判断控制权成功
            self.state.sdk_granted = True
            print(f"[{self.name}]   → SDK 控制权已授予 (function_mode=3)")

        elif cmd == CMD_STAND_UP:
            self.state.function_mode = 2  # stand
            self.state.posture = "stand_up"
            self.state.z = 0.35  # 站立高度
            print(f"[{self.name}]   → 站立完成 z={self.state.z}")

        elif cmd == CMD_SIT_DOWN:
            self.state.function_mode = 0
            self.state.posture = "sit_down"
            self.state.z = 0.10
            self.state.vx = self.state.vy = 0.0
            print(f"[{self.name}]   → 趴下")

        elif cmd == CMD_MOVE_MODE:
            self.state.function_mode = 1  # move
            self.state.posture = "move"
            print(f"[{self.name}]   → 进入移动模式")

        elif cmd == CMD_BALANCE_STAND_MODE:
            self.state.function_mode = 2
            self.state.posture = "balance_stand"
            print(f"[{self.name}]   → 平衡站立模式")

        elif cmd == CMD_LOCK_MODE:
            self.state.function_mode = 0
            self.state.posture = "lock"
            self.state.vx = self.state.vy = 0.0
            print(f"[{self.name}]   → 锁定模式")

        elif cmd == CMD_EMERGENCY_STOP:
            self.state.posture = "emergency"
            self.state.vx = self.state.vy = 0.0
            self.state.target_x = None
            self.state.target_y = None
            self.state.remote_fwd = 0.0
            self.state.remote_turn = 0.0
            print(f"[{self.name}]   → 紧急停止")

        elif cmd == CMD_SLOW_SPEED:
            self.state.speed_level = 1
            print(f"[{self.name}]   → 慢速档")

        elif cmd == CMD_NORMAL_SPEED:
            self.state.speed_level = 2
            print(f"[{self.name}]   → 正常速度档")

        elif cmd == CMD_FAST_SPEED:
            self.state.speed_level = 3
            print(f"[{self.name}]   → 快速档")

    def on_remote(self, msg: dict):
        """收到遥控指令

        轴序按真狗实测（与 SDK 文档 [前后, 旋转, 左右, 头部] 不同）：
            axis0 = 横向平移，+右
            axis1 = 前后平移，+后退
            axis2 = 原地旋转，+航向减小(右转)
        这里换算回内部语义（fwd +前进 / turn +左转 / shift +左），与 UdpDogClient 保持一致
        """
        joystick = msg.get("joystick", [0, 0, 0, 0])
        if len(joystick) >= 4:
            self.state.remote_shift = float(joystick[0])  # 横向平移
            self.state.remote_fwd = -float(joystick[1])   # 前后（真狗 +后退）
            self.state.remote_turn = -float(joystick[2])  # 旋转（真狗 +右转）
            # joystick[3] = head，暂不使用
            
            # 打印遥控指令（节流：变化时才打印）
            if (abs(self.state.remote_fwd) > 0.01 or abs(self.state.remote_turn) > 0.01 or 
                abs(self.state.remote_shift) > 0.01):
                print(f"[{self.name}] 遥控: fwd={self.state.remote_fwd:.2f} "
                      f"turn={self.state.remote_turn:.2f} shift={self.state.remote_shift:.2f}")

    # --------------------------------------------------------
    # 状态发送（狗 → 终端）
    # --------------------------------------------------------

    async def send_state_loop(self):
        """周期发送状态消息（50Hz）"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"[{self.name}] 状态发送循环启动")
        
        # 确定发送目标：优先用学习到的终端地址，否则用配置地址
        target_ip = self.ip
        if self._terminal_addr:
            target_ip = self._terminal_addr[0]
            print(f"[{self.name}] 使用终端地址: {target_ip}")
        
        while self._running:
            try:
                now = time.time()
                dt = 0.02  # 50Hz

                # 简单运动学模拟
                self._update_motion(dt)

                # 发送 heartbeat（狗→终端，让终端确认连接）
                hb_msg = {
                    "type": "heartbeat",
                    "timestamp": int(now * 1000),
                }
                sock.sendto(json.dumps(hb_msg).encode(), (target_ip, self.recv_port))

                # 发送 imu_info
                imu_msg = {
                    "type": "imu_info",
                    "quat": [1.0, 0.0, 0.0, 0.0],  # 简化：单位四元数
                    "rpy": [round(self.state.roll, 4),
                            round(self.state.pitch, 4),
                            round(self.state.yaw, 4)],
                    "gyro": [round(self.state.gyro_x, 4),
                             round(self.state.gyro_y, 4),
                             round(self.state.gyro_z, 4)],
                    "acc": [0.0, 0.0, 9.8],
                    "timestamp": int(now * 1000),
                }
                sock.sendto(json.dumps(imu_msg).encode(), (target_ip, self.recv_port))

                # 计算机体系速度（v_body = R(-yaw) * v_world）
                cos_yaw = math.cos(self.state.yaw)
                sin_yaw = math.sin(self.state.yaw)
                v_body_x = cos_yaw * self.state.vx + sin_yaw * self.state.vy
                v_body_y = -sin_yaw * self.state.vx + cos_yaw * self.state.vy

                # 发送 odom_info
                odom_msg = {
                    "type": "odom_info",
                    "position": [round(self.state.x, 4),
                                 round(self.state.y, 4),
                                 round(self.state.z, 4)],
                    "v_world": [round(self.state.vx, 4),
                                round(self.state.vy, 4),
                                round(self.state.vz, 4)],
                    "omega_world": [0.0, 0.0, round(self.state.gyro_z, 4)],
                    "v_body": [round(v_body_x, 4),
                               round(v_body_y, 4),
                               round(self.state.vz, 4)],
                    "omega_body": [0.0, 0.0, round(self.state.gyro_z, 4)],
                    "timestamp": int(now * 1000),
                }
                sock.sendto(json.dumps(odom_msg).encode(), (target_ip, self.recv_port))

                # 发送 feedback（字段取值对齐真狗协议：
                #   function: remote/trace/sdk/general_sdk/roamer
                #   control_mode: stand_up/sit_down/lock/emergency/move/balance_stand/init）
                moving = abs(self.state.vx) + abs(self.state.vy) > 0.01
                fb_msg = {
                    "type": "feedback",
                    "function": "sdk" if self.state.sdk_granted else "remote",
                    "control_mode": self.state.posture,
                    "motion_mode": "running" if moving else "rest",
                    "motion_type": "unknown",
                    "timestamp": int(now * 1000),
                }
                sock.sendto(json.dumps(fb_msg).encode(), (target_ip, self.recv_port))

                # 发送 speed_set（终端从该报文取速度档位）
                speed_msg = {
                    "type": "speed_set",
                    "speed_set": {1: "low", 2: "mid", 3: "high"}.get(
                        self.state.speed_level, "mid"),
                    "timestamp": int(now * 1000),
                }
                sock.sendto(json.dumps(speed_msg).encode(), (target_ip, self.recv_port))

                # 发送 dev_info（电池）
                dev_msg = {
                    "type": "dev_info",
                    "battery": {
                        "current": 1.2,
                        "error": 0,
                        "power": int(self.state.battery),
                        "temp": 32.5,
                        "volt": 24.0,
                    },
                    "timestamp": int(now * 1000),
                }
                sock.sendto(json.dumps(dev_msg).encode(), (self.ip, self.recv_port))

                # 发送 dog_state
                dog_msg = {
                    "type": "dog_state",
                    "power": int(self.state.battery),
                    "temp": 32.5,
                    "speed": round(math.hypot(self.state.vx, self.state.vy), 3),
                    "sn": f"SIM{self.name.upper()}001",
                    "dev_name": self.name,
                    "ssid": "DogSim_WiFi",
                    "model": 1,
                    "timestamp": int(now * 1000),
                }
                sock.sendto(json.dumps(dog_msg).encode(), (self.ip, self.recv_port))

                # 电池缓慢消耗
                self.state.battery = max(0.0, self.state.battery - 0.0001)

                await asyncio.sleep(dt)
            except Exception as e:
                print(f"[{self.name}] 状态发送异常: {e}")
                import traceback
                traceback.print_exc()
                await asyncio.sleep(0.1)

    def _update_motion(self, dt: float):
        """更新运动状态（简化运动学）"""
        s = self.state

        # 如果有导航目标，向目标移动
        if s.target_x is not None and s.target_y is not None:
            dx = s.target_x - s.x
            dy = s.target_y - s.y
            dist = math.hypot(dx, dy)

            if dist < 0.05:  # 到达
                s.vx = 0.0
                s.vy = 0.0
                s.target_x = None
                s.target_y = None
                print(f"[{self.name}] 到达目标")
            else:
                # 比例控制
                speed = min(0.5, dist * 2.0)  # 最大 0.5 m/s
                s.vx = speed * dx / dist
                s.vy = speed * dy / dist
                # 朝向目标
                s.yaw = math.atan2(dy, dx)
                s.gyro_z = 0.0  # 简化：无旋转

        # 如果有遥控输入，直接控制
        elif abs(s.remote_fwd) > 0.01 or abs(s.remote_turn) > 0.01:
            # 遥控模式：forward 控制速度，turn 控制角速度
            max_speed = 1.0  # joyLinearScale
            max_turn = 1.5   # joyTurnScale

            s.vx = s.remote_fwd * max_speed
            s.gyro_z = s.remote_turn * max_turn
            s.yaw += s.gyro_z * dt
            # 速度方向跟随 yaw
            s.vy = s.vx * math.sin(s.yaw)
            s.vx = s.vx * math.cos(s.yaw)

        else:
            # 无输入，减速停止
            s.vx *= 0.95
            s.vy *= 0.95
            s.gyro_z *= 0.95
            if abs(s.vx) < 0.001:
                s.vx = 0.0
            if abs(s.vy) < 0.001:
                s.vy = 0.0

        # 积分位置
        s.x += s.vx * dt
        s.y += s.vy * dt
        # 站高：移动/站立/平衡站立 0.35m；趴下 0.10m（真狗实测）；其余贴地
        if s.function_mode in (1, 2):
            s.z = 0.35
        elif s.posture == "sit_down":
            s.z = 0.10
        else:
            s.z = 0.0

    # --------------------------------------------------------
    # 主循环
    # --------------------------------------------------------

    async def run(self):
        """启动模拟器"""
        self._running = True

        try:
            # 启动 UDP 监听（接收终端指令）
            loop = asyncio.get_event_loop()
            transport, protocol = await loop.create_datagram_endpoint(
                lambda: DogSimProtocol(self),
                local_addr=(self.ip, self.send_port)
            )

            print(f"[{self.name}] 模拟狗启动")
            print(f"  控制口 (收终端): {self.ip}:{self.send_port}")
            print(f"  状态口 (发状态): {self.ip}:{self.recv_port}")
            print(f"  终端IP: {self.terminal_ip}")
            print(f"  等待终端连接...")

            # 启动状态发送循环
            state_task = asyncio.create_task(self.send_state_loop())
            print(f"[{self.name}] 状态发送任务已创建")

            try:
                # 使用 Event 而不是 Future，更可靠
                stop_event = asyncio.Event()
                await stop_event.wait()  # 永远运行
            except asyncio.CancelledError:
                pass
            finally:
                self._running = False
                state_task.cancel()
                transport.close()
                print(f"[{self.name}] 模拟狗停止")
        except Exception as e:
            print(f"[{self.name}] 启动失败: {e}")
            import traceback
            traceback.print_exc()
            raise


def main():
    parser = argparse.ArgumentParser(description="机器狗 SDK 模拟器")
    parser.add_argument("--name", default="dog_1", help="狗名称（用于日志区分）")
    parser.add_argument("--ip", default="127.0.0.1", help="绑定 IP")
    parser.add_argument("--send-port", type=int, default=8081,
                        help="控制口（终端发到这里）")
    parser.add_argument("--recv-port", type=int, default=8080,
                        help="状态口（终端从这里收）")
    parser.add_argument("--terminal-ip", default="127.0.0.1",
                        help="终端 IP（默认 127.0.0.1，实际部署时为终端微型计算机的固定 IP）")
    args = parser.parse_args()

    sim = DogSim(args.name, args.send_port, args.recv_port, args.ip, args.terminal_ip)

    try:
        asyncio.run(sim.run())
    except KeyboardInterrupt:
        print(f"\n[{args.name}] 用户中断")


if __name__ == "__main__":
    main()
