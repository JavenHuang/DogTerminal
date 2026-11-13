# 6 项功能无实机验证指南

## 环境准备

### 1. 启动 RabbitMQ
```powershell
# 已配置好的 RabbitMQ 服务（管理界面 http://127.0.0.1:15672/）
# 确认交换机 kdiss.dog 已声明（Topic + Durable）
```

### 2. 启动模拟狗
```powershell
cd D:\Work\KDISS\DogTerminal\scripts
python dogsim.py --name dog_1 --send-port 8081 --recv-port 8080

# 多狗验证时，再开一个终端：
python dogsim.py --name dog_2 --send-port 8083 --recv-port 8082
```

### 3. 启动终端
```powershell
cd D:\Work\KDISS\DogTerminal\Bin\release
.\dogterminal.exe --config config\dogterminal.ini
# dogterminal.ini 中：[Terminal] Id=1, [Dog] Ip=127.0.0.1, SendPort=8081, RecvPort=8080
```

### 4. 启动服务器（simulator 或 UIEDC 想定）
```powershell
# 调试用 simulator
cd D:\Work\KDISS\DogTerminal\Bin\release
.\simulator.exe

# 或部署 UIEDC 插件到平台，想定里配置 DogTerminalService
```

---

## 6 项功能验证步骤

### 功能 1：终端连接服务器（RabbitMQ）

**验证点：** simulator 收到 `state.1` 和 `heartbeat.1`

**预期日志：**
```
[Sim] 收到 state.1: terminalId=1 x=0.000 y=0.000 yaw=0.000
[Sim] 收到 heartbeat.1: status=TS_CONNECTED
```

**判定：** 日志持续滚动，无超时告警 → **PASS**

---

### 功能 2：终端连接机器狗（UDP）

**验证点：** dogsim 收到终端心跳和命令

**预期日志：**
```
[dog_1] 终端已连接 (heartbeat)
[dog_1] 收到命令 #1: 0xB3 (SDK_CONTROL_RIGHT) role=sdk
[dog_1]   → SDK 控制权已授予
[dog_1] 收到命令 #2: 0x8A (MOVE_MODE) role=sdk
[dog_1]   → 进入移动模式
```

**判定：** 终端日志显示 `狗已连接`、`控制权已申请` → **PASS**

---

### 功能 3：状态转换→服务器

**验证点：** simulator 显示的坐标与 dogsim 实际坐标一致

**关键检查：** 如果服务器想定里仿真狗初始朝向不朝 x+（originYaw ≠ 0），simulator 显示的世界坐标应该是局部坐标旋转后的值。

**预期日志（simulator）：**
```
[Sim] 收到 state.1: x=0.000 y=0.000 yaw=0.000   # 初始
# ... 狗移动后 ...
[Sim] 收到 state.1: x=1.500 y=2.300 yaw=0.785   # 世界坐标（已旋转）
```

**判定：** 坐标数值匹配，yaw 旋转正确 → **PASS**

---

### 功能 4：姿态控制→狗

**验证点：** simulator 发姿态命令，dogsim 收到对应 CMD

**simulator 输入：**
```
attitude 1 1    # terminalId=1, action=1(AA_STAND_UP)
```

**预期日志：**
```
[Sim] 下发姿态命令 → 终端 1: action=1(AA_STAND_UP)
[dog_1] 收到命令 #3: 0x7A (STAND_UP) role=sdk
[dog_1]   → 站立完成 z=0.35
```

**命令码对照：**
| action | 命令码 | 含义 |
|--------|--------|------|
| 1 | 0x7A | 站立 |
| 2 | 0x6A | 趴下 |
| 3 | 0x9A | 平衡站立 |
| 4 | 0x9B | 锁定 |
| 5 | 0x5A | 急停 |
| 6 | 0x8A | 移动模式 |
| 7 | 0xAE | 慢速 |
| 8 | 0xAF | 正常 |
| 9 | 0xB0 | 快速 |

**判定：** 命令类型匹配 → **PASS**

---

### 功能 5：机动控制（坐标点）导航

**验证点：** simulator 发移动命令，狗实际走到目标

**simulator 输入：**
```
move 1 2.0 0.0 0.5 0.3 30000
# terminalId=1, x=2.0, y=0.0, maxSpeed=0.5, arriveRadius=0.3, timeout=30s
```

**终端日志：**
```
[Main] 导航启动 task 1 目标世界(2.0, 0.0) 局部(2.0, 0.0)
[Navigator] 转向对准: 当前yaw=0.00 目标yaw=0.00 误差=0.00
[Navigator] 直行: 距离=1.50 速度=0.50
...
[Navigator] 任务 1 结束 result=3(MR_ARRIVED)
[Main] 任务 1 反馈 result: 3
```

**dogsim 日志：**
```
[dog_1] 收到 remote: fwd=0.50 turn=0.00
[dog_1] 收到 remote: fwd=0.50 turn=0.00
...
[dog_1] 到达目标
```

**关键检查（坐标转换）：**
- 如果 originYaw = π/2（狗初始朝 y+），目标世界 (2,0) 应转换为局部 (0, -2)
- 狗应向局部 y- 方向移动，simulator 显示的世界坐标从 (0,0) 变到 (2,0)

**判定：** 导航完成，反馈 MR_ARRIVED，世界坐标正确 → **PASS**

---

### 功能 6：多终端多狗

**配置：**

| 终端 | terminalId | 狗 IP | 控制口 | 状态口 |
|------|-----------|-------|--------|--------|
| A | 1 | 127.0.0.1 | 8081 | 8080 |
| B | 2 | 127.0.0.1 | 8083 | 8082 |

**启动：**
```powershell
# 终端 A
.\dogterminal.exe --config config\dogterminal.ini

# 终端 B（新窗口）
.\dogterminal.exe --config config\dogterminal_b.ini

# dogsim A
python dogsim.py --name dog_1 --send-port 8081 --recv-port 8080

# dogsim B（新窗口）
python dogsim.py --name dog_2 --send-port 8083 --recv-port 8082
```

**simulator 输入：**
```
move 1 1.0 0.0 0.5 0.3 30000   # 只控制狗1
move 2 0.0 1.0 0.5 0.3 30000   # 只控制狗2
```

**判定：**
- 狗 A 的 dogsim 显示移动，狗 B 不动
- simulator 收到 `state.1` 和 `state.2` 分别更新
- 服务器想定里 `dog_1` 和 `dog_2` 实体各自移动 → **PASS**

---

## 快速验证脚本

```powershell
# verify_all.ps1 — 一键启动所有组件（调试用）
# 注意：需要 4 个终端窗口

# 窗口 1：dogsim
python D:\Work\KDISS\DogTerminal\scripts\dogsim.py --name dog_1

# 窗口 2：终端
cd D:\Work\KDISS\DogTerminal\Bin\release
.\dogterminal.exe --config config\dogterminal.ini

# 窗口 3：simulator
.\simulator.exe

# 窗口 4：观察日志
# 在 simulator 中输入测试命令
```

---

## 常见问题排查

| 现象 | 可能原因 | 解决 |
|------|---------|------|
| simulator 收不到 state.1 | 终端未连接 MQ | 检查终端日志 `MQ 链路已就绪` |
| dogsim 收不到命令 | 端口不匹配 | 检查 ini 中 SendPort/RecvPort 与 dogsim 参数一致 |
| 导航目标错位 | originYaw 未配置 | 检查想定里实体 visualDirection |
| 速度方向错误 | vWorld 未旋转 | 确认终端已编译最新版本（含 localVecToWorld） |
| 看门狗不触发 | 服务器未发心跳 | 确认 UIEDC 插件已加载且 serverHeartbeatMs > 0 |
