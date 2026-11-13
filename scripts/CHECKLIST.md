# 机器狗终端 实机联调 Runbook

> 先运行 `scripts/preflight_check.sh` 通过全部硬性检查，再按本清单逐步联调。
> 目标环境：ARM64 + Ubuntu 22.04 + Qt 5.15.2

## 0. 前置准备

- [ ] 机器狗已开机，WiFi 路由器已连接狗与终端
- [ ] 服务器（或 simulator）已启动 RabbitMQ
- [ ] 终端二进制已构建：`qmake DogTerminal.pro && make -j4`（产物在 `Bin/dogterminal`）
- [ ] 配置文件已随构建复制到 `Bin/config/`，按本狗参数修改 `Bin/config/dogterminal.ini`（IP、terminalId）

## 1. 运行预检脚本

```bash
bash scripts/preflight_check.sh config/dogterminal.ini
```

**通过标准**：`FAIL=0`。WARN 项可酌情忽略（如开发机非 ARM64）。

## 2. 启动终端并观察南向链路

```bash
cd Bin && mkdir -p logs
QT_FORCE_STDERR_LOGGING=1 ./dogterminal --config config/dogterminal.ini 2>&1 | tee logs/dog_$(date +%Y%m%d_%H%M%S).log
```

**观察点**：
- [ ] `[DogLink] 启动心跳` — UDP 客户端已启动
- [ ] 狗连接状态变化：`[Main] 机器狗连接状态: 已连接`
- [ ] 控制权申请：`[Main] 开始申请SDK控制权...` → `[Main] 已取得SDK控制权，进入移动模式`
- [ ] 终端状态由 `1(TS_CONNECTING_DOG)` → `3(TS_DOG_CONNECTED)` → `4(TS_READY)`

**若控制权申请失败**：检查是否有其他 App/终端占用狗的 SDK 控制权，先断开其他控制端。

## 3. 验证北向链路（RabbitMQ）

- [ ] `[MqLink] 已连接` → `[MqLink] 链路就绪，绑定路由键: cmd.<Id>`
- [ ] 服务器（或 simulator）能收到 `state.<Id>` 消息
- [ ] 服务器下发 `MSG_CALIBRATION` 后终端打印 `[Main] 收到坐标标定`

**无标定不得下发机动指令**——终端会返回 `MR_REJECTED "尚未收到坐标标定"`。

## 4. 姿态控制联调

服务器下发 `MSG_ATTITUDE_CMD`，观测终端日志与狗动作：

| action | 含义 | 狗动作 |
|--------|------|--------|
| 1 (STAND_UP) | 站立 | 四肢站立 |
| 2 (SIT_DOWN) | 趴下 | 趴下 |
| 5 (EMERGENCY_STOP) | 急停 | 趴下（软急停）|
| 6 (MOVE_MODE) | 移动模式 | 进入移动 |
| 10 (REQUEST_CONTROL) | 申请控制权 | 重新申请 |

## 5. 机动控制联调（航点导航）

1. 服务器下发 `MSG_MOVE_CMD`（世界坐标目标点）
2. 终端返回 `MSG_TASK_FEEDBACK` `result=0(ACCEPTED)`，打印 `目标局部坐标(x,y)`
3. 狗开始行走，终端持续上报状态（位置/航向变化）
4. 到达目标后返回 `result=3(ARRIVED)`

**观察点**：
- [ ] 狗朝向先对准目标再前进
- [ ] 到达半径内停车
- [ ] 侧后方目标若 `AllowBackward=true` 则倒退接近

## 6. 速度直控联调

服务器周期下发 `MSG_VELOCITY_CMD`（机体系 vx/vy/yawRate），狗应连续运动。
停止发送后 **500ms（VelocityTimeoutMs）内** 狗自动零速停车（死手保护）。

## 7. 失联保护验证

| 场景 | 预期行为 |
|------|----------|
| 断开终端与 RabbitMQ 网络 | 终端取消当前任务并零速停车，日志 `服务器链路断开` |
| 执行任务时服务器停止发心跳 >5s | 取消任务并零速停车，日志 `服务器消息超时` |
| 断开终端与狗的 WiFi | 终端取消任务，日志 `机器狗断连` |
| 速度直控时停止发指令 | 500ms 后自动零速 |

## 8. 常见问题排查

| 现象 | 原因 | 处理 |
|------|------|------|
| `[MqLink] Socket错误` 持续 | RabbitMQ 地址/端口错误或未启动 | 检查 `[RabbitMq] Host/Port` 与服务器 |
| 狗状态一直 `TS_DOG_CONNECTED` 不变 `TS_READY` | 未取得 SDK 控制权 | 释放其他控制端，或检查狗是否在 `sdk` 角色 |
| `MR_REJECTED 尚未收到坐标标定` | 服务器未下发标定 | 确认服务器发送 `MSG_CALIBRATION` |
| `MR_REJECTED 任务执行中` | 已有任务在跑 | 先发 `MSG_STOP_CMD` 取消，或等任务结束 |
| 状态上报位置始终为 0 | 未收到标定，局部坐标直接当世界坐标上报 | 服务器下发标定后位置正常 |
