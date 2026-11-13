# DogTerminal

机器狗控制终端。Qt 5.12 / C++14 工程，目标运行环境为 ARM64 + Ubuntu（qmake 构建），Windows 下可编译验证。

## 组成

- `src/`：终端源码。南向驱动支持 `udp` / `sdk` / `matrix`（`[Dog] Driver` 选择），北向通过 RabbitMQ 与任务服务器对接。
- `thirdparty/RabbitMq/`：内置的 QAMQP 移植版，无需外部 MQ 库。
- `config/dogterminal.ini`：运行配置。段为 `Terminal` / `RabbitMq` / `Report` / `Dog` / `Map` / `Pose` / `Matrix` / `Nav` / `Rtk` / `Detection`。
- `scripts/`：现场运维脚本。

## 构建与运行

```bash
qmake DogTerminal.pro
make -j4
cd Bin/release && ./dogterminal --config config/dogterminal.ini
```

构建产物输出到 `Bin/release`（Debug 为 `Bin/debug`），`config/` 会自动复制到产物目录同级。

## 注意

`config/dogterminal.ini` 内含现场设备地址与交换机账号，公开仓库前请自行脱敏。

模拟器工程 [simulator](../simulator) 通过 `TERM_ROOT=$$PWD/../DogTerminal` 只读引用本仓库的 `src/Protocol`、`src/ServerLink/MqLink` 与 `thirdparty/RabbitMq`，两个仓库需要克隆到同一父目录、且目录名保持 `DogTerminal` 与 `simulator`。
