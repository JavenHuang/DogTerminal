#!/usr/bin/env bash
# ============================================================
# 机器狗终端 - 实机联调前验证脚本（Preflight Check）
# 适用环境: ARM64 + Ubuntu 22.04 + Qt 5.15.2
# 用法:
#   bash scripts/preflight_check.sh [配置文件路径]
#   默认配置: config/dogterminal.ini
#
# 验证项:
#   1. 运行环境（OS/架构/Qt 版本）
#   2. 配置文件字段完整性与合法性
#   3. 网络连通性（RabbitMQ TCP / 机器狗 IP + 端口）
#   4. 终端二进制可执行性与启动日志
#   5. 联调模拟器二进制
#   6. 日志目录就绪
#   7. SDK 控制权申请前置条件
# ============================================================
set -u

# ---------- 颜色与工具函数 ----------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

PASS=0
FAIL=0
WARN=0

pass() { echo -e "${GREEN}[PASS]${NC} $1"; PASS=$((PASS+1)); }
fail() { echo -e "${RED}[FAIL]${NC} $1"; FAIL=$((FAIL+1)); }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; WARN=$((WARN+1)); }
info() { echo -e "[INFO] $1"; }

# 定位脚本与工程根目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
CONFIG_FILE="${1:-$PROJECT_ROOT/config/dogterminal.ini}"

echo "============================================================"
echo " 机器狗终端 实机联调前验证"
echo " 工程根: $PROJECT_ROOT"
echo " 配置文件: $CONFIG_FILE"
echo " 时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo "============================================================"

# ---------- Step 0: 运行环境 ----------
echo ""
echo "--- Step 0: 运行环境 ---"

OS_DISTRO=""
if [ -f /etc/os-release ]; then
    . /etc/os-release
    OS_DISTRO="$NAME $VERSION_ID"
fi
echo "$OS_DISTRO" | grep -qiE "ubuntu.?22\.04" \
    && pass "操作系统: $OS_DISTRO" \
    || { warn "操作系统非 Ubuntu 22.04: $OS_DISTRO（可能影响 Qt 二进制兼容性）"; }

ARCH="$(uname -m)"
if [ "$ARCH" = "aarch64" ]; then
    pass "CPU 架构: $ARCH (ARM64)"
else
    warn "CPU 架构: $ARCH（目标应为 aarch64/ARM64，当前可能是开发机交叉环境）"
fi

# Qt 版本：优先 qmake，其次 qmake6
QMAKE_BIN=""
for q in qmake qmake6; do
    if command -v "$q" >/dev/null 2>&1; then
        QMAKE_BIN="$q"; break
    fi
done
if [ -n "$QMAKE_BIN" ]; then
    QT_VER="$($QMAKE_BIN -v 2>/dev/null | grep -oE 'Qt version [0-9.]+' | head -n1)"
    QT_VER_NUM="$($QMAKE_BIN -query QT_VERSION 2>/dev/null)"
    pass "Qt: $QT_VER ($QMAKE_BIN -query QT_VERSION=$QT_VER_NUM)"
    if [ -z "$QT_VER_NUM" ]; then
        warn "无法解析 Qt 版本号，请确认 qmake 指向 Qt 5.15.2"
    elif ! echo "$QT_VER_NUM" | grep -qE "^5\.(1[2-9]|2[0-9])"; then
        warn "Qt 版本 $QT_VER_NUM 不在 5.12~5.15 兼容范围内"
    fi
else
    fail "未找到 qmake，请安装 Qt 5.15.2 并将 qmake 加入 PATH"
fi

# 必需工具
for tool in nc ping awk grep; do
    command -v "$tool" >/dev/null 2>&1 \
        && pass "工具可用: $tool" \
        || fail "工具缺失: $tool（sudo apt install $tool）"
done

# ---------- Step 1: 配置文件字段校验 ----------
echo ""
echo "--- Step 1: 配置文件校验 ---"

if [ ! -f "$CONFIG_FILE" ]; then
    fail "配置文件不存在: $CONFIG_FILE"
    echo ""
    echo "验证中止（配置文件缺失）"
    exit 1
fi
pass "配置文件存在: $CONFIG_FILE"

# 简易 INI 解析: 读取 [section] 下 key=value
# 用法: ini_get <section> <key>
ini_get() {
    local section="$1" key="$2" val
    val="$(awk -v sec="$section" -v k="$key" '
        BEGIN{insec=0}
        /^\[/{gsub(/[\[\]]/,"",$0); insec=($0==sec); next}
        insec && /^[[:space:]]*[^;#[:space:]]/{
            idx=index($0,"="); if(idx>0){
                key=substr($0,1,idx-1); val=substr($0,idx+1);
                gsub(/^[[:space:]]+|[[:space:]]+$/,"",key);
                gsub(/^[[:space:]]+|[[:space:]]+$/,"",val);
                if(key==k) print val;
            }
        }' "$CONFIG_FILE")"
    echo "$val"
}

# 必填字段检查
check_field() {
    local section="$1" key="$2" desc="$3" must_positive="$4"
    local val="$(ini_get "$section" "$key")"
    if [ -z "$val" ]; then
        fail "缺失字段: [$section] $key ($desc)"
        return 1
    fi
    if [ "$must_positive" = "yes" ]; then
        if echo "$val" | grep -qE '^[0-9]+(\.[0-9]+)?$' && awk -v v="$val" 'BEGIN{exit !(v>0)}'; then
            pass "[$section] $key = $val ($desc)"
        else
            fail "[$section] $key = $val 应大于0 ($desc)"
            return 1
        fi
    else
        pass "[$section] $key = $val ($desc)"
    fi
    return 0
}

check_field "Terminal" "Id" "终端唯一ID" "yes"
TID="$(ini_get Terminal Id)"

check_field "RabbitMq" "Host" "RabbitMQ主机" "no"
check_field "RabbitMq" "Port" "RabbitMQ端口" "yes"
check_field "RabbitMq" "Vhost" "虚拟主机" "no"
check_field "RabbitMq" "User" "用户名" "no"
check_field "RabbitMq" "Password" "密码" "no"
check_field "RabbitMq" "Exchange" "Topic交换机名" "no"

check_field "Dog" "Ip" "机器狗IP" "no"
check_field "Dog" "SendPort" "SDK控制口" "yes"
check_field "Dog" "RecvPort" "SDK状态口" "yes"
check_field "Dog" "Role" "SDK角色(sdk/remote)" "no"
check_field "Dog" "HeartbeatMs" "心跳周期ms" "yes"
check_field "Dog" "TimeoutMs" "断连判定ms" "yes"

# 心跳 < 超时 一致性
HB="$(ini_get Dog HeartbeatMs)"
TO="$(ini_get Dog TimeoutMs)"
if [ -n "$HB" ] && [ -n "$TO" ] && awk -v h="$HB" -v t="$TO" 'BEGIN{exit !(h<t)}'; then
    pass "心跳周期($HB ms) < 断连超时($TO ms)"
else
    fail "心跳周期($HB ms) 必须 < 断连超时($TO ms)"
fi

# StateRateHz 范围
SRH="$(ini_get Report StateRateHz)"
if [ -n "$SRH" ] && awk -v v="$SRH" 'BEGIN{exit !(v>=1 && v<=100)}'; then
    pass "状态上报频率 StateRateHz=$SRH Hz (1~100)"
else
    fail "StateRateHz=$SRH 超出 1~100 范围"
fi

# ---------- Step 2: 网络连通性 ----------
echo ""
echo "--- Step 2: 网络连通性 ---"

MQ_HOST="$(ini_get RabbitMq Host)"
MQ_PORT="$(ini_get RabbitMq Port)"
DOG_IP="$(ini_get Dog Ip)"
DOG_SEND="$(ini_get Dog SendPort)"
DOG_RECV="$(ini_get Dog RecvPort)"

# RabbitMQ TCP
if command -v nc >/dev/null 2>&1; then
    if nc -z -w 3 "$MQ_HOST" "$MQ_PORT" 2>/dev/null; then
        pass "RabbitMQ 可达: $MQ_HOST:$MQ_PORT"
    else
        fail "RabbitMQ 不可达: $MQ_HOST:$MQ_PORT（检查网络/防火墙/RabbitMQ是否启动）"
    fi
else
    warn "nc 不可用，跳过 RabbitMQ TCP 检测"
fi

# 机器狗 ICMP
if ping -c 2 -W 2 "$DOG_IP" >/dev/null 2>&1; then
    pass "机器狗 ICMP 可达: $DOG_IP"
else
    warn "机器狗 ICMP 不可达: $DOG_IP（狗可能禁 ping，转测端口）"
fi

# 机器狗状态口（UDP 端口无法 nc 直连，仅做路由可达提示）
info "机器狗 UDP 状态口 $DOG_IP:$DOG_RECV 需终端运行后通过回包确认（启动终端观察 [DogLink] 日志）"
info "机器狗 UDP 控制口 $DOG_IP:$DOG_SEND 同上"

# ---------- Step 3: 终端二进制 ----------
echo ""
echo "--- Step 3: 终端二进制 ---"

BIN_DIR="$PROJECT_ROOT"
# 查找可执行文件: Bin/（qmake DESTDIR 约定）或历史构建目录
DOG_BIN=""
for cand in "$PROJECT_ROOT/Bin/dogterminal" "$PROJECT_ROOT/dogterminal" "$PROJECT_ROOT/build/release/dogterminal" "$PROJECT_ROOT/build/dogterminal"; do
    if [ -x "$cand" ]; then DOG_BIN="$cand"; break; fi
done
if [ -z "$DOG_BIN" ]; then
    fail "未找到可执行的 dogterminal（请先 qmake && make 构建）"
else
    pass "终端二进制: $DOG_BIN"
    # --help 不应崩溃
    HELP_OUT="$($DOG_BIN --help 2>&1 || true)"
    if echo "$HELP_OUT" | grep -qi "Usage\|用法\|--config"; then
        pass "dogterminal --help 正常输出"
    else
        warn "dogterminal --help 输出未匹配预期（可能命令行解析器未显示用法）"
    fi

    # 启动 5 秒，抓取日志确认配置加载与启动流程
    LOG_FILE="/tmp/dogterminal_preflight_$$.log"
    QT_FORCE_STDERR_LOGGING=1 timeout 5 "$DOG_BIN" --config "$CONFIG_FILE" >"$LOG_FILE" 2>&1 || true
    if grep -q "终端启动" "$LOG_FILE"; then
        pass "终端启动日志包含 '终端启动 terminalId=$TID'"
    else
        fail "终端启动 5 秒内未出现 '终端启动' 日志"
        info "--- 启动日志 ---"; cat "$LOG_FILE"
    fi
    # 配置加载成功
    if grep -q "配置加载成功" "$LOG_FILE"; then
        pass "配置文件被成功加载"
    else
        fail "配置文件未成功加载"
    fi
    rm -f "$LOG_FILE"
fi

# ---------- Step 4: 模拟器二进制 ----------
echo ""
echo "--- Step 4: 联调模拟器二进制 ---"

SIM_BIN=""
for cand in "$PROJECT_ROOT/Bin/simulator" "$PROJECT_ROOT/simulator" "$PROJECT_ROOT/build-sim/release/simulator" "$PROJECT_ROOT/build-sim/simulator"; do
    if [ -x "$cand" ]; then SIM_BIN="$cand"; break; fi
done
if [ -z "$SIM_BIN" ]; then
    warn "未找到 simulator 二进制（联调模拟器非必需，可用真实服务器替代）"
else
    pass "模拟器二进制: $SIM_BIN"
fi

# ---------- Step 5: 日志目录 ----------
echo ""
echo "--- Step 5: 日志目录 ---"

LOG_DIR="$PROJECT_ROOT/logs"
if mkdir -p "$LOG_DIR" 2>/dev/null; then
    pass "日志目录可用: $LOG_DIR"
else
    warn "无法创建日志目录: $LOG_DIR"
fi

# ---------- Step 6: SDK 控制权前置检查 ----------
echo ""
echo "--- Step 6: SDK 控制权前置条件 ---"

ROLE="$(ini_get Dog Role)"
AUTO="$(ini_get Dog AutoAcquireControl)"
if [ "$ROLE" = "sdk" ]; then
    pass "SDK 角色为 sdk，连接后将申请 SDK 控制权"
else
    warn "SDK 角色为 '$ROLE'（非 sdk），将无法申请 SDK 控制权下发运动指令"
fi
if [ "$AUTO" = "true" ]; then
    pass "AutoAcquireControl=true，狗连接后自动申请控制权并进入移动模式"
else
    warn "AutoAcquireControl=false，需服务器下发 MSG_ATTITUDE_CMD(AA_REQUEST_CONTROL=10) 主动申请"
fi
info "实机提示: 若其他终端/App 已占用控制权，需先释放；否则 SDK 控制权申请将持续重试直至成功"

# ---------- Step 7: 服务器侧约定确认 ----------
echo ""
echo "--- Step 7: 服务器侧协议约定（人工确认） ---"
EXCHANGE="$(ini_get RabbitMq Exchange)"
info "Topic 交换机名: $EXCHANGE（必须与服务器一致）"
info "上行路由键: state.$TID / heartbeat.$TID / feedback.$TID"
info "下行路由键: cmd.$TID（终端绑定此键接收指令）"
info "序列化版本: QDataStream::Qt_5_12（服务器必须使用相同版本解包）"
warn "请人工确认服务器已就绪并使用上述交换机/路由键/序列化版本，否则终端收不到指令"

# ---------- 汇总 ----------
echo ""
echo "============================================================"
echo " 验证汇总: PASS=$PASS  FAIL=$FAIL  WARN=$WARN"
echo "============================================================"
if [ $FAIL -eq 0 ]; then
    echo -e "${GREEN}全部硬性检查通过${NC}，可进入实机联调（参考 CHECKLIST.md）"
    exit 0
else
    echo -e "${RED}存在 $FAIL 项硬性失败${NC}，请修复后重新运行本脚本"
    exit 1
fi
