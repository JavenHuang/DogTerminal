# ============================================================
# 机器狗终端程序（纯控制台）
# 目标环境: ARM64 + Ubuntu 22.04 + Qt 5.15.2 (qmake)
# 验证环境: Windows + Qt 5.12.0 (mingw73_64)
# 依赖: Qt Network（内含移植版 QAMQP 第三方源码，无需外部库）
# 构建: qmake && make -j4
# 产物: Bin/dogterminal（config 随构建自动复制到 Bin/config）
# 运行: cd Bin && ./dogterminal --config config/dogterminal.ini
# ============================================================
QT -= gui
QT += network

TEMPLATE = app
CONFIG += console c++14
CONFIG -= app_bundle

TARGET = dogterminal

BIN_DIR = $$PWD/Bin
CONFIG(debug, debug|release) {
    OUT_DIR = $$BIN_DIR/debug
} else {
    OUT_DIR = $$BIN_DIR/release
}
DESTDIR = $$OUT_DIR

# ---------- 构建时创建输出目录并同步 config ----------
# qmake 不会为 DESTDIR 自动建目录，需在链接前创建
win32 {
    QMAKE_PRE_LINK += if not exist $$shell_quote($$shell_path($$OUT_DIR)) $$QMAKE_MKDIR $$shell_quote($$shell_path($$OUT_DIR))
    # xcopy /i 自动创建 config 目录并覆盖同名文件
    QMAKE_POST_LINK += $$QMAKE_COPY_DIR $$shell_quote($$shell_path($$PWD/config)) $$shell_quote($$shell_path($$OUT_DIR/config))
} else {
    QMAKE_PRE_LINK += $$QMAKE_MKDIR $$shell_quote($$shell_path($$OUT_DIR))
    QMAKE_POST_LINK += $$QMAKE_MKDIR $$shell_quote($$shell_path($$OUT_DIR/config)) $$escape_expand(\\n\\t)
    # 复制目录内容（config/. 避免生成 config/config 嵌套）
    QMAKE_POST_LINK += $$QMAKE_COPY_DIR $$shell_quote($$shell_path($$PWD/config/.)) $$shell_quote($$shell_path($$OUT_DIR/config))
}

INCLUDEPATH += $$PWD $$PWD/src
DEPENDPATH += $$PWD $$PWD/src

DEFINES += QT_DEPRECATED_WARNINGS

# ============================================================
# 南向驱动：udp=自实现 UDP+JSON（跨平台）；sdk=官方 libzsibot（仅 Linux）
# 选择由运行期 INI 的 [Dog] Driver 决定，编译期两者都编进 Linux 产物。
# SDK 头文件/静态库默认取同级目录 ../genisom_L1_sdk-main，
# 可用环境变量 ZSIBOT_SDK_ROOT 覆盖。
# ============================================================
win32 {
    message("南向驱动: 仅 UDP+JSON（官方 SDK 无 Windows 库）")
    CONFIG += c++14
}

unix {
    # SDK 头文件用 std::optional（C++17）；Qt 5.15 的 qmake 认 c++1z/c++2a，不认 "c++20"
    CONFIG += c++17
    CONFIG -= c++14

    ZSIBOT_SDK_ROOT = $$(ZSIBOT_SDK_ROOT)
    isEmpty(ZSIBOT_SDK_ROOT) {
        ZSIBOT_SDK_ROOT = $$clean_path($$PWD/../genisom_L1_sdk-main)
    }

    # 架构判定用 uname -m（比 QT_ARCH 更可靠）
    # 仅 x86_64 / aarch64 有对应预编译库；其它架构（armv7l、i686 等）自动只编 UDP
    ZSIBOT_MACHINE = $$system(uname -m)
    ZSIBOT_ARCH =
    contains(ZSIBOT_MACHINE, aarch64): ZSIBOT_ARCH = aarch64
    contains(ZSIBOT_MACHINE, arm64):   ZSIBOT_ARCH = aarch64
    contains(ZSIBOT_MACHINE, x86_64):  ZSIBOT_ARCH = x86_64

    # 关闭 SDK 的开关：qmake CONFIG+=no_zsibot_sdk（例如 ARM 上库版本不匹配时）
    no_zsibot_sdk {
        message("已按 CONFIG+=no_zsibot_sdk 关闭 SDK，仅编译 UDP 驱动")
    } else: isEmpty(ZSIBOT_ARCH) {
        message("当前架构 $$ZSIBOT_MACHINE 无匹配 SDK 库，仅编译 UDP 驱动（[Dog] Driver=sdk 将运行期回退为 udp）")
    } else: !exists($$ZSIBOT_SDK_ROOT/lib/$$ZSIBOT_ARCH/libzsibot.a) {
        message("未找到 SDK 静态库 $$ZSIBOT_SDK_ROOT/lib/$$ZSIBOT_ARCH/libzsibot.a，仅编译 UDP 驱动")
    } else {
        INCLUDEPATH += $$ZSIBOT_SDK_ROOT/include
        LIBS += -L$$ZSIBOT_SDK_ROOT/lib/$$ZSIBOT_ARCH -lzsibot -lpthread
        DEFINES += DOGTERMINAL_HAS_SDK
        SOURCES += src/DogLink/SdkDogClient.cpp
        HEADERS += src/DogLink/SdkDogClient.h
        message("SDK 静态库: $$ZSIBOT_SDK_ROOT/lib/$$ZSIBOT_ARCH/libzsibot.a")
    }
}

# ============================================================
# MATRiX 仿真驱动：matrix=Zenoh+UDP 组合（仅 Linux 构建）
# 需要 zenoh-cxx 库：sudo apt install libzenoh-cxx-dev
# 或从 https://github.com/eclipse-zenoh/zenoh-cxx 编译
# 关闭开关：qmake CONFIG+=no_zenoh_matrix
# ============================================================
unix {
    no_zenoh_matrix {
        message("已按 CONFIG+=no_zenoh_matrix 关闭 MATRiX 驱动")
    } else {
        # zenoh-cxx 安装路径：优先系统默认 /usr，其次可通过 ZENOH_CXX_ROOT 环境变量覆盖
        ZENOH_CXX_ROOT = $$(ZENOH_CXX_ROOT)
        isEmpty(ZENOH_CXX_ROOT) {
            ZENOH_CXX_ROOT = /usr
        }
        # 检查头文件是否存在
        ZENOH_HEADER = $$ZENOH_CXX_ROOT/include/zenoh.hxx
        exists($$ZENOH_HEADER) {
            INCLUDEPATH += $$ZENOH_CXX_ROOT/include
            LIBS += -L$$ZENOH_CXX_ROOT/lib -lzenohc
            DEFINES += DOGTERMINAL_HAS_MATRIX
            DEFINES += ZENOHCXX_ZENOHC
            SOURCES += src/DogLink/MatrixDogClient.cpp
            HEADERS += src/DogLink/MatrixDogClient.h src/DogLink/MatrixDefs.h
            message("MATRiX 驱动已启用 (zenoh-cxx @ $$ZENOH_CXX_ROOT)")
        } else {
            message("未找到 zenoh-cxx 头文件 $$ZENOH_HEADER，MATRiX 驱动未启用（[Dog] Driver=matrix 将运行期回退为 udp）")
        }
    }
}

SOURCES += \
    src/main.cpp \
    src/MainController.cpp \
    src/MainControllerSetup.cpp \
    src/Command/CommandRouter.cpp \
    src/Convert/RtkPoseTracker.cpp \
    src/Convert/RtkFrameReader.cpp \
    src/Control/RtkJumpGate.cpp \
    src/Control/YawStabilizer.cpp \
    src/Control/NavBridge.cpp \
    src/Report/ReportService.cpp \
    src/Report/StatePrinter.cpp \
    src/MainControllerPose.cpp \
    src/MainControllerNav.cpp \
    src/Config/TerminalConfig.cpp \
    src/Protocol/DogProtocol.cpp \
    src/Convert/MapTransform.cpp \
    src/Convert/RtkFusion.cpp \
    src/ServerLink/MqLink.cpp \
    src/ServerLink/DetectionLink.cpp \
    src/ServerLink/CalibPanel.cpp \
    src/DogLink/UdpDogClient.cpp \
    src/Control/Navigator.cpp \
    src/Control/AttitudeExecutor.cpp \
    thirdparty/RabbitMq/qamqpauthenticator.cpp \
    thirdparty/RabbitMq/qamqpchannel.cpp \
    thirdparty/RabbitMq/qamqpchannelhash.cpp \
    thirdparty/RabbitMq/qamqpclient.cpp \
    thirdparty/RabbitMq/qamqpexchange.cpp \
    thirdparty/RabbitMq/qamqpframe.cpp \
    thirdparty/RabbitMq/qamqpmessage.cpp \
    thirdparty/RabbitMq/qamqpqueue.cpp \
    thirdparty/RabbitMq/qamqptable.cpp

HEADERS += \
    src/MainController.h \
    src/Config/TerminalConfig.h \
    src/Protocol/DogProtocol.h \
    src/Common/DogText.h \
    src/Convert/MapTransform.h \
    src/Convert/RtkFusion.h \
    src/Convert/RtkPoseTracker.h \
    src/Convert/RtkFrameReader.h \
    src/Control/RtkJumpGate.h \
    src/Control/MotionState.h \
    src/Control/YawStabilizer.h \
    src/Control/NavBridge.h \
    src/Report/ReportService.h \
    src/Report/StatePrinter.h \
    src/Command/CommandHost.h \
    src/Command/CommandRouter.h \
    src/ServerLink/MqLink.h \
    src/ServerLink/DetectionLink.h \
    src/ServerLink/CalibPanel.h \
    src/DogLink/DogTypes.h \
    src/DogLink/DogClientBase.h \
    src/DogLink/UdpDogClient.h \
    src/DogLink/ZsibotDefs.h \
    src/Control/Navigator.h \
    src/Control/AttitudeExecutor.h \
    thirdparty/RabbitMq/qamqpauthenticator.h \
    thirdparty/RabbitMq/qamqpchannel.h \
    thirdparty/RabbitMq/qamqpchannel_p.h \
    thirdparty/RabbitMq/qamqpchannelhash_p.h \
    thirdparty/RabbitMq/qamqpclient.h \
    thirdparty/RabbitMq/qamqpclient_p.h \
    thirdparty/RabbitMq/qamqpexchange.h \
    thirdparty/RabbitMq/qamqpexchange_p.h \
    thirdparty/RabbitMq/qamqpframe_p.h \
    thirdparty/RabbitMq/qamqpglobal.h \
    thirdparty/RabbitMq/qamqpmessage.h \
    thirdparty/RabbitMq/qamqpmessage_p.h \
    thirdparty/RabbitMq/qamqpqueue.h \
    thirdparty/RabbitMq/qamqpqueue_p.h \
    thirdparty/RabbitMq/qamqptable.h
