#include <QCoreApplication>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QMutex>
#include <QMutexLocker>
#include <QTimer>

#include <csignal>
#include <cstdio>

#include "Config/TerminalConfig.h"
#include "MainController.h"

namespace
{

// ============================================================
// 日志：控制台 + 可选落盘（按大小滚动）
//
// 控制台级别与文件级别分开控制（Log/Console / LogLevel）：
// 现场用 SSH 跑时控制台的 1Hz 状态行、每帧差分行会把真正有用的告警刷走，
// 所以默认控制台只留告警以上，全部日志仍按 LogLevel 写进 .log 文件。
//
// 级别：debug < info < warn < error < off
// 注意 QtMsgType 的枚举值顺序不是语义顺序（QtInfoMsg = 4），只能靠 rank 映射。
// ============================================================
int      gMinRank = 0;              // 写文件的最低级别 rank（LogLevel）
int      gConsoleRank = 2;          // 打印到控制台的最低级别 rank（Log/Console，默认只留 warn 以上）
QFile    gLogFile;
QMutex   gLogMutex;
QString  gLogPath;                  // 空 = 不落盘
qint64   gLogMaxBytes = 0;          // 0 = 不滚动
int      gLogKeep = 3;

int levelRank(QtMsgType type)
{
    switch (type)
    {
    case QtDebugMsg:    return 0;
    case QtInfoMsg:     return 1;
    case QtWarningMsg:  return 2;
    case QtCriticalMsg: return 3;
    case QtFatalMsg:    return 4;
    }
    return 0;
}

const char *levelTag(QtMsgType type)
{
    switch (type)
    {
    case QtDebugMsg:    return "D";
    case QtInfoMsg:     return "I";
    case QtWarningMsg:  return "W";
    case QtCriticalMsg: return "C";
    case QtFatalMsg:    return "F";
    }
    return "?";
}

// 调用方必须已持有 gLogMutex
void rotateLogLocked()
{
    if (gLogPath.isEmpty() || gLogMaxBytes <= 0)
        return;

    gLogFile.close();
    // 从大到小滚：先删最老的一份，再把 i-1 挪到 i，最后把当前文件挪成 .1
    for (int i = gLogKeep; i >= 1; --i)
    {
        const QString dst = gLogPath + QLatin1Char('.') + QString::number(i);
        QFile::remove(dst);                       // Windows 上 rename 不覆盖已存在文件
        const QString src = (i == 1)
                                ? gLogPath
                                : gLogPath + QLatin1Char('.') + QString::number(i - 1);
        if (QFile::exists(src))
            QFile::rename(src, dst);
    }
    gLogFile.setFileName(gLogPath);
    gLogFile.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text);
}

// 控制台输出固定走 stderr 并立即刷新：Qt 在检测不到控制台窗口（重定向、SSH、
// 作为服务运行时）会改用 OutputDebugString，日志会直接丢失。
void consoleMessageHandler(QtMsgType type, const QMessageLogContext &, const QString &msg)
{
    if (levelRank(type) < gMinRank)
        return;

    if (levelRank(type) >= gConsoleRank)
    {
        const QByteArray line = msg.toLocal8Bit() + '\n';
        std::fwrite(line.constData(), 1, size_t(line.size()), stderr);
        std::fflush(stderr);
    }

    if (gLogPath.isEmpty())
        return;

    QMutexLocker lock(&gLogMutex);
    if (!gLogFile.isOpen())
        return;

    const QString entry =
        QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd HH:mm:ss.zzz"))
        + QLatin1String(" [") + QLatin1String(levelTag(type)) + QLatin1String("] ")
        + msg + QLatin1Char('\n');
    gLogFile.write(entry.toUtf8());
    gLogFile.flush();

    if (gLogMaxBytes > 0 && gLogFile.size() >= gLogMaxBytes)
        rotateLogLocked();
}

// 启动摘要与 --dump-params 必须始终可见：它们本来就用于核对“配置到底生效了没有”，
// 不能因为现场把 LogLevel 调成 info/warn/error 就一并消失。
void alwaysMessage(const QString &msg)
{
    const int saved = gMinRank;
    const int savedConsole = gConsoleRank;
    gMinRank = 0;
    gConsoleRank = 0;
    qInfo().noquote() << msg;
    gMinRank = saved;
    gConsoleRank = savedConsole;
}

// 控制台级别：debug/info/warn/error/off；无法识别时退回 warn
void applyConsoleLevel(const QString &level)
{
    const QString v = level.trimmed().toLower();
    if (v == QLatin1String("debug"))
        gConsoleRank = 0;
    else if (v == QLatin1String("info"))
        gConsoleRank = 1;
    else if (v == QLatin1String("error"))
        gConsoleRank = 3;
    else if (v == QLatin1String("off"))
        gConsoleRank = 4;      // 只在文件里留痕，控制台彻底静音
    else
        gConsoleRank = 2;      // warn 及未知值 = 只留告警/错误
}

void applyLogLevel(const QString &level)
{
    const QString v = level.trimmed().toLower();
    if (v == QLatin1String("info"))
        gMinRank = 1;
    else if (v == QLatin1String("warn"))
        gMinRank = 2;
    else if (v == QLatin1String("error"))
        gMinRank = 3;
    else if (v == QLatin1String("off"))
        gMinRank = 4;      // off 仍保留 Fatal：致命错误必须留痕，否则现场无据可查
    else
        gMinRank = 0;      // debug 及未知值 = 全输出
}

void openLog(const QString &path, int maxMb, int keep)
{
    gLogPath.clear();
    gLogMaxBytes = 0;
    if (path.trimmed().isEmpty())
        return;            // 空 = 只输出控制台

    QString p = path.trimmed();
    QFileInfo fi(p);
    if (fi.isRelative())
        p = QCoreApplication::applicationDirPath() + QLatin1Char('/') + p;
    QDir().mkpath(QFileInfo(p).absolutePath());

    QMutexLocker lock(&gLogMutex);
    gLogFile.setFileName(p);
    if (!gLogFile.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text))
    {
        std::fprintf(stderr, "日志文件打开失败: %s\n", p.toLocal8Bit().constData());
        std::fflush(stderr);
        return;
    }
    gLogPath = p;
    gLogMaxBytes = (maxMb > 0) ? qint64(maxMb) * 1024 * 1024 : 0;
    gLogKeep = qMax(1, keep);
}

}  // namespace

namespace
{
// ============================================================
// 退出信号：Ctrl-C / kill 时先停狗再退
//
// 之前没有安装 handler，SIGINT 默认终止进程：MQ/UDP 直接断电式退出，
// stop() 从没跑过，狗会继续按最后一条控制流走（现场很危险）。
// handler 只置标志（async-signal-safe），真正的停机放在事件循环里做。
// ============================================================
volatile sig_atomic_t g_quitSignal = 0;

void onQuitSignal(int sig)
{
    g_quitSignal = sig;
}

}  // namespace

// ============================================================
// 机器狗终端程序入口（纯控制台）
// 用法: dogterminal --config <ini路径> [--dump-params]
// 默认配置路径: 应用目录/config/dogterminal.ini
// 部署形态（已确认）: 多终端 = 同一程序 + 每狗一份INI
//   例如: ./dogterminal --config config/dog1.ini
// --dump-params: 打印全部生效参数（含默认值）后退出，用于核对 ini 是否真被读到
// ============================================================
int main(int argc, char *argv[])
{
    qInstallMessageHandler(consoleMessageHandler);

    QCoreApplication app(argc, argv);
    QCoreApplication::setApplicationName(QStringLiteral("DogTerminal"));
    QCoreApplication::setApplicationVersion(QStringLiteral("1.0"));

    QCommandLineParser parser;
    parser.setApplicationDescription(QStringLiteral("机器狗终端：北向RabbitMQ + 南向UDP机器狗"));
    parser.addHelpOption();
    parser.addVersionOption();
    QCommandLineOption cfgOpt(QStringList() << QStringLiteral("c") << QStringLiteral("config"),
                              QStringLiteral("配置文件路径(INI)"),
                              QStringLiteral("config"),
                              QStringLiteral("config/dogterminal.ini"));
    parser.addOption(cfgOpt);
    QCommandLineOption dumpOpt(QStringLiteral("dump-params"),
                               QStringLiteral("打印全部生效参数后退出"));
    parser.addOption(dumpOpt);
    parser.process(app);

    QString iniPath = parser.value(cfgOpt);
    // 相对路径按应用目录解析，避免受工作目录影响
    QFileInfo iniInfo(iniPath);
    if (iniInfo.isRelative())
        iniPath = QCoreApplication::applicationDirPath() + QStringLiteral("/") + iniPath;

    TerminalConfig cfg;
    if (!cfg.load(iniPath))
    {
        qCritical() << "配置文件加载失败:" << iniPath;
        return 1;
    }
    QString errMsg;
    if (!cfg.validate(&errMsg))
    {
        qCritical() << "配置校验失败:" << errMsg;
        return 1;
    }

    // 日志级别与落盘在配置读出来之后才生效；配置加载本身这几行始终可见
    applyLogLevel(cfg.logLevel);
    applyConsoleLevel(cfg.logConsole);

    // 只 dump 参数时不落盘：避免按终端逐个核对参数时创建空日志，
    // 也不与正在运行的实例争同一个日志文件
    const bool dumpOnly = parser.isSet(dumpOpt);
    if (!dumpOnly)
        openLog(cfg.logFile, cfg.logMaxMb, cfg.logKeep);

    // 摘要与 dump 输出始终可见（不受 LogLevel 过滤）：现场就是用它们核对 ini 是否被读到
    alwaysMessage(QStringLiteral("配置加载成功: %1  terminalId=%2  日志级别=%3  控制台=%4  日志文件=%5")
                      .arg(iniPath)
                      .arg(cfg.terminalId)
                      .arg(cfg.logLevel)
                      .arg(cfg.logConsole)
                      .arg(cfg.logFile.isEmpty() ? QStringLiteral("(仅控制台)")
                                                 : cfg.logFile));
    alwaysMessage(QStringLiteral("[Cfg] 生效参数 %1").arg(cfg.summaryText()));

    // 逐项核对 ini 是否被真正读到：把默认值与生效值一起打出来
    if (dumpOnly)
    {
        const QStringList lines = cfg.dumpParams();
        for (int i = 0; i < lines.size(); ++i)
            alwaysMessage(lines.at(i));
        return 0;
    }

    MainController controller;
    if (!controller.init(cfg, &errMsg))
    {
        qCritical() << "初始化失败:" << errMsg;
        return 1;
    }
    controller.start();

    // 退出信号：Ctrl-C / kill 后先走 controller.stop()（发停机、断链路、关日志），
    // 再退出事件循环；否则狗会带着最后的控制流继续跑
    std::signal(SIGINT, onQuitSignal);
#ifdef SIGTERM
    std::signal(SIGTERM, onQuitSignal);
#endif

    QTimer quitTimer;
    quitTimer.setInterval(100);
    QObject::connect(&quitTimer, &QTimer::timeout, [&controller, &quitTimer]() {
        if (!g_quitSignal)
            return;
        quitTimer.stop();
        alwaysMessage(QStringLiteral("[Main] 收到退出信号 %1：先停机再退出").arg(int(g_quitSignal)));
        controller.stop();
        QCoreApplication::quit();
    });
    quitTimer.start();

    const int rc = app.exec();
    alwaysMessage(QStringLiteral("[Main] 已退出（信号=%1）").arg(int(g_quitSignal)));
    return rc;
}
