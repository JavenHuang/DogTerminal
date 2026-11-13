#include <QCoreApplication>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QDebug>
#include <QDir>
#include <QFileInfo>

#include <cstdio>

#include "Config/TerminalConfig.h"
#include "MainController.h"

namespace
{

// 真狗联调时日志需要可靠可见：Qt 在检测不到控制台窗口（如被重定向、SSH 下运行）
// 时会改用 OutputDebugString 导致输出丢失，这里固定写 stderr 并立即刷新。
void consoleMessageHandler(QtMsgType type, const QMessageLogContext &, const QString &msg)
{
    Q_UNUSED(type);
    const QByteArray line = msg.toLocal8Bit() + '\n';
    std::fwrite(line.constData(), 1, size_t(line.size()), stderr);
    std::fflush(stderr);
}

}  // namespace

// ============================================================
// 机器狗终端程序入口（纯控制台）
// 用法: dogterminal --config <ini路径>
// 默认配置路径: 应用目录/config/dogterminal.ini
// 部署形态（已确认）: 多终端 = 同一程序 + 每狗一份INI
//   例如: ./dogterminal --config config/dog1.ini
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
    qInfo() << "配置加载成功:" << iniPath << "terminalId =" << cfg.terminalId;

    MainController controller;
    if (!controller.init(cfg, &errMsg))
    {
        qCritical() << "初始化失败:" << errMsg;
        return 1;
    }
    controller.start();

    return app.exec();
}
