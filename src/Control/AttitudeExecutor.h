#ifndef ATTITUDE_EXECUTOR_H
#define ATTITUDE_EXECUTOR_H

#include <QObject>
#include "Protocol/DogProtocol.h"

// ============================================================
// 姿态执行器：将服务器下发的离散姿态控制命令
// 映射为机器狗 SDK 的 CmdCode（已确认的姿态控制语义）
// ============================================================
class AttitudeExecutor : public QObject
{
    Q_OBJECT
public:
    explicit AttitudeExecutor(QObject *parent = nullptr);

    // 执行姿态命令，输出对应的SDK命令码
    void execute(DogProtocol::AttitudeAction action);

signals:
    void cmdCode(quint32 code);
    void unknownAction(DogProtocol::AttitudeAction action);
};

#endif  // ATTITUDE_EXECUTOR_H
