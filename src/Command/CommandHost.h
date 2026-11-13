#ifndef COMMAND_HOST_H
#define COMMAND_HOST_H

#include <QString>

#include "Protocol/DogProtocol.h"

// ============================================================
// 指令处理侧对主控制器的回调契约
//
// 这五项都属于只由 MainController 持有的运行状态机操作：控制权申请、
// 任务取消、速度直控的进入与退出、地图参考重建。指令处理器不拥有这些
// 状态，只通过本接口请求，因此不必把整个 MainController 反向包含进来。
// 接口刻意只留这五项：一旦继续膨胀，说明该把状态本身挪出来了。
// ============================================================
class CommandHost
{
public:
    virtual ~CommandHost() = default;

    virtual void beginControlAcquire() = 0;
    virtual void cancelActiveTask(DogProtocol::MoveResult result, const QString &reason) = 0;
    virtual void enterVelocityMode() = 0;
    virtual void exitVelocityMode(bool sendZero) = 0;
    virtual void resetMapReference(double localX, double localY, double localYaw) = 0;
};

#endif  // COMMAND_HOST_H
