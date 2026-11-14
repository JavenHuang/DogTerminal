#ifndef COMMAND_HOST_H
#define COMMAND_HOST_H

#include <QString>

#include "Protocol/DogProtocol.h"

// ============================================================
// 指令处理侧对主控制器的回调契约
//
// 这六项都属于只由 MainController 持有的运行状态机操作：控制权申请、
// 任务取消、速度直控的进入与退出、地图参考重建、地图标定回填融合。
// 指令处理器不拥有这些状态，只通过本接口请求，因此不必把整个
// MainController 反向包含进来。接口刻意只留这六项：一旦继续膨胀，
// 说明该把状态本身挪出来了。
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
    // 地图标定（mapreset / mapline）成功后，把 Scale 与 refYaw 直接写给融合，
    // 现场不再依赖"4 个停点解 k/α"。由 Router 在标定成功分支里调用。
    virtual void applyMapCalibToFusion() = 0;
};

#endif  // COMMAND_HOST_H
