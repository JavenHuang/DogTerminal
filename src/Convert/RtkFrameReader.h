#ifndef RTK_FRAME_READER_H
#define RTK_FRAME_READER_H

#include <QByteArray>
#include <QtGlobal>

#include "Protocol/DogProtocol.h"

// ============================================================
// 差分载荷读取器
//
// 只做一件事：把 la 路由键上的载荷读成 AbsPoseCmd。
//   1) 去 BOM、判空；
//   2) JSON（含"数组包一层"的服务端写法）按候选键名取值；
//   3) 否则按 QDataStream(Qt_5_12) 反序列化，并核对数值是否合理；
//   4) 差分 ID 过滤（0 表示本机不校验 ID）；
//   5) 前 3 帧打印原始报文，供现场核对字段名与字节序。
// 不做坐标系换算、不判跳变、不进融合 —— 那些属于 RtkPoseTracker / RtkJumpGate。
// ============================================================
class RtkFrameReader
{
public:
    // 解析成功返回 true 并填充 cmd；失败/ID 不匹配返回 false（已打日志）
    // expectedDiffId：本机差分 ID，0 = 不校验
    bool parse(const QByteArray &body, qint32 expectedDiffId, DogProtocol::AbsPoseCmd *cmd);

private:
    int _rawPrinted = 0;   // 只打前几帧，避免刷屏
    bool _idWarned = false;
};

#endif  // RTK_FRAME_READER_H
