#ifndef MAP_TRANSFORM_H
#define MAP_TRANSFORM_H

#include <QString>
#include <cmath>

// ============================================================
// 狗原始局部坐标 <-> 本地地图坐标 变换
//
// 坐标约定：
//   局部坐标：+x 机头前方，+y 机体左侧，yaw 逆时针为正（狗 MCU 原始输出）
//   地图坐标：+x 正东，+y 正南；
//             yaw 0=正东，顺时针为正（0~360：0=东 90=南 180=西 270=北）
//   两者手性相反，因此变换含一次镜像（det = -1）。
//
// 变换参数：
//   R = (refX, refY)   标定参考点在局部坐标下的位置
//   refYaw             标定参考点在局部坐标下的 yaw（此时定义地图角 = 0，即正东）
//   O = (originX, originY) 参考点对应的地图坐标
//   scale              局部位移 -> 地图位移的比例（直线标定拟合，默认 1）
//
// 正变换（反射 T，T 自逆）：
//   r      = |L - R| * scale
//   theta  = atan2(L.y-R.y, L.x-R.x)     // 局部系(odom)内方向（逆时针为正）
//   a      = refYaw - theta              // 地图角（0=东，顺时针为正）
//   mapX   = originX + r * cos(a)
//   mapY   = originY + r * sin(a)
//   mapYaw = refYaw - localYaw
//
// 轴向依据（真狗数据验证 2026-11-12）：狗 odom 位移方位角 = IMU yaw（同轴、逆时针为正），
// 即机头方向在 odom 系中的方位角就是 localYaw，变换不含额外 90° 偏置。
// 手性：地图 +y=南（顺时针角）与局部 +y=左（逆时针角）相反，T 含一次镜像。
//
// 参考基准：参考以"终端启动（或手动重置）时狗的位姿"为准，与狗开机时刻无关——
// 狗可以先开机、先被遥控移动，终端启动那一刻狗所在点 = 地图 (0,0)、
// 狗体朝向 = 0°（正东）。之后可随时通过 resetReference 重新设定。
// 静止时 yaw 会随时间漂移，转向/行走时 yaw 准确，因此仍建议配合直线标定。
// ============================================================
class MapTransform
{
public:
    struct Params
    {
        bool enabled;             // 是否启用地图坐标
        double originX;           // 参考点对应的地图 x m
        double originY;           // 参考点对应的地图 y m
        bool autoReference;       // 终端启动后首个有效位姿是否自动建立参考
        bool jumpCompensate;      // 里程计坐标系跳变时平移参考，保持地图输出连续
        double scaleMin;          // 直线标定尺度下限
        double scaleMax;          // 直线标定尺度上限
        double maxHeadingErrRad;  // 直线标定允许的最大残余朝向误差（精确档）
        double coarseHeadingErrRad; // 粗标定允许的最大残余朝向误差（超出精确档但在此内=只修朝向）
        double minLineDist;       // 直线标定要求的最小局部位移 m

        Params()
            : enabled(false)
            , originX(0.0)
            , originY(0.0)
            , autoReference(true)
            , jumpCompensate(false)
            , scaleMin(0.7)
            , scaleMax(1.3)
            , maxHeadingErrRad(15.0 * 3.14159265358979323846 / 180.0)
            , coarseHeadingErrRad(40.0 * 3.14159265358979323846 / 180.0)
            , minLineDist(1.0) {}
    };

    // 标定分档结果
    enum CalibGrade
    {
        CalibRejected = 0,  // 拒绝
        CalibCoarse   = 1,  // 粗标定：只修参考朝向，尺度保持 1
        CalibExact    = 2   // 精确标定：同时修朝向与尺度
    };

    MapTransform();

    void setParams(const Params &p) { _params = p; }
    const Params &params() const { return _params; }

    bool hasReference() const { return _hasRef; }
    // 启用且已建立参考，才可做坐标变换
    bool isActive() const { return _params.enabled && _hasRef; }
    double scale() const { return _scale; }
    double refYaw() const { return _refYaw; }
    // 参考点在局部坐标下的位置（上报"相对参考的局部位移"时使用）
    double refLocalX() const { return _refX; }
    double refLocalY() const { return _refY; }

    // 参考重置：以当前位姿为参考点（地图 O 点、狗体朝向 = 0° 即正东）；scale 复位为 1
    void resetReference(double localX, double localY, double localYaw);

    // 参考平移：狗端里程计坐标系整体跳变（如遥控接管后 odom 归零）时调用，
    // 参考点同步平移，使 map 输出保持连续（狗未真实移动则地图位置不变）
    void shiftReference(double dLocalX, double dLocalY)
    {
        _refX += dLocalX;
        _refY += dLocalY;
    }

    // 参考朝向补偿：狗端局部的 yaw 基准整体跳变（断链重连、SDK 坐标系重置后
    // 常见 30~70° 的单帧台阶）时调用，refYaw 同步跟着跳，
    // 地图 yaw 因此保持连续，狗的"相对旋转 90°"不会把台阶算成真实转向。
    void shiftRefYaw(double dLocalYaw)
    {
        _refYaw = std::atan2(std::sin(_refYaw + dLocalYaw), std::cos(_refYaw + dLocalYaw));
    }

    // 直线标定：局部位移 vs 目标地图位移，拟合尺度与残余朝向。
    // 成功则更新尺度/参考朝向并返回 true；失败保持原参数不变并写 message。
    bool calibrateLine(double localX, double localY,
                       double targetMapX, double targetMapY,
                       QString *message);

    // 直线标定（可只预检不修改）。预算结果写入 grade（见 CalibGrade）：
    //   CalibExact  → apply=true 时更新 refYaw 与 scale
    //   CalibCoarse → apply=true 时只更新 refYaw，scale 保持 1
    //   CalibRejected → 参数不变
    bool calibrateLineEx(double localX, double localY,
                         double targetMapX, double targetMapY,
                         bool apply, CalibGrade *grade, QString *message);

    // 尺度外部加载（标定结果持久化后启动恢复）
    void setScale(double s) { _scale = (s > 1e-6) ? s : 1.0; }

    void clearReference() { _hasRef = false; }

    void toMap(double localX, double localY, double localYaw,
               double *mapX, double *mapY, double *mapYaw) const;
    void toLocal(double mapX, double mapY, double mapYaw,
                 double *localX, double *localY, double *localYaw) const;

    // 地图角：0=正东，顺时针为正，弧度
    double localYawToMapYaw(double localYaw) const;
    double mapYawToLocalYaw(double mapYaw) const;

    // 参数摘要（日志/反馈用）
    QString describe() const;

private:
    Params _params;
    bool _hasRef;
    double _refX;
    double _refY;
    double _refYaw;
    double _scale;
};

#endif  // MAP_TRANSFORM_H
