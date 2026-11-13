#include "MapTransform.h"

#include <cmath>

namespace
{

const double kPi = 3.14159265358979323846;
const double kRadToDeg = 180.0 / kPi;

double radStd(double rad)
{
    while (rad > kPi)
        rad -= 2.0 * kPi;
    while (rad <= -kPi)
        rad += 2.0 * kPi;
    return rad;
}

}  // namespace

MapTransform::MapTransform()
    : _hasRef(false)
    , _refX(0.0)
    , _refY(0.0)
    , _refYaw(0.0)
    , _scale(1.0)
{
}

void MapTransform::resetReference(double localX, double localY, double localYaw)
{
    _refX = localX;
    _refY = localY;
    _refYaw = radStd(localYaw);
    _scale = 1.0;
    _hasRef = true;
}

bool MapTransform::calibrateLine(double localX, double localY,
                                 double targetMapX, double targetMapY,
                                 QString *message)
{
    return calibrateLineEx(localX, localY, targetMapX, targetMapY, true, nullptr, message);
}

bool MapTransform::calibrateLineEx(double localX, double localY,
                                   double targetMapX, double targetMapY,
                                   bool apply, CalibGrade *grade, QString *message)
{
    if (grade)
        *grade = CalibRejected;

    if (!_hasRef)
    {
        if (message)
            *message = QStringLiteral("尚未建立参考，无法直线标定");
        return false;
    }

    const double dx = localX - _refX;
    const double dy = localY - _refY;
    const double lineLen = std::sqrt(dx * dx + dy * dy);
    if (lineLen < _params.minLineDist)
    {
        if (message)
            *message = QStringLiteral("局部位移仅 %1m，小于门限 %2m")
                           .arg(lineLen, 0, 'f', 2)
                           .arg(_params.minLineDist, 0, 'f', 2);
        return false;
    }

    const double dMx = targetMapX - _params.originX;
    const double dMy = targetMapY - _params.originY;
    const double mapLen = std::sqrt(dMx * dMx + dMy * dMy);
    if (mapLen < 1e-6)
    {
        if (message)
            *message = QStringLiteral("终点与地图原点重合，无法拟合尺度");
        return false;
    }

    const double k = mapLen / lineLen;
    if (k < _params.scaleMin || k > _params.scaleMax)
    {
        if (message)
            *message = QStringLiteral("尺度 %1 超出允许范围 [%2, %3]")
                           .arg(k, 0, 'f', 3)
                           .arg(_params.scaleMin, 0, 'f', 2)
                           .arg(_params.scaleMax, 0, 'f', 2);
        return false;
    }

    // 局部位移方向 -> 以当前参考朝向推算的地图角（0=东，顺时针为正）
    // odom 系与 IMU 系同轴（真狗数据验证：前进时机体位移方位角=IMU yaw），无 90° 偏置
    const double thetaL = std::atan2(dy, dx);
    const double predHeading = radStd(_refYaw - thetaL);
    // 目标位移方向：地图角 = atan2(南分量, 东分量)
    const double desHeading = std::atan2(dMy, dMx);
    const double phi = radStd(desHeading - predHeading);

    const double absPhi = std::fabs(phi);
    if (absPhi > _params.coarseHeadingErrRad)
    {
        if (message)
            *message = QStringLiteral("残余朝向误差 %1° 超出上限 ±%2°（确认是从参考点直线走出的）")
                           .arg(phi * kRadToDeg, 0, 'f', 2)
                           .arg(_params.coarseHeadingErrRad * kRadToDeg, 0, 'f', 1);
        return false;
    }

    // 分档：残余角在精确档内→采信尺度；否则按粗标定处理，只修参考朝向
    const bool precise = (absPhi <= _params.maxHeadingErrRad);
    if (grade)
        *grade = precise ? CalibExact : CalibCoarse;

    if (apply)
    {
        _refYaw = radStd(_refYaw + phi);
        _scale = precise ? k : 1.0;
    }

    if (message)
    {
        QString head;
        if (!apply)
            head = precise ? QStringLiteral("预检通过(精确档)：") : QStringLiteral("预检通过(粗标定档)：");
        else
            head = precise ? QStringLiteral("精确标定完成：") : QStringLiteral("粗标定完成：");
        *message = head
                   + QStringLiteral("位移 %1m，尺度 %2，残余朝向 %3°")
                         .arg(lineLen, 0, 'f', 2)
                         .arg(precise ? k : 1.0, 0, 'f', 3)
                         .arg(phi * kRadToDeg, 0, 'f', 2);
    }
    return true;
}

void MapTransform::toMap(double localX, double localY, double localYaw,
                         double *mapX, double *mapY, double *mapYaw) const
{
    if (!isActive())
    {
        if (mapX) *mapX = localX;
        if (mapY) *mapY = localY;
        if (mapYaw) *mapYaw = localYaw;
        return;
    }

    const double dx = localX - _refX;
    const double dy = localY - _refY;
    const double r = std::sqrt(dx * dx + dy * dy) * _scale;
    const double thetaL = std::atan2(dy, dx);
    // 地图角：0=东，顺时针为正（反射变换 T：W 角 θ -> 地图角 refYaw-θ，T 自逆）
    const double heading = radStd(_refYaw - thetaL);

    if (mapX) *mapX = _params.originX + r * std::cos(heading);
    if (mapY) *mapY = _params.originY + r * std::sin(heading);
    if (mapYaw) *mapYaw = localYawToMapYaw(localYaw);
}

void MapTransform::toLocal(double mapX, double mapY, double mapYaw,
                           double *localX, double *localY, double *localYaw) const
{
    if (!isActive())
    {
        if (localX) *localX = mapX;
        if (localY) *localY = mapY;
        if (localYaw) *localYaw = mapYaw;
        return;
    }

    const double mx = mapX - _params.originX;
    const double my = mapY - _params.originY;
    const double rMap = std::sqrt(mx * mx + my * my);
    // 地图角（0=东，顺时针为正）
    const double heading = std::atan2(my, mx);
    // 局部系内方向（逆时针为正）；T 自逆：theta = refYaw - heading
    const double thetaL = radStd(_refYaw - heading);
    const double r = (_scale > 1e-9) ? (rMap / _scale) : rMap;

    if (localX) *localX = _refX + r * std::cos(thetaL);
    if (localY) *localY = _refY + r * std::sin(thetaL);
    if (localYaw) *localYaw = mapYawToLocalYaw(mapYaw);
}

double MapTransform::localYawToMapYaw(double localYaw) const
{
    return radStd(_refYaw - localYaw);
}

double MapTransform::mapYawToLocalYaw(double mapYaw) const
{
    return radStd(_refYaw - mapYaw);
}

QString MapTransform::describe() const
{
    if (!_hasRef)
        return QStringLiteral("未建立参考");
    return QStringLiteral("参考局部(%1, %2) 原点(%3, %4) refYaw=%5° scale=%6")
        .arg(_refX, 0, 'f', 2)
        .arg(_refY, 0, 'f', 2)
        .arg(_params.originX, 0, 'f', 2)
        .arg(_params.originY, 0, 'f', 2)
        .arg(_refYaw * kRadToDeg, 0, 'f', 2)
        .arg(_scale, 0, 'f', 3);
}
