#ifndef DOG_TYPES_H
#define DOG_TYPES_H

#include <QString>
#include <QList>

// ============================================================
// 南向链路公共数据结构
// UdpDogClient（UDP+JSON）与 SdkDogClient（官方 libzsibot）共用，
// 保证上层 MainController 不感知底层实现差异。
// ============================================================

// IMU 数据
struct DogImu
{
    float quat[4];       // 四元数 [w,x,y,z]
    float rpy[3];        // 欧拉角 [roll,pitch,yaw] 弧度
    float gyro[3];       // 机体角速度 rad/s
    float acc[3];        // 机体加速度 m/s2
    bool gyroField;      // 报文中确实带 gyro 字段（老固件不发时置 false，航向改用里程计角速度）
    bool valid;

    DogImu() : gyroField(false), valid(false)
    {
        quat[0] = 1.0f; quat[1] = quat[2] = quat[3] = 0.0f;
        rpy[0] = rpy[1] = rpy[2] = 0.0f;
        gyro[0] = gyro[1] = gyro[2] = 0.0f;
        acc[0] = acc[1] = acc[2] = 0.0f;
    }
};

// 里程计数据（狗上电位姿为局部原点）
struct DogOdom
{
    float position[3];     // 局部世界系位置 m
    float vWorld[3];       // 局部世界系速度 m/s
    float omegaWorld[3];   // 局部世界系角速度 rad/s
    float vBody[3];        // 机体系速度 m/s
    float omegaBody[3];    // 机体系角速度 rad/s
    bool valid;

    DogOdom() : valid(false)
    {
        for (int i = 0; i < 3; ++i)
        {
            position[i] = 0.0f; vWorld[i] = 0.0f; omegaWorld[i] = 0.0f;
            vBody[i] = 0.0f; omegaBody[i] = 0.0f;
        }
    }
};

// 模式反馈（值为SDK枚举，-1未知）
struct DogFeedback
{
    int functionMode;
    int controlMode;
    int motionMode;
    int motionType;
    int speedLevel;  // SDK SpeedLevel
    bool valid;

    DogFeedback()
        : functionMode(-1), controlMode(-1), motionMode(-1), motionType(-1)
        , speedLevel(-1), valid(false) {}
};

// 电池信息
struct DogBattery
{
    float current;    // A
    int error;        // 错误码
    int power;        // 电量百分比
    float temp;       // ℃
    float volt;       // V
    bool valid;

    DogBattery() : current(0), error(0), power(-1), temp(0), volt(0), valid(false) {}
};

// 基础信息
struct DogBasic
{
    quint32 power;       // 电量百分比
    float temperature;   // ℃
    float speed;         // 前后速度（狗机体系）
    QString sn;
    QString devName;
    QString ssid;
    int model;           // SDK Model，-1未知
    bool valid;

    DogBasic() : power(0), temperature(0), speed(0), model(-1), valid(false) {}
};

// 故障信息
struct DogFault
{
    QString module;
    QString submodule;
    quint32 errorCode;
    QString level;
    QString info;
};

#endif  // DOG_TYPES_H
