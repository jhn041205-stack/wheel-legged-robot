#ifndef __CUSTOM_TYPEDEF_H
#define __CUSTOM_TYPEDEF_H

#include "attribute_typedef.h"
#include "stdbool.h"
#include "struct_typedef.h"

// 数据名称宏
#define IMU_NAME "imu_data"
#define CHASSIS_FDB_SPEED_NAME "chassis_fdb_speed"
#define ROBOT_CMD_DATA_NAME "ROBOT_CMD_DATA"
#define USB_OFFLINE_NAME "usb_offline"
#define VIRTUAL_RC_NAME "virtual_rc_ctrl"
#define CHASSIS_SOLVED_RC_CMD_NAME "solved_rc_cmd"

typedef struct __Imu
{
    float angle[3];  // rad 欧拉角数据
    float gyro[3];   // rad/s 陀螺仪数据
    float accel[3];  // m/s^2 加速度计数据
} Imu_t;

typedef struct  // 底盘速度向量结构体
{
    float vx;  // (m/s) x方向速度
    float vy;  // (m/s) y方向速度
    float wz;  // (rad/s) 旋转速度
} ChassisSpeedVector_t;

typedef struct
{
    ChassisSpeedVector_t speed_vector;
    struct
    {
        float roll;
        float pitch;
        float yaw;
        float leg_length;
    } chassis;

    struct
    {
        bool fire;
        bool fric_on;
    } shoot;

} RobotCmdData_t;

typedef struct
{
    uint8_t mode;
    uint8_t step;
    uint8_t rc_offline;
    uint8_t reserved;

    float vx;  // m/s
    float vy;  // m/s
    float wz;  // rad/s
    float roll;          // rad
    float pitch;         // rad
    float yaw;           // rad
    float leg_length_l;  // m
    float leg_length_r;  // m
    float leg_angle_l;   // rad
    float leg_angle_r;   // rad
    float tail_beta;     // rad
} __packed__ ChassisSolvedRcCmd_t;

#endif  // __CUSTOM_TYPEDEF_H
