/**
  * @file       robot_param_balanced_infantry.h
  * @brief      杩欓噷鏄钩琛℃鍏垫満鍣ㄤ汉鍙傛暟閰嶇疆鏂囦欢锛屽寘鎷墿鐞嗗弬鏁般€丳ID鍙傛暟绛?  */

#ifndef INCLUDED_ROBOT_PARAM_H
#define INCLUDED_ROBOT_PARAM_H
#include "robot_typedef.h"

// clang-format off
#define __SELF_BOARD_ID C_BOARD_BALANCE_CHASSIS // 鏈澘ID
#define __GYRO_BIAS_YAW  0.003096855f           // 闄€铻轰华闆堕锛屽崟浣峳ad/s

#define __CONTROL_LINK_RC CL_RC_DIRECT
#define __USB_RC_SOURCE USB_RC_SOURCE_ROBOT_CMD
#define __CONTROL_LINK_KM CL_KM_NONE

/*-------------------- Chassis --------------------*/
#define CHASSIS_TASK_INIT_TIME 357
#define CHASSIS_CONTROL_TIME_MS 2
#define CHASSIS_CONTROL_TIME_S (CHASSIS_CONTROL_TIME_MS / 1000.0f)

// 搴曠洏鐨勯仴鎺у櫒鐩稿叧瀹忓畾涔?---------------------
#define CHASSIS_MODE_CHANNEL 1
#define CHASSIS_FUNCTION 0
#define CHASSIS_X_CHANNEL 3
#define CHASSIS_Y_CHANNEL 2
#define CHASSIS_WZ_CHANNEL 2
#define CHASSIS_ANGLE_CHANNEL 1
#define CHASSIS_LENGTH_CHANNEL 0
#define CHASSIS_ROLL_CHANNEL 0
#define CHASSIS_PITCH_CHANNEL 1
#define CHASSIS_TAIL_POS_CHANNEL 1
#define CHASSIS_RC_DEADLINE 20
// 0-鍙冲钩, 1-鍙崇珫, 2-宸﹀钩, 3-宸︾珫, 4-宸︽粴杞?
// deadzone parameters ---------------------
#define WHEEL_DEADZONE (0.01f)  // (m/s)杞瓙閫熷害姝诲尯

// ratio parameters ---------------------
#define FF_RATIO               (1.0f)   // 鍓嶉姣斾緥绯绘暟
#define ROLL_VEL_LIMIT_FACTOR  (0.1f)    // roll瑙掗€熷害鎶戝埗姣斾緥绯绘暟

// motor parameters ---------------------
#define JOINT_CAN (2)
#define WHEEL_CAN (1)

#define J0_DIRECTION (-1)
#define J1_DIRECTION (1)
#define J2_DIRECTION (1)
#define J3_DIRECTION (-1)

#define W0_DIRECTION ( 1)
#define W1_DIRECTION (-1)

#define T_DIRECTION (-1)

//physical parameters ---------------------
#define LEG_L1 (0.215f)  // (m)鑵?闀垮害
#define LEG_L2 (0.258f)  // (m)鑵?闀垮害
#define LEG_L3 (LEG_L2)  // (m)鑵?闀垮害
#define LEG_L4 (LEG_L1)  // (m)鑵?闀垮害
#define LEG_L5 (0.0f)    // (m)鍏宠妭闂磋窛

#define BODY_MASS            (13.0f)      // (kg)鏈鸿韩閲嶉噺
#define WHEEL_MASS           (0.82f)      // (kg)杞瓙閲嶉噺
#define TAIL_MASS            (0.86f)      // (kg)杞瓙閲嶉噺 v1.0
#define WHEEL_RADIUS         (0.13f)    // (m)杞瓙鍗婂緞
#define TAIL_WHEEL_RADIUS    (0.029f)
#define WHEEL_START_TORQUE   (0.3f)      // (Nm)杞瓙璧峰姩鍔涚煩
#define WHEEL_BASE           (0.386f)
#define WHEEL_BASE_2         (0.193f)
// v1.1
// #define TAIL_COM_to_MOTOR    (0.17755f)
// #define TAIL_BETA_COM_to_HAND    (0.065972f)
// #define TAIL_BETA_OMNI_to_HAND    (0.38397f)
// v1.0
#define TAIL_COM_to_MOTOR    (0.23985f)
#define TAIL_BETA_COM_to_HAND    (0.1034f)
#define TAIL_BETA_OMNI_to_HAND    (0.3141f)
#define TAIL_POS_OFFSET (0.089f)
#define TAIL_POS_OFFSET_HORIZON (0.0f)
#define TAIL_POS_OFFSET_VERTICAL (0.089f)
#define TAIL_LENGTH     (0.330f)

#define J0_ANGLE_OFFSET     (0.0f + M_PI) // (rad)鍏宠妭0瑙掑害鍋忕Щ閲?鐢垫満0鐐瑰埌姘村钩绾跨殑澶硅)
#define J1_ANGLE_OFFSET     (0.0f)         // (rad)鍏宠妭1瑙掑害鍋忕Щ閲?鐢垫満0鐐瑰埌姘村钩绾跨殑澶硅)
#define J2_ANGLE_OFFSET     (0.0f - M_PI)  // (rad)鍏宠妭2瑙掑害鍋忕Щ閲?鐢垫満0鐐瑰埌姘村钩绾跨殑澶硅)
#define J3_ANGLE_OFFSET     (0.0f + DOUBLE_PI)        // (rad)鍏宠妭3瑙掑害鍋忕Щ閲?鐢垫満0鐐瑰埌姘村钩绾跨殑澶硅)

#define T_ANGLE_OFFSET     (0.0f)

#define DLENGTH_DIRECTION  (-1) // ROLL瑙掕ˉ鍋块噺鏂瑰悜(鑵块暱澧炲姞鏂瑰悜)
//upper_limit parameters ---------------------

#define MAX_DELTA_ROD_ANGLE (0.3f) // (rad)鑵挎憜瑙掓渶澶у彉鍖栭噺
#define MAX_TORQUE_PROTECT  (25.0f)  // (Nm)鏈€澶ф壄鐭╀繚鎶?
#define MAX_DELTA_VEL_FDB_TO_REF (0.8f) // (m/s)閫熷害鍙嶉鍒板弬鑰冮€熷害鐨勬渶澶у彉鍖栭噺

#define MAX_THETA      (1.0f)
#define MAX_THETA_DOT  (2.0f)
#define MAX_X          (1.0f)
#define MAX_X_DOT      (10.0f)
#define MAX_PHI        (1.0f)
#define MAX_PHI_DOT    (2.0f)

#define MAX_SPEED_INTEGRAL  (0.5f)
#define MAX_ROLL            (0.3f)
#define MAX_PITCH           (0.3f)
#define MAX_ROLL_VELOCITY   (1.0f)
#define MAX_YAW             (M_PI)
#define MAX_YAW_VELOCITY    (3.0f)

#define MAX_J0_ANGLE  (1.2f) // (rad)鍏宠妭瑙掑害涓婇檺
#define MAX_J1_ANGLE  (1.2f) // (rad)鍏宠妭瑙掑害涓婇檺
#define MAX_J2_ANGLE  (1.2f) // (rad)鍏宠妭瑙掑害涓婇檺
#define MAX_J3_ANGLE  (1.2f) // (rad)鍏宠妭瑙掑害涓婇檺

#define MAX_LEG_LENGTH       (0.3f)
#define MAX_LEG_ANGLE        (MAX_DELTA_ROD_ANGLE)
#define MAX_TAIL_ANGLE       (M_PI_2)
#define MAX_SPEED            (3.5f)
#define MAX_SPEED_VECTOR_VX  (3.5f)
#define MAX_SPEED_VECTOR_VY  (3.5f)
#define MAX_SPEED_VECTOR_WZ  (6.0f)

#define MAX_JOINT_TORQUE      (10.0f)
#define MAX_JOINT_TORQUE_JUMP (20.0f)
#define MAX_VEL_ADD           (1.0f)
#define MAX_PITCH_VEL         (0.1f)

#define MAX_TOUCH_INTERVAL    (200)    // (ms)鏈€澶х鍦版椂闂达紝瓒呰繃杩欎釜鏃堕棿璁や负绂诲湴

//lower_limit parameters ---------------------

#define MIN_DELTA_ROD_ANGLE (-MAX_DELTA_ROD_ANGLE) // (rad)鑵挎憜瑙掓渶灏忓彉鍖栭噺

#define MIN_DELTA_VEL_FDB_TO_REF (-MAX_DELTA_VEL_FDB_TO_REF) // (m/s)閫熷害鍙嶉鍒板弬鑰冮€熷害鐨勬渶灏忓彉鍖栭噺

#define MIN_THETA      (-MAX_THETA)
#define MIN_THETA_DOT  (-MAX_THETA_DOT)
#define MIN_X          (-MAX_X)
#define MIN_X_DOT      (-MAX_X_DOT)
#define MIN_PHI        (-MAX_PHI)
#define MIN_PHI_DOT    (-MAX_PHI_DOT)

#define MIN_SPEED_INTEGRAL  (-MAX_SPEED_INTEGRAL)
#define MIN_ROLL            (-MAX_ROLL)
#define MIN_PITCH           (-MAX_PITCH)
#define MIN_ROLL_VELOCITY   (-MAX_ROLL_VELOCITY)
#define MIN_YAW             (-MAX_YAW)
#define MIN_YAW_VELOCITY    (-MAX_YAW_VELOCITY)

#define MIN_J0_ANGLE (-0.47f) // (rad)鍏宠妭瑙掑害涓嬮檺
#define MIN_J1_ANGLE (-0.27f) // (rad)鍏宠妭瑙掑害涓嬮檺
#define MIN_J2_ANGLE (-0.47f) // (rad)鍏宠妭瑙掑害涓嬮檺
#define MIN_J3_ANGLE (-0.27f) // (rad)鍏宠妭瑙掑害涓嬮檺

#define MIN_LEG_LENGTH       ( 0.15f)
#define MIN_LEG_ANGLE        ( - MAX_DELTA_ROD_ANGLE)
#define MIN_TAIL_ANGLE       ( 0.0f)
#define MIN_SPEED            (-MAX_SPEED)
#define MIN_SPEED_VECTOR_VX  (-MAX_SPEED_VECTOR_VX)
#define MIN_SPEED_VECTOR_VY  (-MAX_SPEED_VECTOR_VY)
#define MIN_SPEED_VECTOR_WZ  (-MAX_SPEED_VECTOR_WZ)

#define MIN_JOINT_TORQUE      (-MAX_JOINT_TORQUE)  // 
#define MIN_JOINT_TORQUE_JUMP (-MAX_JOINT_TORQUE_JUMP)  // 
#define MIN_VEL_ADD           (-MAX_VEL_ADD)    // (m/s)閫熷害澧為噺涓嬮檺
#define MIN_PITCH_VEL         (-MAX_PITCH_VEL)  // (rad/s)pitch杞撮€熷害涓嬮檺

//PID parameters ---------------------
//yaw杞磋窡韪搴︾幆PID鍙傛暟
#define KP_CHASSIS_YAW_ANGLE        (31.6227766016838f)
#define KI_CHASSIS_YAW_ANGLE        (0.0f)
#define KD_CHASSIS_YAW_ANGLE        (4.93619232696934f)
#define MAX_IOUT_CHASSIS_YAW_ANGLE  (0.0f)
#define MAX_OUT_CHASSIS_YAW_ANGLE   (3.5f)

// #define KP_CHASSIS_YAW_ANGLE        (2.3f)
// #define KI_CHASSIS_YAW_ANGLE        (1.0f)
// #define KD_CHASSIS_YAW_ANGLE        (0.0f)
// #define MAX_IOUT_CHASSIS_YAW_ANGLE  (0.5f)
// #define MAX_OUT_CHASSIS_YAW_ANGLE   (5.0f)

//yaw杞磋窡韪€熷害鐜疨ID鍙傛暟
// #define KP_CHASSIS_YAW_VELOCITY        (1.0f)
// #define KI_CHASSIS_YAW_VELOCITY        (0.01f)
// #define KD_CHASSIS_YAW_VELOCITY        (0.02f)
// #define MAX_IOUT_CHASSIS_YAW_VELOCITY  (0.1f)
// #define MAX_OUT_CHASSIS_YAW_VELOCITY   (2.0f)

#define KP_CHASSIS_YAW_VELOCITY        (3.0f)
#define KI_CHASSIS_YAW_VELOCITY        (0.5f)
#define KD_CHASSIS_YAW_VELOCITY        (0.1f)
#define MAX_IOUT_CHASSIS_YAW_VELOCITY  (0.5f)
#define MAX_OUT_CHASSIS_YAW_VELOCITY   (2.0f)

// vel_add PID鍙傛暟
#define KP_CHASSIS_VEL_ADD        (0.1f) //0.1
#define KI_CHASSIS_VEL_ADD        (0.005f)//0.005
#define KD_CHASSIS_VEL_ADD        (0.001f)//0.001
#define MAX_IOUT_CHASSIS_VEL_ADD  (0.8f)//0.5
#define MAX_OUT_CHASSIS_VEL_ADD   (1.5f)//1.0

/*========== Start of locomotion control pid ==========*/

//roll杞磋窡韪搴︾幆PID鍙傛暟
#define KP_CHASSIS_ROLL_ANGLE        (0.0f)
#define KI_CHASSIS_ROLL_ANGLE        (0.0f)
#define KD_CHASSIS_ROLL_ANGLE        (0.0f)
#define MAX_IOUT_CHASSIS_ROLL_ANGLE  (0.0f)
#define MAX_OUT_CHASSIS_ROLL_ANGLE   (20.0f)
#define N_CHASSIS_ROLL_ANGLE         (0.1f)

// 涓よ吙涓€鑷碢ID鍙傛暟
#define KP_CHASSIS_LEG_COOR        (50.0f)
#define KI_CHASSIS_LEG_COOR        (0.0f)
#define KD_CHASSIS_LEG_COOR        (25.0f)
#define MAX_IOUT_CHASSIS_LEG_COOR  (1.0f)
#define MAX_OUT_CHASSIS_LEG_COOR   (6.0f)
#define N_CHASSIS_LEG_COOR         (0.0f)
#define ERRORSUM_UP_LEG_COOR       (0.20f)
#define ERRORSUM_LOW_LEG_COOR      (-ERRORSUM_UP_LEG_COOR)

// 灏惧反閫傚簲鍦板舰琛ュ伩PID鍙傛暟
#define KP_CHASSIS_TAIL_COMP        (0.5f)
#define KI_CHASSIS_TAIL_COMP        (0.0f)
#define KD_CHASSIS_TAIL_COMP        (0.01f)
#define MAX_IOUT_CHASSIS_TAIL_COMP  (0.0f)
#define MAX_OUT_CHASSIS_TAIL_COMP   (0.15f)
#define N_CHASSIS_TAIL_COMP         (0.0f)

// 灏惧反鎶崌琛ュ伩PID鍙傛暟
#define KP_CHASSIS_TAIL_UP        (0.0f)
#define KI_CHASSIS_TAIL_UP        (0.0f)
#define KD_CHASSIS_TAIL_UP        (0.0f)
#define MAX_IOUT_CHASSIS_TAIL_UP  (0.0f)
#define MAX_OUT_CHASSIS_TAIL_UP   (0.5f)
#define N_CHASSIS_TAIL_UP         (0.0f)

// 灏惧反鏈杞瓙绂诲湴楂樺害PID鍙傛暟
#define KP_CHASSIS_TAIL_Z        (300.0f)
#define KI_CHASSIS_TAIL_Z        (0.0f)
#define KD_CHASSIS_TAIL_Z        (18.0f)
#define MAX_IOUT_CHASSIS_TAIL_Z  (0.0f)
#define MAX_OUT_CHASSIS_TAIL_Z   (5.0f)
#define N_CHASSIS_TAIL_Z         (0.0f)

//roll杞磋窡韪€熷害鐜疨ID鍙傛暟
// #define KP_CHASSIS_ROLL_VELOCITY        (0.6f)
// #define KI_CHASSIS_ROLL_VELOCITY        (0.0001f)
// #define KD_CHASSIS_ROLL_VELOCITY        (0.005f)
// #define MAX_IOUT_CHASSIS_ROLL_VELOCITY  (0.01f)
// #define MAX_OUT_CHASSIS_ROLL_VELOCITY   (0.1f)

// 鑵块暱璺熻釜闀垮害鐜疨ID鍙傛暟
#define KP_CHASSIS_LEG_LENGTH_LENGTH        (600.0f)
#define KI_CHASSIS_LEG_LENGTH_LENGTH        (3.5f)
#define KD_CHASSIS_LEG_LENGTH_LENGTH        (620.0f)
#define MAX_IOUT_CHASSIS_LEG_LENGTH_LENGTH  (40.0f)
#define MAX_OUT_CHASSIS_LEG_LENGTH_LENGTH   (80.0f)
#define N_LEG_LENGTH_LENGTH                 (0.1f)
#define ERRORSUM_UP_LEG_LENGTH              (0.25f)
#define ERRORSUM_LOW_LEG_LENGTH             (-ERRORSUM_UP_LEG_LENGTH)

// theta琛ュ伩PID鍙傛暟
#define KP_CHASSIS_THETA_COMP        (0.0f)
#define KI_CHASSIS_THETA_COMP        (0.02f)
#define KD_CHASSIS_THETA_COMP        (0.0f)
#define MAX_IOUT_CHASSIS_THETA_COMP  (3.0f)
#define MAX_OUT_CHASSIS_THETA_COMP   (3.0f)
#define N_CHASSIS_THETA_COMP                 (0.1f)
#define ERRORSUM_UP_THETA_COMP              (0.3f)
#define ERRORSUM_LOW_THETA_COMP             (-ERRORSUM_UP_LEG_LENGTH)

// 鑵块暱璺熻釜闀垮害鐜疨ID鍙傛暟
// #define KP_CHASSIS_LEG_LENGTH_LENGTH        (150.0f)
// #define KI_CHASSIS_LEG_LENGTH_LENGTH        (0.0f)
// #define KD_CHASSIS_LEG_LENGTH_LENGTH        (1500.0f)
// #define MAX_IOUT_CHASSIS_LEG_LENGTH_LENGTH  (0.0f)
// #define MAX_OUT_CHASSIS_LEG_LENGTH_LENGTH   (40.0f)
// #define N_LEG_LENGTH_LENGTH                 (0.1f)

// 鑵块暱璺熻釜閫熷害鐜疨ID鍙傛暟
// #define KP_CHASSIS_LEG_LENGTH_SPEED 0.0f
// #define KI_CHASSIS_LEG_LENGTH_SPEED 0.0f
// #define KD_CHASSIS_LEG_LENGTH_SPEED 0.0f
// #define MAX_IOUT_CHASSIS_LEG_LENGTH_SPEED 0.0f
// #define MAX_OUT_CHASSIS_LEG_LENGTH_SPEED 0.0f

/*========== End of locomotion control pid ==========*/

// 璧风珛鐢ㄧ殑pid
#define KP_CHASSIS_STAND_UP       (2000.0f)
#define KI_CHASSIS_STAND_UP       (0.0f)
#define KD_CHASSIS_STAND_UP       (10.0f)
#define MAX_IOUT_CHASSIS_STAND_UP (0.0f)
#define MAX_OUT_CHASSIS_STAND_UP  (2000.0f)

// 杞瓙鍋滄鐢ㄧ殑pid
#define KP_CHASSIS_WHEEL_STOP       (0.3f)
#define KI_CHASSIS_WHEEL_STOP       (0.0f)
#define KD_CHASSIS_WHEEL_STOP       (0.1f)
#define MAX_IOUT_CHASSIS_WHEEL_STOP (0.0f)
#define MAX_OUT_CHASSIS_WHEEL_STOP  (200.0f)

//LPF parameters ---------------------
#define LEG_DDL0_LPF_ALPHA           (0.9f)
#define LEG_DDPHI0_LPF_ALPHA         (0.9f)
#define LEG_DDTHETA_LPF_ALPHA        (0.9f)
#define LEG_SUPPORT_FORCE_LPF_ALPHA  (0.9f)
#define CHASSIS_ROLL_ALPHA           (0.5f)

//offest parameters ---------------------

#define X0_OFFSET  (0.0f)
#define X1_OFFSET  (0.0f)
#define X2_OFFSET  (0.0f)
#define X3_OFFSET  (0.0f)
#define X4_OFFSET  (0.0f)
#define X5_OFFSET  (0.0f)
#define X6_OFFSET  (0.0f)
#define X7_OFFSET  (0.0f)
#define X8_OFFSET  (0.0f)
#define X9_OFFSET  (0.0f)
#define X10_OFFSET (0.0f)
#define X11_OFFSET (0.0f)
//other parameters ---------------------

// clang-format on
#endif /* INCLUDED_ROBOT_PARAM_H */



