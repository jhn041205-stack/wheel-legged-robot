/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       usb_task.c/h
  * @brief      閫氳繃USB涓插彛涓庝笂浣嶆満閫氫俊
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jun-24-2024     Penguin         1. done

  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/

#include "usb_task.h"

#include <stdbool.h>
#include <string.h>

#include "CRC8_CRC16.h"
#include "cmsis_os.h"
#include "data_exchange.h"
#include "macro_typedef.h"
#include "usb_debug.h"
#include "usb_device.h"
#include "usb_typdef.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"
#include "supervisory_computer_cmd.h"
#include "IMU.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t usb_high_water;
#endif

#ifndef __USB_RC_SOURCE
#define __USB_RC_SOURCE USB_RC_SOURCE_VIRTUAL_RC
#endif

#define USB_TASK_CONTROL_TIME 1  // ms

#define USB_OFFLINE_THRESHOLD 100  // ms
#define USB_CONNECT_CNT 10

// clang-format off

#define SEND_DURATION_Imu         5   // ms
#define SEND_DURATION_RobotStateInfo   10  // ms
#define SEND_DURATION_RobotMotion 10  // ms
#define SEND_DURATION_RobotStatus  10// ms

// clang-format on

#define USB_RX_DATA_SIZE 256  // byte
#define USB_RECEIVE_LEN 150   // byte
#define HEADER_SIZE 4         // byte

#define CheckDurationAndSend(send_name)                                                  \
    do {                                                                                 \
        if ((HAL_GetTick() - LAST_SEND_TIME.##send_name) >= SEND_DURATION_##send_name) { \
            LAST_SEND_TIME.##send_name = HAL_GetTick();                                  \
            UsbSend##send_name##Data();                                                  \
        }                                                                                \
    } while (0)

// Variable Declarations
static uint8_t USB_RX_BUF[USB_RX_DATA_SIZE];

static const Imu_t * IMU;
static const ChassisSpeedVector_t * FDB_SPEED_VECTOR;

// USB connection state.
static bool USB_OFFLINE = true;
static uint32_t RECEIVE_TIME = 0;
static uint32_t LATEST_RX_TIMESTAMP = 0;
static uint32_t CONTINUE_RECEIVE_CNT = 0;

// 鏁版嵁鍙戦€佺粨鏋勪綋
// clang-format off
static SendDataImu_s         SEND_DATA_IMU;
static SendDataRobotMotion_s SEND_ROBOT_MOTION_DATA;

// clang-format on

// Receive frame storage.
static ReceiveDataRobotCmd_s RECEIVE_ROBOT_CMD_DATA;
static ReceiveDataVirtualRc_s RECEIVE_VIRTUAL_RC_DATA;

// Parsed robot command data.
RobotCmdData_t ROBOT_CMD_DATA;
static RC_ctrl_t VIRTUAL_RC_CTRL;

// USB send timestamps.
typedef struct
{
    uint32_t Imu;
    uint32_t RobotMotion;
} LastSendTime_t;
static LastSendTime_t LAST_SEND_TIME;

/*******************************************************************************/
/* Main Function                                                               */
/*******************************************************************************/
static void UsbSendData(void);
static void UsbReceiveData(void);
static void UsbInit(void);

/*******************************************************************************/
/* Send Function                                                               */
/*******************************************************************************/
static void UsbSendImuData(void);
static void UsbSendRobotMotionData(void);

/*******************************************************************************/
/* Receive Function                                                            */
/*******************************************************************************/
static void GetCmdData(void);
static void GetVirtualRcCtrlData(void);
static void TranslateRobotCmdToRcCtrl(RC_ctrl_t * rc_ctrl, const RobotCmdData_t * robot_cmd);

/******************************************************************/
/* Task                                                           */
/******************************************************************/

/**
 * @brief      USB浠诲姟涓诲嚱鏁? * @param[in]  argument: 浠诲姟鍙傛暟
 * @retval     None
 */
void usb_task(void const * argument)
{
    Publish(&ROBOT_CMD_DATA, ROBOT_CMD_DATA_NAME);
    Publish(&USB_OFFLINE, USB_OFFLINE_NAME);
    Publish(&VIRTUAL_RC_CTRL, VIRTUAL_RC_NAME);

    MX_USB_DEVICE_Init();

    vTaskDelay(10);
    UsbInit();

    while (1) {
        UsbSendData();
        UsbReceiveData();
        GetCmdData();
        GetVirtualRcCtrlData();

        if (HAL_GetTick() - RECEIVE_TIME > USB_OFFLINE_THRESHOLD) {
            USB_OFFLINE = true;
            CONTINUE_RECEIVE_CNT = 0;
        } else if (CONTINUE_RECEIVE_CNT > USB_CONNECT_CNT) {
            USB_OFFLINE = false;
        } else {
            CONTINUE_RECEIVE_CNT++;
        }

        RefreshRcControlSource();

        vTaskDelay(USB_TASK_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
        usb_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/*******************************************************************************/
/* Main Function                                                               */
/*******************************************************************************/

/**
 * @brief      USB鍒濆鍖? * @param      None
 * @retval     None
 */
static void UsbInit(void)
{
    // 璁㈤槄鏁版嵁
    IMU = Subscribe(IMU_NAME);                             // 鑾峰彇IMU鏁版嵁鎸囬拡
    FDB_SPEED_VECTOR = Subscribe(CHASSIS_FDB_SPEED_NAME);  // 鑾峰彇搴曠洏閫熷害鐭㈤噺鎸囬拡

    // 鏁版嵁缃浂
    memset(&LAST_SEND_TIME, 0, sizeof(LastSendTime_t));
    memset(&RECEIVE_ROBOT_CMD_DATA, 0, sizeof(ReceiveDataRobotCmd_s));
    memset(&RECEIVE_VIRTUAL_RC_DATA, 0, sizeof(ReceiveDataVirtualRc_s));
    memset(&ROBOT_CMD_DATA, 0, sizeof(RobotCmdData_t));
    memset(&VIRTUAL_RC_CTRL, 0, sizeof(RC_ctrl_t));

    /*******************************************************************************/
    /* Serial                                                                     */
    /*******************************************************************************/    
    // Initialize IMU frame header.
    SEND_DATA_IMU.frame_header.sof = SEND_SOF;
    SEND_DATA_IMU.frame_header.len = (uint8_t)(sizeof(SendDataImu_s) - 6);
    SEND_DATA_IMU.frame_header.id = IMU_DATA_SEND_ID;
    append_CRC8_check_sum(
        (uint8_t *)(&SEND_DATA_IMU.frame_header), sizeof(SEND_DATA_IMU.frame_header));
    
    // 8.鍒濆鍖栨満鍣ㄤ汉杩愬姩鏁版嵁
    SEND_ROBOT_MOTION_DATA.frame_header.sof = SEND_SOF;
    SEND_ROBOT_MOTION_DATA.frame_header.len = (uint8_t)(sizeof(SendDataRobotMotion_s) - 6);
    SEND_ROBOT_MOTION_DATA.frame_header.id = ROBOT_MOTION_DATA_SEND_ID;
    append_CRC8_check_sum(
        (uint8_t *)(&SEND_ROBOT_MOTION_DATA.frame_header),
        sizeof(SEND_ROBOT_MOTION_DATA.frame_header));
}

/**
 * @brief      鐢║SB鍙戦€佹暟鎹? * @param      None
 * @retval     None
 */
static void UsbSendData(void)
{
    // 鍙戦€両mu鏁版嵁
    CheckDurationAndSend(Imu);
    // 鍙戦€丷obotMotion鏁版嵁
    CheckDurationAndSend(RobotMotion);
}

/**
 * @brief      USB鎺ユ敹鏁版嵁
 * @param      None
 * @retval     None
 */
static void UsbReceiveData(void)
{
    static uint32_t len = USB_RECEIVE_LEN;
    static uint8_t * rx_data_start_address = USB_RX_BUF;
    static uint8_t * rx_data_end_address;
    uint8_t * sof_address = USB_RX_BUF;
    uint8_t * rx_buffer_limit;

    // Valid receive window: [rx_data_start_address, rx_data_start_address + USB_RECEIVE_LEN - 1].
    rx_buffer_limit = rx_data_start_address + USB_RECEIVE_LEN;
    rx_data_end_address = rx_buffer_limit - 1;
    // 璇诲彇鏁版嵁
    USB_Receive(rx_data_start_address, &len);  // Read data into the buffer

    while (sof_address <= rx_data_end_address) {  // 瑙ｆ瀽缂撳啿鍖轰腑鐨勬墍鏈夋暟鎹寘
        // 瀵绘壘甯уご浣嶇疆
        while ((sof_address <= rx_data_end_address) && (*sof_address != RECEIVE_SOF)) {
            sof_address++;
        }
        // 鍒ゆ柇鏄惁瓒呭嚭鎺ユ敹鏁版嵁鑼冨洿
        if (sof_address > rx_data_end_address) {
            break;
        }
        // 妫€鏌RC8鏍￠獙
        bool crc8_ok = verify_CRC8_check_sum(sof_address, HEADER_SIZE);
        if (crc8_ok) {
            uint8_t data_len = sof_address[1];
            uint8_t data_id = sof_address[2];
            // 妫€鏌ユ暣鍖匔RC16鏍￠獙 4: header size, 2: crc16 size
            bool crc16_ok = verify_CRC16_check_sum(sof_address, 4 + data_len + 2);
            if (crc16_ok) {
                switch (data_id) {
                    case ROBOT_CMD_DATA_RECEIVE_ID: {
                        memcpy(&RECEIVE_ROBOT_CMD_DATA, sof_address, sizeof(ReceiveDataRobotCmd_s));
                    } break;
                    case VIRTUAL_RC_DATA_RECEIVE_ID: {
                        memcpy(
                            &RECEIVE_VIRTUAL_RC_DATA, sof_address, sizeof(ReceiveDataVirtualRc_s));
                    } break;
                    default:
                        break;
                }
                if (*((uint32_t *)(&sof_address[4])) > LATEST_RX_TIMESTAMP) {
                    LATEST_RX_TIMESTAMP = *((uint32_t *)(&sof_address[4]));
                    RECEIVE_TIME = HAL_GetTick();
                }
            }
            sof_address += (data_len + HEADER_SIZE + 2);
        } else {  // CRC8 error: move to the next byte and continue searching.
            sof_address++;
        }
    }
    // 鏇存柊涓嬩竴娆℃帴鏀舵暟鎹殑璧峰浣嶇疆
    if (sof_address >= rx_buffer_limit) {
        // No remaining data: restart from the beginning of the USB RX buffer next time.
        rx_data_start_address = USB_RX_BUF;
    } else {
        uint16_t remaining_data_len = (uint16_t)(rx_buffer_limit - sof_address);
        // Preserve the unparsed tail and append new USB data after it next cycle.
        rx_data_start_address = USB_RX_BUF + remaining_data_len;
        memcpy(USB_RX_BUF, sof_address, remaining_data_len);
    }
}

/*******************************************************************************/
/* Send Function                                                               */
/*******************************************************************************/

/**
 * @brief 鍙戦€両MU鏁版嵁
 * @param duration 鍙戦€佸懆鏈? */
static void UsbSendImuData(void)
{
    if (IMU == NULL) {
        return;
    }

    SEND_DATA_IMU.time_stamp = HAL_GetTick();

    SEND_DATA_IMU.data.yaw = IMU->angle[AX_Z];
    SEND_DATA_IMU.data.pitch = IMU->angle[AX_Y];
    SEND_DATA_IMU.data.roll = IMU->angle[AX_X];

    SEND_DATA_IMU.data.yaw_vel = IMU->gyro[AX_Z];
    SEND_DATA_IMU.data.pitch_vel = IMU->gyro[AX_Y];
    SEND_DATA_IMU.data.roll_vel = IMU->gyro[AX_X];

    append_CRC16_check_sum((uint8_t *)&SEND_DATA_IMU, sizeof(SendDataImu_s));
    USB_Transmit((uint8_t *)&SEND_DATA_IMU, sizeof(SendDataImu_s));
}

/**
 * @brief 鍙戦€佹満鍣ㄤ汉杩愬姩鏁版嵁
 * @param duration 鍙戦€佸懆鏈? */
static void UsbSendRobotMotionData(void)
{
    if (FDB_SPEED_VECTOR == NULL) {
        return;
    }

    SEND_ROBOT_MOTION_DATA.time_stamp = HAL_GetTick();

    SEND_ROBOT_MOTION_DATA.data.speed_vector.vx = FDB_SPEED_VECTOR->vx;
    SEND_ROBOT_MOTION_DATA.data.speed_vector.vy = FDB_SPEED_VECTOR->vy;
    SEND_ROBOT_MOTION_DATA.data.speed_vector.wz = FDB_SPEED_VECTOR->wz;

    append_CRC16_check_sum((uint8_t *)&SEND_ROBOT_MOTION_DATA, sizeof(SendDataRobotMotion_s));
    USB_Transmit((uint8_t *)&SEND_ROBOT_MOTION_DATA, sizeof(SendDataRobotMotion_s));
}

/*******************************************************************************/
/* Receive Function                                                            */
/*******************************************************************************/

static void GetCmdData(void)
{
    ROBOT_CMD_DATA.speed_vector.vx = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vx;
    ROBOT_CMD_DATA.speed_vector.vy = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vy;
    ROBOT_CMD_DATA.speed_vector.wz = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.wz;

    ROBOT_CMD_DATA.chassis.yaw = RECEIVE_ROBOT_CMD_DATA.data.chassis.yaw;
    ROBOT_CMD_DATA.chassis.pitch = RECEIVE_ROBOT_CMD_DATA.data.chassis.pitch;
    ROBOT_CMD_DATA.chassis.roll = RECEIVE_ROBOT_CMD_DATA.data.chassis.roll;
    ROBOT_CMD_DATA.chassis.leg_length = RECEIVE_ROBOT_CMD_DATA.data.chassis.leg_lenth;

#if __USB_RC_SOURCE == USB_RC_SOURCE_ROBOT_CMD
    TranslateRobotCmdToRcCtrl(&VIRTUAL_RC_CTRL, &ROBOT_CMD_DATA);
    RefreshRcControlSource();
#endif
}

static void GetVirtualRcCtrlData(void)
{
#if __USB_RC_SOURCE == USB_RC_SOURCE_VIRTUAL_RC
    memcpy(&VIRTUAL_RC_CTRL, &RECEIVE_VIRTUAL_RC_DATA.data, sizeof(RC_ctrl_t));
    RefreshRcControlSource();
#endif
}

static float UsbClamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static int16_t UsbSpeedToRcChannel(float value, float max_abs_value)
{
    if (max_abs_value <= 0.0f) {
        return 0;
    }

    float normalized = UsbClamp(value / max_abs_value, -1.0f, 1.0f);
    return (int16_t)(normalized / RC_TO_ONE);
}

static void TranslateRobotCmdToRcCtrl(RC_ctrl_t * rc_ctrl, const RobotCmdData_t * robot_cmd)
{
    memset(rc_ctrl, 0, sizeof(RC_ctrl_t));

    // Translate 0x01 commands into the RC semantics that select normal balanced motion.
    rc_ctrl->rc.s[CHASSIS_MODE_CHANNEL] = RC_SW_MID;
    rc_ctrl->rc.s[CHASSIS_FUNCTION] = RC_SW_DOWN;

    rc_ctrl->rc.ch[CHASSIS_X_CHANNEL] =
        UsbSpeedToRcChannel(robot_cmd->speed_vector.vx, MAX_SPEED_VECTOR_VX);
    rc_ctrl->rc.ch[CHASSIS_WZ_CHANNEL] =
        UsbSpeedToRcChannel(-robot_cmd->speed_vector.wz, MAX_SPEED_VECTOR_WZ);

    // In the current chassis RC framework, BIPEDAL mode only consumes tail on ch[1].
    // Keep it neutral so a 0x01 packet does not introduce an unintended tail command.
    rc_ctrl->rc.ch[CHASSIS_TAIL_POS_CHANNEL] = 0;
}

/*******************************************************************************/
/* Public Function                                                             */
/*******************************************************************************/

bool GetUsbOffline(void) { return USB_OFFLINE; }

bool GetUsbRcOffline(void) { return GetUsbOffline(); }

const RC_ctrl_t * GetUsbVirtualRcCtrl(void) { return &VIRTUAL_RC_CTRL; }

/**
 * @brief 鑾峰彇涓婁綅鏈烘帶鍒舵寚浠わ細搴曠洏鍧愭爣绯讳笅axis鏂瑰悜杩愬姩绾块€熷害
 * @param axis 杞磇d锛屽彲閰嶅悎瀹氫箟濂界殑杞磇d瀹忎娇鐢? * @return float (m/s) 搴曠洏鍧愭爣绯讳笅axis鏂瑰悜杩愬姩绾块€熷害
 */
inline float GetScCmdChassisSpeed(uint8_t axis)
{
    if (axis == AX_X)
    {
        return ROBOT_CMD_DATA.speed_vector.vx;
    } 
    else if (axis == AX_Y) 
    {
        return ROBOT_CMD_DATA.speed_vector.vy;
    }
    else if (axis == AX_Z)
    {
        return 0;
    }
    return 0.0f;
}

/**
 * @brief 鑾峰彇涓婁綅鏈烘帶鍒舵寚浠わ細搴曠洏鍧愭爣绯讳笅axis鏂瑰悜杩愬姩瑙掗€熷害
 * @param axis 杞磇d锛屽彲閰嶅悎瀹氫箟濂界殑杞磇d瀹忎娇鐢? * @return float (rad/s) 搴曠洏鍧愭爣绯讳笅axis鏂瑰悜杩愬姩瑙掗€熷害
 */
inline float GetScCmdChassisVelocity(uint8_t axis)
{
    if (axis == AX_Z)
    {
        return ROBOT_CMD_DATA.speed_vector.wz;
    } 
    return 0.0f;
}


/**
 * @brief 鑾峰彇涓婁綅鏈烘帶鍒舵寚浠わ細搴曠洏绂诲湴楂樺害锛屽钩琛″簳鐩樹腑鍙敤浣滆吙闀垮弬鏁? * @param void
 * @return (m) 搴曠洏绂诲湴楂樺害
 */
inline float GetScCmdChassisHeight(void)
{
    return ROBOT_CMD_DATA.chassis.leg_length;
}
/*------------------------------ End of File ------------------------------*/

