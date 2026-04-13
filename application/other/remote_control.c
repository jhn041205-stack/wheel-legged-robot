/**
  ****************************(C) COPYRIGHT 2025 Polarbear****************************
  * @file       remote_control.c/h
  * @brief      閬ユ帶鍣ㄥ鐞嗭紝閬ユ帶鍣ㄦ槸閫氳繃绫讳技SBUS鐨勫崗璁紶杈擄紝鍒╃敤DMA浼犺緭鏂瑰紡鑺傜害CPU
  *             璧勬簮锛屽埄鐢ㄤ覆鍙ｇ┖闂蹭腑鏂潵鎷夎捣澶勭悊鍑芥暟锛屽悓鏃舵彁渚涗竴浜涙帀绾块噸鍚疍MA锛屼覆鍙?  *             鐨勬柟寮忎繚璇佺儹鎻掓嫈鐨勭ǔ瀹氭€с€?  * @note       璇ヤ换鍔℃槸閫氳繃涓插彛涓柇鍚姩锛屼笉鏄痜reeRTOS浠诲姟
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *  V2.0.0     Feb-17-2025     Penguin         1. support RC AT9S PRO
  *                                             2. support RC HT8A
  *                                             3. support normal sbus RC in struct Sbus_t
  *  V2.0.1     Feb-25-2025     Penguin         1. support RC ET08A
  *
  @verbatim
  ==============================================================================
  浣跨敤At9sPro閬ユ帶鍣ㄦ椂璇疯缃?閫氫负SwE锛?閫氫负SwG

  娉細浣跨敤闈濪T7閬ユ帶鍣ㄦ椂锛岄渶瑕佸厛妫€鏌ラ€氶亾鍊兼暟鎹槸鍚︽甯革紙涓€鑸仴鎺у櫒閮藉甫鏈夐€氶亾鍊兼暟鎹亸绉诲姛鑳斤紝灏嗛€氶亾鍊间腑鍊肩Щ鍔ㄥ埌姝ｇ‘鏁板€煎悗鍐嶄娇鐢級
      AT9S PRO 閬ユ帶鍣ㄤ腑鍊间负 1000
      HT8A 閬ユ帶鍣ㄤ腑鍊间负 992
      ET08A 閬ユ帶鍣ㄤ腑鍊间负 1024
  
  ET08A 閬ユ帶鍣ㄨ缃寚鍗楋細
    1. 璁剧疆 涓昏彍鍗?>绯荤粺璁剧疆->鎽囨潌妯″紡 涓烘ā寮?
    2. 璁剧疆 涓昏彍鍗?>閫氱敤鍔熻兘->閫氶亾璁剧疆 5閫氶亾涓?[杈呭姪1 SB --] 6閫氶亾涓?[杈呭姪2 SC --]
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Polarbear****************************
  */

// clang-format off
#include "remote_control.h"

#include "main.h"

#include "bsp_usart.h"
#include "string.h"

#include "detect_task.h"
#include "robot_param.h"
#include "communication.h"
#include "usb_task.h"

// Remote-control offline timeout.
#define RC_LOST_TIME 100  // ms
// Consecutive lost SBUS frames before judging disconnect for non-DT7 remotes.
#define SBUS_MAX_LOST_NUN 10
// Remote-control channel error threshold.
#define RC_CHANNAL_ERROR_VALUE 700

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

static int16_t RC_abs(int16_t value);
static void RefreshActiveRcCtrl(void);
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

#if (__RC_TYPE == RC_AT9S_PRO)
static void At9sProSbusToRc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
#elif (__RC_TYPE == RC_HT8A)
static void Ht8aSbusToRc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
#elif (__RC_TYPE == RC_ET08A)
static void Et08aSbusToRc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
#endif

// Active control source seen by the chassis.
// Raw physical remote data is stored separately for USB authorization arbitration.
RC_ctrl_t rc_ctrl;
static RC_ctrl_t raw_rc_ctrl;
Sbus_t sbus = {.connect_flag = 0xFF};

//鎺ユ敹鍘熷鏁版嵁锛屼负18涓瓧鑺傦紝缁欎簡36涓瓧鑺傞暱搴︼紝闃叉DMA浼犺緭瓒婄晫
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

// 涓婁竴娆℃帴鏀舵暟鎹殑鏃堕棿
static uint32_t last_receive_time = 0;
// Count of consecutive valid receive events.
static uint32_t receive_count = 0;
// Lost-count tracking for non-DT7 SBUS remotes.
static uint32_t sbus_lost_count = SBUS_MAX_LOST_NUN + 5;

#if (__RC_TYPE != RC_DT7)
static uint8_t connected_flag;  // 閬ユ帶鍣ㄨ繛鎺ユ爣蹇椾綅
#endif  // __RC_TYPE != RC_DT7

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          閬ユ帶鍣ㄥ垵濮嬪寲
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
    memset(&rc_ctrl, 0, sizeof(rc_ctrl));
    memset(&raw_rc_ctrl, 0, sizeof(raw_rc_ctrl));
#if (__RC_TYPE == RC_AT9S_PRO)
    connected_flag = AT9S_PRO_RC_CONNECTED_FLAG;
#elif (__RC_TYPE == RC_HT8A)
    connected_flag = HT8A_RC_CONNECTED_FLAG;
#elif (__RC_TYPE == RC_ET08A)
    connected_flag = ET08A_RC_CONNECTED_FLAG;
#endif
}
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          鑾峰彇閬ユ帶鍣ㄦ暟鎹寚閽?  * @param[in]      none
  * @retval         閬ユ帶鍣ㄦ暟鎹寚閽?  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

void RefreshRcControlSource(void) { RefreshActiveRcCtrl(); }

/**
  * @brief          鑾峰彇SBUS閬ユ帶鍣ㄦ暟鎹寚閽?  * @param[in]      none
  * @retval         SBUS閬ユ帶鍣ㄦ暟鎹寚閽?  */
const Sbus_t *get_sbus_point(void)
{
    return &sbus;
}

static void RefreshActiveRcCtrl(void)
{
    bool usb_authorized =
        (raw_rc_ctrl.rc.s[CHASSIS_MODE_CHANNEL] == RC_SW_DOWN) && !GetUsbOffline();

    if (usb_authorized) {
        memcpy(&rc_ctrl, GetUsbVirtualRcCtrl(), sizeof(RC_ctrl_t));
    } else {
        memcpy(&rc_ctrl, &raw_rc_ctrl, sizeof(RC_ctrl_t));
    }
}

//鍒ゆ柇閬ユ帶鍣ㄦ暟鎹槸鍚﹀嚭閿欙紝
uint8_t RC_data_is_error(void)
{
    // Validate the physical remote data before it participates in source arbitration.
    if (RC_abs(raw_rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(raw_rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(raw_rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(raw_rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (raw_rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if (raw_rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }
    return 0;

error:
    raw_rc_ctrl.rc.ch[0] = 0;
    raw_rc_ctrl.rc.ch[1] = 0;
    raw_rc_ctrl.rc.ch[2] = 0;
    raw_rc_ctrl.rc.ch[3] = 0;
    raw_rc_ctrl.rc.ch[4] = 0;
    raw_rc_ctrl.rc.s[0] = RC_SW_DOWN;
    raw_rc_ctrl.rc.s[1] = RC_SW_DOWN;
    raw_rc_ctrl.mouse.x = 0;
    raw_rc_ctrl.mouse.y = 0;
    raw_rc_ctrl.mouse.z = 0;
    raw_rc_ctrl.mouse.press_l = 0;
    raw_rc_ctrl.mouse.press_r = 0;
    raw_rc_ctrl.key.v = 0;
    RefreshActiveRcCtrl();
    return 1;
}

void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}
void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}

// clang-format on
// Record consecutive valid receive events.
#define COUNT_RECEIVED                            \
    if (now - last_receive_time > RC_LOST_TIME) { \
        receive_count = 0;                        \
    }                                             \
    receive_count++;
// clang-format off

//涓插彛涓柇
void USART3_IRQHandler(void)
{
    if (huart3.Instance->SR & UART_FLAG_RXNE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        uint32_t now = HAL_GetTick();

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //澶辨晥DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //鑾峰彇鎺ユ敹鏁版嵁闀垮害,闀垮害 = 璁惧畾闀垮害 - 鍓╀綑闀垮害
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //閲嶆柊璁惧畾鏁版嵁闀垮害
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //璁惧畾缂撳啿鍖?
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //浣胯兘DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //澶勭悊閬ユ帶鍣ㄦ暟鎹?
                sbus_to_rc(sbus_rx_buf[0], &raw_rc_ctrl);
                RefreshActiveRcCtrl();
                
                COUNT_RECEIVED
                
                //璁板綍鏁版嵁鎺ユ敹鏃堕棿
                last_receive_time = HAL_GetTick();
                detect_hook(DBUS_TOE);
                sbus_to_usart1(sbus_rx_buf[0]);
            } 
            else if (this_time_rx_len == SBUS_RC_FRAME_LENGTH)
            {
                //澶勭悊閬ユ帶鍣ㄦ暟鎹?
#if (__RC_TYPE == RC_AT9S_PRO)
                At9sProSbusToRc(sbus_rx_buf[0], &raw_rc_ctrl);
#elif (__RC_TYPE == RC_HT8A)
                Ht8aSbusToRc(sbus_rx_buf[0], &raw_rc_ctrl);
#elif (__RC_TYPE == RC_ET08A)
                Et08aSbusToRc(sbus_rx_buf[0], &raw_rc_ctrl);
#endif
                RefreshActiveRcCtrl();
                
                COUNT_RECEIVED
                
                //璁板綍鏁版嵁鎺ユ敹鏃堕棿
                last_receive_time = HAL_GetTick();
                detect_hook(DBUS_TOE);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //澶辨晥DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //鑾峰彇鎺ユ敹鏁版嵁闀垮害,闀垮害 = 璁惧畾闀垮害 - 鍓╀綑闀垮害
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //閲嶆柊璁惧畾鏁版嵁闀垮害
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //璁惧畾缂撳啿鍖?
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //浣胯兘DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //澶勭悊閬ユ帶鍣ㄦ暟鎹?
                sbus_to_rc(sbus_rx_buf[1], &raw_rc_ctrl);
                RefreshActiveRcCtrl();
                
                COUNT_RECEIVED

                //璁板綍鏁版嵁鎺ユ敹鏃堕棿
                last_receive_time = HAL_GetTick();
                detect_hook(DBUS_TOE);
                sbus_to_usart1(sbus_rx_buf[1]);
            }
            else if (this_time_rx_len == SBUS_RC_FRAME_LENGTH)
            {
                //澶勭悊閬ユ帶鍣ㄦ暟鎹?
#if (__RC_TYPE == RC_AT9S_PRO)
                At9sProSbusToRc(sbus_rx_buf[1], &raw_rc_ctrl);
#elif (__RC_TYPE == RC_HT8A)
                Ht8aSbusToRc(sbus_rx_buf[1], &raw_rc_ctrl);
#elif (__RC_TYPE == RC_ET08A)
                Et08aSbusToRc(sbus_rx_buf[1], &raw_rc_ctrl);
#endif
                RefreshActiveRcCtrl();
                
                COUNT_RECEIVED

                //璁板綍鏁版嵁鎺ユ敹鏃堕棿
                last_receive_time = HAL_GetTick();
                detect_hook(DBUS_TOE);
            }
        }
    }

}

//鍙栨鍑芥暟
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          閬ユ帶鍣ㄥ崗璁В鏋?  * @param[in]      sbus_buf: 鍘熺敓鏁版嵁鎸囬拡
  * @param[out]     rc_ctrl: 閬ユ帶鍣ㄦ暟鎹寚
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}


#define SBUS_DECODE()                                                                 \
    sbus.ch[0] =((sbus_buf[2]<<8)   + (sbus_buf[1])) & 0x07ff;                        \
    sbus.ch[1] =((sbus_buf[3]<<5)   + (sbus_buf[2]>>3)) & 0x07ff;                     \
    sbus.ch[2] =((sbus_buf[5]<<10)  + (sbus_buf[4]<<2) + (sbus_buf[3]>>6)) & 0x07ff;  \
    sbus.ch[3] =((sbus_buf[6]<<7)   + (sbus_buf[5]>>1)) & 0x07ff;                     \
    sbus.ch[4] =((sbus_buf[7]<<4)   + (sbus_buf[6]>>4)) & 0x07ff;                     \
    sbus.ch[5] =((sbus_buf[9]<<9)   + (sbus_buf[8]<<1) + (sbus_buf[7]>>7)) & 0x07ff;  \
    sbus.ch[6] =((sbus_buf[10]<<6)  + (sbus_buf[9]>>2)) & 0x07ff;                     \
    sbus.ch[7] =((sbus_buf[11]<<3)  + (sbus_buf[10]>>5)) & 0x07ff;                    \
    sbus.ch[8] =((sbus_buf[13]<<8)  + (sbus_buf[12])) & 0x07ff;                       \
    sbus.ch[9] =((sbus_buf[14]<<5)  + (sbus_buf[13]>>3)) & 0x07ff;                    \
    sbus.ch[10]=((sbus_buf[16]<<10) + (sbus_buf[15]<<2) + (sbus_buf[14]>>6)) & 0x07ff;\
    sbus.ch[11]=((sbus_buf[17]<<7)  + (sbus_buf[16]>>1)) & 0x07ff;                    \
    sbus.ch[12]=((sbus_buf[18]<<4)  + (sbus_buf[17]>>4)) & 0x07ff;                    \
    sbus.ch[13]=((sbus_buf[20]<<9)  + (sbus_buf[19]<<1) + (sbus_buf[18]>>7)) & 0x07ff;\
    sbus.ch[14]=((sbus_buf[21]<<6)  + (sbus_buf[20]>>2)) & 0x07ff;                    \
    sbus.ch[15]=((sbus_buf[22]<<3)  + (sbus_buf[21]>>5)) & 0x07ff;                    \
    sbus.connect_flag = sbus_buf[23];

#define SBUS_LOST_CHECK()                      \
    if (sbus.connect_flag == connected_flag) { \
        sbus_lost_count = 0;                   \
    } else {                                   \
        sbus_lost_count++;                     \
    }

// DT7閬ユ帶鍣ㄧ壒娈婇€氶亾缃浂
#define SPECIAL_CHANNEL_SET_SERO()\
    rc_ctrl->mouse.x = 0;         \
    rc_ctrl->mouse.y = 0;         \
    rc_ctrl->mouse.z = 0;         \
    rc_ctrl->mouse.press_l = 0;   \
    rc_ctrl->mouse.press_r = 0;   \
    rc_ctrl->key.v = 0;           \
    rc_ctrl->rc.ch[4] = 0;        \

#if (__RC_TYPE == RC_AT9S_PRO)

/**
  * @brief          AT9S PRO 閬ユ帶鍣ㄥ崗璁В鏋?  * @param[in]      sbus_buf: 鍘熺敓鏁版嵁鎸囬拡
  * @param[out]     rc_ctrl: 閬ユ帶鍣ㄦ暟鎹寚閽?  * @retval         none
  */
static void At9sProSbusToRc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    // SBUS閫氶亾瑙ｆ瀽
    SBUS_DECODE()
    SBUS_LOST_CHECK()

    // 灏哠BUS閫氶亾鏁版嵁杞崲涓篋T7閬ユ帶鍣ㄦ暟鎹紝鏂逛究鍏煎浣跨敤
    rc_ctrl->rc.ch[0] =  (sbus.ch[0] - AT9S_PRO_RC_CH_VALUE_OFFSET) / 800.0f * 660;
    rc_ctrl->rc.ch[1] = -(sbus.ch[1] - AT9S_PRO_RC_CH_VALUE_OFFSET) / 800.0f * 660;
    rc_ctrl->rc.ch[2] =  (sbus.ch[3] - AT9S_PRO_RC_CH_VALUE_OFFSET) / 800.0f * 660;
    rc_ctrl->rc.ch[3] =  (sbus.ch[2] - AT9S_PRO_RC_CH_VALUE_OFFSET) / 800.0f * 660;

    static char sw_mapping[3] = {RC_SW_UP, RC_SW_MID, RC_SW_DOWN};
    rc_ctrl->rc.s[0] = sw_mapping[(sbus.ch[5] - AT9S_PRO_RC_CH_VALUE_OFFSET) / 800 + 1];
    rc_ctrl->rc.s[1] = sw_mapping[(sbus.ch[4] - AT9S_PRO_RC_CH_VALUE_OFFSET) / 800 + 1];

    // AT9S PRO 閬ユ帶鍣ㄦ病鏈夐紶鏍囧拰閿洏鏁版嵁
    SPECIAL_CHANNEL_SET_SERO()
}

#elif (__RC_TYPE == RC_HT8A)

/**
  * @brief          HT8A 閬ユ帶鍣ㄥ崗璁В鏋?  * @param[in]      sbus_buf: 鍘熺敓鏁版嵁鎸囬拡
  * @param[out]     rc_ctrl: 閬ユ帶鍣ㄦ暟鎹寚閽?  * @retval         none
  */
static void Ht8aSbusToRc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    // SBUS閫氶亾瑙ｆ瀽
    SBUS_DECODE()
    SBUS_LOST_CHECK()

    // 灏哠BUS閫氶亾鏁版嵁杞崲涓篋T7閬ユ帶鍣ㄦ暟鎹紝鏂逛究鍏煎浣跨敤
    rc_ctrl->rc.ch[0] =  (sbus.ch[0] - HT8A_RC_CH013_VALUE_OFFSET) / 560.0f * 660;
    rc_ctrl->rc.ch[1] =  (sbus.ch[1] - HT8A_RC_CH013_VALUE_OFFSET) / 560.0f * 660;
    rc_ctrl->rc.ch[2] =  (sbus.ch[3] - HT8A_RC_CH013_VALUE_OFFSET) / 560.0f * 660;
    rc_ctrl->rc.ch[3] =  (sbus.ch[2] - HT8A_RC_CH247_VALUE_OFFSET) / 800.0f * 660;

    static char sw_mapping[3] = {RC_SW_UP, RC_SW_MID, RC_SW_DOWN};
    rc_ctrl->rc.s[0] = sw_mapping[(sbus.ch[7] - HT8A_RC_CH247_VALUE_OFFSET) / 800 + 1];
    rc_ctrl->rc.s[1] = sw_mapping[(sbus.ch[4] - HT8A_RC_CH247_VALUE_OFFSET) / 800 + 1];

    // HT8A 閬ユ帶鍣ㄦ病鏈夐紶鏍囧拰閿洏鏁版嵁
    SPECIAL_CHANNEL_SET_SERO()
}

#elif (__RC_TYPE == RC_ET08A)

/**
  * @brief          ET08A 閬ユ帶鍣ㄥ崗璁В鏋?  * @param[in]      sbus_buf: 鍘熺敓鏁版嵁鎸囬拡
  * @param[out]     rc_ctrl: 閬ユ帶鍣ㄦ暟鎹寚閽?  * @retval         none
  */
static void Et08aSbusToRc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl){
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    // SBUS閫氶亾瑙ｆ瀽
    SBUS_DECODE()
    SBUS_LOST_CHECK()

    // 灏哠BUS閫氶亾鏁版嵁杞崲涓篋T7閬ユ帶鍣ㄦ暟鎹紝鏂逛究鍏煎浣跨敤
    rc_ctrl->rc.ch[0] =  (sbus.ch[0] - ET08A_RC_CH_VALUE_OFFSET) / 671.0f * 660;
    rc_ctrl->rc.ch[1] = -(sbus.ch[1] - ET08A_RC_CH_VALUE_OFFSET) / 671.0f * 660;
    rc_ctrl->rc.ch[2] =  (sbus.ch[3] - ET08A_RC_CH_VALUE_OFFSET) / 671.0f * 660;
    rc_ctrl->rc.ch[3] =  (sbus.ch[2] - ET08A_RC_CH_VALUE_OFFSET) / 671.0f * 660;

    static char sw_mapping[3] = {RC_SW_UP, RC_SW_MID, RC_SW_DOWN};
    rc_ctrl->rc.s[0] = sw_mapping[(sbus.ch[5] - ET08A_RC_CH_VALUE_OFFSET) / 670 + 1];
    rc_ctrl->rc.s[1] = sw_mapping[(sbus.ch[4] - ET08A_RC_CH_VALUE_OFFSET) / 670 + 1];

    // ET08A 閬ユ帶鍣ㄦ病鏈夐紶鏍囧拰閿洏鏁版嵁
    SPECIAL_CHANNEL_SET_SERO()
}

#endif

#undef SBUS_DECODE
#undef SPECIAL_CHANNEL_SET_SERO

/**
  * @brief          send sbus data by usart1, called in usart3_IRQHandle
  * @param[in]      sbus: sbus data, 18 bytes
  * @retval         none
  */
/**
  * @brief          閫氳繃usart1鍙戦€乻bus鏁版嵁,鍦╱sart3_IRQHandle璋冪敤
  * @param[in]      sbus: sbus鏁版嵁, 18瀛楄妭
  * @retval         none
  */
void sbus_to_usart1(uint8_t *sbus)
{
    static uint8_t usart_tx_buf[20];
    static uint8_t i =0;
    usart_tx_buf[0] = 0xA6;
    memcpy(usart_tx_buf + 1, sbus, 18);
    for(i = 0, usart_tx_buf[19] = 0; i < 19; i++)
    {
        usart_tx_buf[19] += usart_tx_buf[i];
    }
    usart1_tx_dma_enable(usart_tx_buf, 20);
}

// clang-format on

/******************************************************************/
/* API                                                            */
/*----------------------------------------------------------------*/
/* function:      GetRcOffline                                    */
/*                GetDt7RcCh                                      */
/*                GetDt7RcSw                                      */
/*                GetDt7MouseSpeed                                */
/*                GetDt7Mouse                                     */
/*                GetDt7Keyboard                                  */
/******************************************************************/

/**
  * @brief          鑾峰彇閬ユ帶鍣ㄦ槸鍚︾绾裤€?  * @retval         true:绂荤嚎锛宖alse:鍦ㄧ嚎
  */
inline bool GetRcOffline(void)
{
#if __RC_TYPE == RC_DT7
#define USE_SBUS_LOST_COUNT 0
#else
#define USE_SBUS_LOST_COUNT 0
#endif

#if __CONTROL_LINK_RC == CL_RC_DIRECT
    return !((receive_count > 5) && (HAL_GetTick() - last_receive_time < RC_LOST_TIME)) ||
           ((sbus_lost_count > SBUS_MAX_LOST_NUN) && USE_SBUS_LOST_COUNT);
#elif __CONTROL_LINK_RC == CL_RC_UART2
    return GetUartRcOffline();
#elif __CONTROL_LINK_RC == CL_RC_USB
    return GetUsbRcOffline();
#else
    return true;
#endif

#undef USE_SBUS_LOST_COUNT
}

/**
  * @brief          鑾峰彇DT7閬ユ帶鍣ㄩ€氶亾鍊笺€?  * @param[in]      ch 閫氶亾id锛?-鍙冲钩, 1-鍙崇珫, 2-宸﹀钩, 3-宸︾珫, 4-宸︽粴杞紝閰嶅悎ch id瀹忚繘琛屼娇鐢?  * @retval         DT7閬ユ帶鍣ㄩ€氶亾鍊硷紝鑼冨洿涓?[鈭?,1]
  */
inline float GetDt7RcCh(uint8_t ch) { return rc_ctrl.rc.ch[ch] * RC_TO_ONE; }
/**
  * @brief          鑾峰彇DT7閬ユ帶鍣ㄦ嫧鏉嗗€硷紝鍙厤鍚坰witch_is_xxx绯诲垪瀹忓嚱鏁颁娇鐢ㄣ€?  * @param[in]      sw 閫氶亾id锛?-鍙? 1-宸︼紝閰嶅悎sw id瀹忚繘琛屼娇鐢?  * @retval         DT7閬ユ帶鍣ㄦ嫧鏉嗗€硷紝鑼冨洿涓簕1,2,3}
  */
inline char GetDt7RcSw(uint8_t sw) { return rc_ctrl.rc.s[sw]; }
/**
  * @brief          鑾峰彇榧犳爣axis杞寸殑绉诲姩閫熷害
  * @param[in]      axis 杞磇d, 0-, 1-, 2-锛岄厤鍚堣酱id瀹忚繘琛屼娇鐢?  * @retval         榧犳爣axis杞寸Щ鍔ㄩ€熷害锛岃寖鍥翠负[,]
  */
inline float GetDt7MouseSpeed(uint8_t axis)
{
    switch (axis) {
        case AX_X:
            return rc_ctrl.mouse.x;
        case AX_Y:
            return rc_ctrl.mouse.y;
        case AX_Z:
            return rc_ctrl.mouse.z;
        default:
            return 0;
    }
}
/**
  * @brief          鑾峰彇榧犳爣鎸夐敭淇℃伅
  * @param[in]      key 鎸夐敭id锛岄厤鍚堟寜閿甶d瀹忚繘琛屼娇鐢?  * @retval         榧犳爣鎸夐敭鏄惁琚寜涓?  */
inline bool GetDt7Mouse(uint8_t key)
{
    switch (key) {
        case KEY_LEFT:
            return rc_ctrl.mouse.press_l;
        case KEY_RIGHT:
            return rc_ctrl.mouse.press_r;
        default:
            return 0;
    }
}
/**
  * @brief          鑾峰彇閿洏鎸夐敭淇℃伅
  * @param[in]      key 鎸夐敭id锛岄厤鍚堟寜閿甶d瀹忚繘琛屼娇鐢?  * @retval         閿洏鎸夐敭鏄惁琚寜涓?  */
inline bool GetDt7Keyboard(uint8_t key) { return rc_ctrl.key.v & ((uint16_t)1 << key); }
/*------------------------------ End of File ------------------------------*/


