/**
  ****************************(C) COPYRIGHT 2025 Polarbear****************************
  * @file       remote_control.h
  * @brief      Remote-control interface definitions.
  ****************************(C) COPYRIGHT 2025 Polarbear****************************
  */

#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdbool.h>

#include "attribute_typedef.h"
#include "bsp_rc.h"
#include "macro_typedef.h"
#include "struct_typedef.h"

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u
#define SBUS_RC_FRAME_LENGTH 25u

// DT7 remote control channel range.
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

// AT9S PRO remote control channel range.
#define AT9S_PRO_RC_CH_VALUE_MIN ((uint16_t)200)
#define AT9S_PRO_RC_CH_VALUE_OFFSET ((uint16_t)1000)
#define AT9S_PRO_RC_CH_VALUE_MAX ((uint16_t)1800)
#define AT9S_PRO_RC_CONNECTED_FLAG ((uint8_t)12)

// HT8A remote control channel range.
#define HT8A_RC_CH013_VALUE_MIN ((uint16_t)432)
#define HT8A_RC_CH013_VALUE_OFFSET ((uint16_t)992)
#define HT8A_RC_CH013_VALUE_MAX ((uint16_t)1552)

#define HT8A_RC_CH247_VALUE_MIN ((uint16_t)192)
#define HT8A_RC_CH247_VALUE_OFFSET ((uint16_t)992)
#define HT8A_RC_CH247_VALUE_MAX ((uint16_t)1792)

#define HT8A_RC_CH47_VALUE_MIN ((uint16_t)192)
#define HT8A_RC_CH47_VALUE_OFFSET ((uint16_t)992)
#define HT8A_RC_CH47_VALUE_MAX ((uint16_t)1792)

#define HT8A_RC_CONNECTED_FLAG ((uint8_t)12)

// ET08A remote control channel range.
#define ET08A_RC_CH_VALUE_MIN ((uint16_t)353)
#define ET08A_RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define ET08A_RC_CH_VALUE_MAX ((uint16_t)1694)
#define ET08A_RC_CONNECTED_FLAG ((uint8_t)0)

#define RC_TO_ONE 0.0015151515151515f  // 1 / 660

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)

#define RC_CH_LEFT_HORIZONTAL ((uint8_t)2)
#define RC_CH_LEFT_VERTICAL ((uint8_t)3)
#define RC_CH_RIGHT_HORIZONTAL ((uint8_t)0)
#define RC_CH_RIGHT_VERTICAL ((uint8_t)1)
#define RC_CH_LEFT_ROTATE ((uint8_t)4)

#define RC_SW_LEFT ((uint8_t)1)
#define RC_SW_RIGHT ((uint8_t)0)

/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_W ((uint8_t)0)
#define KEY_S ((uint8_t)1)
#define KEY_A ((uint8_t)2)
#define KEY_D ((uint8_t)3)
#define KEY_SHIFT ((uint8_t)4)
#define KEY_CTRL ((uint8_t)5)
#define KEY_Q ((uint8_t)6)
#define KEY_E ((uint8_t)7)
#define KEY_R ((uint8_t)8)
#define KEY_F ((uint8_t)9)
#define KEY_G ((uint8_t)10)
#define KEY_Z ((uint8_t)11)
#define KEY_X ((uint8_t)12)
#define KEY_C ((uint8_t)13)
#define KEY_V ((uint8_t)14)
#define KEY_B ((uint8_t)15)

#define KEY_LEFT ((uint8_t)0)
#define KEY_RIGHT ((uint8_t)1)

#define KEY_PRESSED_OFFSET_W ((uint16_t)1 << KEY_W)
#define KEY_PRESSED_OFFSET_S ((uint16_t)1 << KEY_S)
#define KEY_PRESSED_OFFSET_A ((uint16_t)1 << KEY_A)
#define KEY_PRESSED_OFFSET_D ((uint16_t)1 << KEY_D)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << KEY_SHIFT)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << KEY_CTRL)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)1 << KEY_Q)
#define KEY_PRESSED_OFFSET_E ((uint16_t)1 << KEY_E)
#define KEY_PRESSED_OFFSET_R ((uint16_t)1 << KEY_R)
#define KEY_PRESSED_OFFSET_F ((uint16_t)1 << KEY_F)
#define KEY_PRESSED_OFFSET_G ((uint16_t)1 << KEY_G)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)1 << KEY_Z)
#define KEY_PRESSED_OFFSET_X ((uint16_t)1 << KEY_X)
#define KEY_PRESSED_OFFSET_C ((uint16_t)1 << KEY_C)
#define KEY_PRESSED_OFFSET_V ((uint16_t)1 << KEY_V)
#define KEY_PRESSED_OFFSET_B ((uint16_t)1 << KEY_B)

/* ----------------------- Data Struct ------------------------------------- */
typedef struct __RC_ctrl
{
    struct __rc
    {
        int16_t ch[5];
        char s[2];
    } __packed__ rc;

    struct __mouse
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } __packed__ mouse;

    struct __key
    {
        uint16_t v;
    } __packed__ key;
} __packed__ RC_ctrl_t;

typedef struct
{
    uint16_t ch[16];
    uint8_t connect_flag;
} __packed__ Sbus_t;

/* ----------------------- Internal Data ----------------------------------- */
extern void remote_control_init(void);
extern const RC_ctrl_t * get_remote_control_point(void);
extern const Sbus_t * get_sbus_point(void);
extern uint8_t RC_data_is_error(void);
extern void slove_RC_lost(void);
extern void slove_data_error(void);
extern void sbus_to_usart1(uint8_t * sbus);
extern void RefreshRcControlSource(void);

/******************************************************************/
/* API                                                            */
/******************************************************************/

extern inline bool GetRcOffline(void);

extern inline float GetDt7RcCh(uint8_t ch);
extern inline char GetDt7RcSw(uint8_t sw);
extern inline float GetDt7MouseSpeed(uint8_t axis);
extern inline bool GetDt7Mouse(uint8_t key);
extern inline bool GetDt7Keyboard(uint8_t key);

#endif
/*------------------------------ End of File ------------------------------*/
