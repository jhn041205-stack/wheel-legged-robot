/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       oled_task.c/h
  * @brief      OLED show error value.oled屏幕显示错误码
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "oled_task.h"

#include "cmsis_os.h"
#include "chassis_balance.h"
#include "detect_task.h"
#include "IMU.h"
#include "main.h"
#include "oled.h"
#include "voltage_task.h"

#define OLED_CONTROL_TIME 10
#define REFRESH_RATE 10
#ifndef OLED_DEBUG_LQR_PAGE
#define OLED_DEBUG_LQR_PAGE 1
#endif
#ifndef OLED_LQR_PAGE_HOLD_REFRESHES
#define OLED_LQR_PAGE_HOLD_REFRESHES 20
#endif

const error_t * error_list_local;

uint8_t other_toe_name[4][4] = {"GYR\0","ACC\0","MAG\0"};

uint8_t last_oled_error = 0;
uint8_t now_oled_errror = 0;
static uint8_t refresh_tick = 0;
static uint8_t lqr_page_index = 0;
static uint8_t lqr_page_refresh_count = 0;

#if OLED_DEBUG_LQR_PAGE
static void oled_split_centi(float value, char *sign, uint16_t *integer, uint16_t *decimal)
{
    int32_t centi;

    if (value > 999.99f) {
        value = 999.99f;
    } else if (value < -999.99f) {
        value = -999.99f;
    }

    if (value < 0.0f) {
        *sign = '-';
        centi = (int32_t)(-value * 100.0f + 0.5f);
    } else {
        *sign = '+';
        centi = (int32_t)(value * 100.0f + 0.5f);
    }

    *integer = (uint16_t)(centi / 100);
    *decimal = (uint16_t)(centi % 100);
}

static void oled_print_pair(uint8_t col, uint8_t row, const char *left_label, float left_value,
                            const char *right_label, float right_value)
{
    char left_sign, right_sign;
    uint16_t left_integer, left_decimal;
    uint16_t right_integer, right_decimal;

    oled_split_centi(left_value, &left_sign, &left_integer, &left_decimal);
    oled_split_centi(right_value, &right_sign, &right_integer, &right_decimal);

    OLED_printf(col, row, "%s%c%u.%02u %s%c%u.%02u",
                left_label, left_sign, (unsigned int)left_integer, (unsigned int)left_decimal,
                right_label, right_sign, (unsigned int)right_integer, (unsigned int)right_decimal);
}

static const char * oled_get_mode_name(ChassisMode_e mode)
{
    switch (mode) {
        case CHASSIS_NOTAIL:
            return "NT";
        case CHASSIS_BIPEDAL:
            return "BP";
        case CHASSIS_TRIPOD:
            return "TP";
        case CHASSIS_JOINED:
            return "JN";
        case CHASSIS_STAND_UP:
            return "SU";
        case CHASSIS_SAFE:
            return "SF";
        case CHASSIS_OFF:
        default:
            return "OF";
    }
}

static uint8_t oled_get_lqr_page_count(ChassisMode_e mode)
{
    switch (mode) {
        case CHASSIS_BIPEDAL:
        case CHASSIS_TRIPOD:
            return 2;
        case CHASSIS_NOTAIL:
        case CHASSIS_JOINED:
        case CHASSIS_STAND_UP:
        case CHASSIS_SAFE:
        case CHASSIS_OFF:
        default:
            return 2;
    }
}

static void oled_show_lqr_debug(void)
{
    const Chassis_s * chassis = ChassisGetData();
    uint8_t page_count = oled_get_lqr_page_count(chassis->mode);
    uint8_t page = lqr_page_index % page_count;

    OLED_printf(0, 0, "LQR %s %u/%u", oled_get_mode_name(chassis->mode),
                (unsigned int)(page + 1), (unsigned int)page_count);

    if (page == 0) {
        oled_print_pair(0, 12, "X",  chassis->fdb.body.x,          "DX", chassis->fdb.body.x_dot_obv);
        oled_print_pair(0, 24, "YA", chassis->fdb.body.yaw,        "DY", chassis->fdb.body.yaw_dot);
        oled_print_pair(0, 36, "TL", chassis->fdb.leg_state[0].theta, "DL", chassis->fdb.leg_state[0].theta_dot);
        oled_print_pair(0, 48, "TR", chassis->fdb.leg_state[1].theta, "DR", chassis->fdb.leg_state[1].theta_dot);
    } else {
        oled_print_pair(0, 12, "PH", chassis->fdb.body.phi,        "DP", chassis->fdb.body.phi_dot);

        if (chassis->mode == CHASSIS_BIPEDAL || chassis->mode == CHASSIS_TRIPOD) {
            oled_print_pair(0, 24, "BT", chassis->fdb.tail_state.beta, "DB", chassis->fdb.tail_state.beta_dot);
        } else {
            OLED_printf(0, 24, "BT N/A DB N/A");
        }
    }
}

static void oled_update_lqr_page(void)
{
    const Chassis_s * chassis = ChassisGetData();
    uint8_t page_count = oled_get_lqr_page_count(chassis->mode);

    lqr_page_refresh_count++;
    if (lqr_page_refresh_count >= OLED_LQR_PAGE_HOLD_REFRESHES) {
        lqr_page_refresh_count = 0;
        lqr_page_index++;
        if (lqr_page_index >= page_count) {
            lqr_page_index = 0;
        }
    } else if (lqr_page_index >= page_count) {
        lqr_page_index = 0;
    }
}
#endif

/**
  * @brief          oled task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          oled任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void oled_task(void const * argument)
{
    uint8_t i;
#if !OLED_DEBUG_LQR_PAGE
    uint8_t show_col, show_row;
#endif
    error_list_local = get_error_list_point();
    osDelay(1000);
    OLED_init();
    OLED_LOGO();
    i = 100;
    while (i--) {
        if (OLED_check_ack()) {
            detect_hook(OLED_TOE);
        }
        osDelay(10);
    }
    while (1) {
        //use i2c ack to check the oled
        if (OLED_check_ack()) {
            detect_hook(OLED_TOE);
        }

        now_oled_errror = toe_is_error(OLED_TOE);
        //oled init
        if (last_oled_error == 1 && now_oled_errror == 0) {
            OLED_init();
        }

        if (now_oled_errror == 0) {
            refresh_tick++;
            //10Hz refresh
            if (refresh_tick > configTICK_RATE_HZ / (OLED_CONTROL_TIME * REFRESH_RATE)) {
                refresh_tick = 0;
                OLED_operate_gram(PEN_CLEAR);
#if OLED_DEBUG_LQR_PAGE
                oled_update_lqr_page();
                oled_show_lqr_debug();
#else
                OLED_show_graphic(0, 1, &battery_box);

                if (get_battery_percentage() < 10) {
                    OLED_printf(9, 2, "%d", get_battery_percentage());
                } else if (get_battery_percentage() < 100) {
                    OLED_printf(6, 2, "%d", get_battery_percentage());
                } else {
                    OLED_printf(3, 2, "%d", get_battery_percentage());
                }

                OLED_show_string(90, 27, "DBUS");
                OLED_show_graphic(115, 27, &check_box[error_list_local[DBUS_TOE].error_exist]);
                for (i = CHASSIS_MOTOR1_TOE; i < CHASSIS_MOTOR4_TOE + 1; i++) {
                    show_col = ((i - 1) * 32) % 128;
                    show_row = 15 + (i - 1) / 4 * 12;
                    OLED_show_char(show_col, show_row, 'M');
                    OLED_show_char(show_col + 6, show_row, '0' + i);
                    OLED_show_graphic(
                        show_col + 12, show_row, &check_box[error_list_local[i].error_exist]);
                }

                for(i = BOARD_GYRO_TOE; i < BOARD_MAG_TOE + 1; i++)
                {
                    // show_col = (i * 32) % 128;
                    show_col = (i - BOARD_GYRO_TOE) * 32;   // -> 0, 32, 64

                    show_row = 15 + i / 4 * 12;
                    OLED_show_string(show_col, show_row, other_toe_name[i - BOARD_GYRO_TOE]);
                    OLED_show_graphic(show_col + 18, show_row, &check_box[error_list_local[i].error_exist]);

                }
#endif
                OLED_refresh_gram();
            }
        }

        last_oled_error = now_oled_errror;
        osDelay(OLED_CONTROL_TIME);
    }
}
