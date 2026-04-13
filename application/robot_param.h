/**
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  * @file       robot_param.h
  * @brief      Robot parameter entry header.
  ****************************(C) COPYRIGHT 2024 Polarbear****************************
  */

#ifndef ROBOT_PARAM_H
#define ROBOT_PARAM_H

#include "robot_typedef.h"
#include "struct_typedef.h"

// Include the concrete robot parameter configuration.
#include "robot_param_balanced_infantry.h"

// Select robot feature switches.
#define __RC_TYPE RC_HT8A
#define __DEVELOP 0
#define __DEBUG 0
#define __TUNING 0
#define __MUSIC_ON 0
#define __TUNING_MODE TUNING_NONE
#define __HEAT_IMU 1
#define __IMU_CONTROL_TEMPERATURE 35

#define __BOARD_INSTALL_SPIN_MATRIX \
    {0.0f, 1.0f, 0.0f},            \
    {-1.0f, 0.0f, 0.0f},           \
    {0.0f, 0.0f, 1.0f}

// USB communication options.
#define __USB_SEND_DEBUG 1

// Board id.
#ifndef __SELF_BOARD_ID
#define __SELF_BOARD_ID C_BOARD_DEFAULT
#endif

// Control-link selection.
#ifndef __CONTROL_LINK_RC
#define __CONTROL_LINK_RC CL_RC_DIRECT
#endif

#ifndef __CONTROL_LINK_KM
#define __CONTROL_LINK_KM CL_KM_RC
#endif

#ifndef __CONTROL_LINK_PS2
#define __CONTROL_LINK_PS2 CL_PS2_NONE
#endif

#endif /* ROBOT_PARAM_H */
