#ifndef USB_TASK_H
#define USB_TASK_H

#include "robot_param.h"
#include "remote_control.h"
#include "stdbool.h"

extern void usb_task(void const * argument);
extern bool GetUsbOffline(void);
extern bool GetUsbRcOffline(void);
extern const RC_ctrl_t * GetUsbVirtualRcCtrl(void);

#endif /* USB_TASK_H */
