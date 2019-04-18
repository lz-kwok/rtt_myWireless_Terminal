/*
********************************************************************************
*                            中国自主物联网操作系统
*                            Thread Operating System
*
*                              主芯片:STM32F401re
*
********************************************************************************
*文件名     : device_usb.h
*作用       : USB设备
********************************************************************************
*版本     作者            日期            说明
*V0.1    Guolz         2016/12/14        初始版本
********************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEVICE_USB__H
#define __DEVICE_USB__H



/* Includes ------------------------------------------------------------------*/
#include "system.h"
#include "usbd_def.h"


extern USBD_HandleTypeDef hUsbDeviceFS;


void USB_Vcp_Init(void);


#endif /* __DEVICEUART__H */
