/*
********************************************************************************
*                            中国自主物联网操作系统
*                            Thread Operating System
*
*                              主芯片:STM32F401re
*
********************************************************************************
*文件名     : device_gpio.c
*作用       : IO控制
********************************************************************************
*版本     作者            日期            说明
*V0.1    Guolz         2019/4/5        初始版本
********************************************************************************
*/
#include "device_gpio.h"
//////////////////////////////////////////////////////////////////////////////////	 


//初始化PB1为输出.并使能时钟	    
//LED IO初始化
void Gpio_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    
    EnableCtrl();
    
    /* Configure the GPIO_CTRL pin */
    GPIO_InitStruct.Pin   = CtrlPin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    
    HAL_GPIO_Init(CtrlPORT, &GPIO_InitStruct);
   
    SETA_CLR();
    SETB_CLR();
    
    CtrPowerOff(); 
    OledPowerOn();
}
