/*
********************************************************************************
*                            中国自主物联网操作系统
*                            Thread Operating System
*
*                              主芯片:STM32F401re
*
********************************************************************************
*文件名     : systemglobal.c
*作用       : 系统全局变量
********************************************************************************
*版本     作者            日期            说明
*V0.1    Guolz         2019/4/5        初始版本
********************************************************************************
*/
#include "systemglobal.h"


/* Global variables（Uart） -------------------------------------------------*/
UART_HandleTypeDef Uart1Handle;
UART_HandleTypeDef Uart2Handle;
UART_HandleTypeDef Uart6Handle;
DMA_HandleTypeDef hdma_tx;
DMA_HandleTypeDef hdma_rx;
uint8_t rxBuff[20] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t rx1Buff[10];


/* Global variables（Scada） -------------------------------------------------*/
uint8_t CmdShow[25] = "HelloWorld";
uint8_t ModbusCmd1[8] = {0x0c,0x02,0x00,0x00,0x00,0x01,0xb8,0xd7};
uint8_t ModbusCmd2[8] = {0x0c,0x05,0x00,0x03,0xff,0x00,0x7d,0x27};
uint8_t ModbusCmd3[8] = {0x0c,0x05,0x00,0x03,0x00,0x00,0x3c,0xd7};
uint8_t ModbusCmd4[8] = {0x0c,0x06,0x00,0x00,0x00,0x01,0x49,0x17};
uint8_t ModbusCmd5[8] = {0x0c,0x03,0x00,0x00,0x00,0x01,0x85,0x17};
uint8_t ModbusCmd6[8] = {0x0c,0x06,0x00,0x05,0x00,0x01,0x59,0x16};
uint8_t ModbusCmd7[8] = {0x0c,0x03,0x00,0x05,0x00,0x01,0x95,0x16};



uint8_t PowerOnFlag = 0;
uint8_t Menuhierarchy = 0;
int8_t MenuChooseIndex = 0;
int8_t ParaChooseIndex = 0;
int8_t SysChooseIndex = 0;
uint8_t ParaMenuFlag = 0;
uint8_t ParaAskFlag = 0;
uint8_t SystemSetFlag = 0;
int8_t OKorCancel = 0;
uint8_t ParaSetConfirm = 0;
uint8_t DevIDConfirm = 0;
uint8_t SystemSetConfirm = 0;
uint8_t LanguageConfirm = 0;
uint8_t ChargingFlag = 0;
uint8_t SignalPower = 2;
uint8_t RecPower = 1;
uint8_t Uart1enth = 0;
uint8_t Ack_Flag = 0;
int8_t Shortcut  = 0;
uint32_t CodeSum = 0;
uint8_t TypeCodeTick = 0;
uint16_t AckValue = 0;
uint8_t RssiFlag = 0;
uint8_t NonAck = 0;
uint8_t Read433Para = 1;

int8_t DevIdChooseIndex = 1;
int8_t FreSetIndex = 1;
uint16_t TurnoffTick = 0;

int8_t LanSetIndex = 0;
int8_t DeviceNum = 0;
uint8_t FouthMenuIndex = 0;
int8_t ForceSetIndex = 0;
int16_t MovementSetIndex = 0;
int8_t InOutIndex = 1;
uint8_t InOutValue = 1;
int16_t DelaySetIndex = 0;
int16_t SpacingIndex = 0;
uint16_t OutNumIndex = 0;
int16_t LengthSetIndex = 150;
int8_t ABSetIndex = 1;
uint8_t ABValue = 1;
uint8_t DeviceIDConfirmTick[8] = {0,0,0,0,0,0,0,0};
uint8_t SysConfirmTick[8] = {0,0,0,0,0,0,0,0};
uint8_t ParaConfirmTick[8] = {0,0,0,0,0,0,0,0};
uint8_t TurnoffIndex = 1;
uint16_t TurnoffValue = 0;
uint8_t resetConfirm = 0;
uint8_t writeConfirm = 0;

const uint8_t BAT_RB5[] = 
{0x80,0x00,0x1F,0xBF,0xFF,0xDF,0x24,0x92,0x5F,0x24,0x92,0x5F,0x24,0x92,0x5F,0xBF,
0xFF,0xDF,0x80,0x00,0x1F};


const uint8_t BAT_RB4[] = 
{0x80,0x00,0x1F,0xBF,0xFF,0xDF,0x3C,0x92,0x5F,0x3C,0x92,0x5F,0x3C,0x92,0x5F,0xBF,
0xFF,0xDF,0x80,0x00,0x1F};


const uint8_t BAT_RB3[] = 
{0x80,0x00,0x1F,0xBF,0xFF,0xDF,0x3F,0x92,0x5F,0x3F,0x92,0x5F,0x3F,0x92,0x5F,0xBF,
0xFF,0xDF,0x80,0x00,0x1F};


const uint8_t BAT_RB2[] = 
{0x80,0x00,0x1F,0xBF,0xFF,0xDF,0x3F,0xF2,0x5F,0x3F,0xF2,0x5F,0x3F,0xF2,0x5F,0xBF,
0xFF,0xDF,0x80,0x00,0x1F};


const uint8_t BAT_RB1[] = 
{0x80,0x00,0x1F,0xBF,0xFF,0xDF,0x3F,0xFE,0x5F,0x3F,0xFE,0x5F,0x3F,0xFE,0x5F,0xBF,
0xFF,0xDF,0x80,0x00,0x1F};

const uint8_t BAT_RB0[] = 
{0x80,0x00,0x1F,0xBF,0xFF,0xDF,0x3F,0xFF,0xDF,0x3F,0xFF,0xDF,0x3F,0xFF,0xDF,0xBF,
0xFF,0xDF,0x80,0x00,0x1F};

const uint8_t Signal_2[] = 
{0x01,0xFF,0xBB,0xFF,0xD7,0xFF,0xEF,0xFF,0xEF,0xFF,0xEE,0x3F,0xEE,0x3F,0xEE,0x3F};

const uint8_t Signal_1[] = 
{0x01,0xBB,0xD7,0xEF,0xEF,0xEF,0xEF,0xEE};

const uint8_t Signal_0[] = 
{0x7E,0xBD,0xDB,0xE7,0xE7,0xDB,0xBD,0x7E};

const uint8_t BAT_B3[] =
{0x80,0x07,0xBF,0xF7,0x24,0x97,0x24,0x97,0x24,0x97,0xBF,0xF7,0x80,0x07};

const uint8_t BAT_B2[] = 
{0x80,0x07,0xBF,0xF7,0x3C,0x97,0x3C,0x97,0x3C,0x97,0xBF,0xF7,0x80,0x07};

const uint8_t BAT_B1[] = 
{0x80,0x07,0xBF,0xF7,0x3F,0x97,0x3F,0x97,0x3F,0x97,0xBF,0xF7,0x80,0x07};

const uint8_t BAT_B0[] = 
{0x80,0x07,0xBF,0xF7,0x3F,0xF7,0x3F,0xF7,0x3F,0xF7,0xBF,0xF7,0x80,0x07};

const uint8_t BAT_CHAR[] = 
{0x80,0x07,0xBE,0xF7,0x3D,0xF7,0x38,0x37,0x3F,0x77,0xBE,0xF7,0x80,0x07};

const uint8_t TypeCode[] = 
{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC,0xFD,
0xFF,0xFF,0xFF,0xFF,0x7F,0xFF,0xFF,0xFF,0xFD,0xF9,0xFF,0xF3,0xFF,0xFF,0xBF,0xF0,
0x00,0x3F,0xFC,0x32,0xFF,0xFD,0xFF,0xC0,0x40,0x7D,0xF7,0xBF,0xFB,0xF7,0x3F,0xFC,
0xFF,0xDF,0x75,0xFD,0xF7,0xBF,0xFA,0x40,0x0F,0xFC,0xFF,0xFD,0x6F,0xFD,0xF7,0xBF,
0xFA,0xBF,0xFF,0xFC,0xFF,0xFD,0x9D,0xF8,0x37,0xBF,0xF0,0x01,0xDF,0xFB,0x7F,0xED,
0x7E,0xFB,0x77,0x7F,0xFE,0xDB,0x5F,0xFB,0x7F,0xF8,0xF6,0xF3,0x70,0x0F,0xFE,0xC3,
0x5F,0xF7,0xBF,0xC4,0x0F,0xFB,0x7F,0xEF,0xFE,0x1B,0x5F,0xEF,0x9F,0xFF,0x3F,0xFB,
0x7F,0x5F,0xF0,0xC3,0x5F,0xEF,0xCF,0xE7,0x79,0xFB,0x60,0x9F,0xFE,0xDB,0x5F,0x9F,
0xE7,0xEF,0x7B,0xF8,0xFF,0xDF,0xFE,0xDB,0xDF,0x7F,0xF1,0xEF,0x7B,0xFB,0xFE,0xDF,
0xFE,0xD3,0x1E,0xFF,0xFF,0xE0,0x03,0xFF,0xFF,0x3F,0xFF,0xDB,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

const uint8_t TypeCodeNone[] = 
{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

const uint8_t NowFrequeceIco[] = 
{
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xBF,
0xDF,0x7E,0xC1,0xEC,0x3F,0xFB,0xBB,0xEE,0xFA,0x37,0x9D,0xBF,0xFD,0xB7,0x00,0x1A,
0xEF,0xBD,0xBF,0xFF,0xBF,0xFF,0xFA,0xC1,0x8B,0x9F,0xF8,0x03,0x83,0xB0,0x1D,0xBF,
0xFF,0xFF,0xFB,0xBA,0xBE,0xD5,0x88,0x3F,0xFF,0xFB,0x82,0xBA,0xD5,0xBD,0xBF,0xFC,
0x03,0xBA,0xBA,0x95,0xAD,0xBF,0xFF,0xFB,0x82,0xB7,0x55,0x9D,0x7F,0xFF,0xFB,0xBA,
0xBE,0xF7,0x3E,0xFF,0xF8,0x03,0xBB,0xBD,0xEB,0xBD,0x7F,0xFF,0xFB,0xB3,0x33,0xDD,
0xB3,0x9F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

const uint8_t ParaSetIco[] =
{
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE,0xFF,
0xEE,0xFB,0x87,0x80,0x3F,0xFD,0xF7,0xAA,0xFD,0xB7,0xB5,0xBF,0xF8,0x03,0xEE,0xFF,
0xB7,0x80,0x3F,0xFF,0x7F,0x02,0x1F,0xB7,0xFB,0xFF,0xF0,0x01,0xCD,0xB1,0x79,0x00,
0x1F,0xFD,0xD7,0xA6,0xBD,0xFF,0xFB,0xFF,0xFB,0x3B,0x6A,0xBD,0x03,0xC0,0x7F,0xF4,
0xED,0xDE,0xBD,0xBB,0xDF,0x7F,0xFF,0x9F,0x06,0xBD,0xD7,0xDF,0x7F,0xF8,0x7B,0xB7,
0x7C,0xEF,0xDB,0x7F,0xFF,0xC7,0xCE,0xBD,0xD7,0xDB,0x7F,0xF8,0x3F,0x35,0xDF,0x39,
0x00,0x1F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

const uint8_t LanguIco[] = 
{
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xF6,0x01,
0xF7,0xFB,0x87,0x80,0x3F,0xFB,0xDF,0xFB,0xFD,0xB7,0xB5,0xBF,0xFF,0x03,0x00,0x1F,
0xB7,0x80,0x3F,0xFF,0xBB,0xFF,0xFF,0xB7,0xFB,0xFF,0xF3,0xBB,0x80,0x31,0x79,0x00,
0x1F,0xFA,0x01,0xFF,0xFD,0xFF,0xFB,0xFF,0xFB,0xFF,0x80,0x3D,0x03,0xC0,0x7F,0xFB,
0x03,0xFF,0xFD,0xBB,0xDF,0x7F,0xFB,0x7B,0x80,0x3D,0xD7,0xDF,0x7F,0xF9,0x7B,0xBF,
0xBC,0xEF,0xDB,0x7F,0xFB,0x03,0x80,0x3D,0xD7,0xDB,0x7F,0xFF,0x7B,0xBF,0xBF,0x39,
0x00,0x1F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

const uint8_t SystemSetIco[] = 
{
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xC3,
0xDE,0xFB,0x87,0x80,0x3F,0xF8,0x3F,0xDF,0x7D,0xB7,0xB5,0xBF,0xFF,0x7F,0xB8,0x1F,
0xB7,0x80,0x3F,0xFE,0xF7,0x6E,0xFF,0xB7,0xFB,0xFF,0xFC,0x0F,0x1D,0xB1,0x79,0x00,
0x1F,0xFF,0x9F,0xDB,0xDD,0xFF,0xFB,0xFF,0xFE,0x7B,0xB8,0x1D,0x03,0xC0,0x7F,0xF8,
0x01,0x0D,0x5D,0xBB,0xDF,0x7F,0xFF,0xBD,0xFD,0x7D,0xD7,0xDF,0x7F,0xFD,0xB7,0xCD,
0x7C,0xEF,0xDB,0x7F,0xFB,0xBB,0x3B,0x5D,0xD7,0xDB,0x7F,0xF7,0x3D,0xF7,0x9F,0x39,
0x00,0x1F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

const uint8_t ExitSetupIco[] = 
{
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFB,0x03,
0xFB,0xFE,0xEF,0xDF,0x7F,0xFD,0x7B,0xBB,0xB0,0x01,0xEE,0xFF,0xFF,0x03,0xBB,0xBE,
0xEF,0x80,0x3F,0xFF,0x7B,0xBB,0xBF,0xE3,0xBB,0xBF,0xF1,0x03,0x80,0x30,0x1F,0x80,
0x3F,0xFD,0x5D,0xFB,0xFB,0x77,0xBB,0xBF,0xFD,0x6B,0xFB,0xFD,0xAF,0x80,0x3F,0xFD,
0x57,0x7B,0xDF,0xBF,0xFB,0xFF,0xFD,0x3B,0x7B,0xD0,0x01,0x00,0x1F,0xFD,0x7B,0x7B,
0xDE,0xAF,0xFB,0xFF,0xFA,0xFF,0x00,0x1D,0xB7,0xFB,0xFF,0xF7,0x01,0xFF,0xD3,0xB9,
0xFB,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

const uint8_t ChooseDevIco[] = 
{
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFB,0xEF,
0xD0,0x3D,0xFF,0xC0,0x7F,0xFD,0xAF,0xDB,0xBD,0x87,0xDF,0x7F,0xFF,0x83,0xDD,0x7D,
0xB7,0xDF,0x7F,0xFF,0x6F,0x0E,0xF0,0xB7,0xC0,0x7F,0xF1,0xEF,0xDD,0x7D,0xB7,0xFF,
0xFF,0xFD,0x01,0xD2,0x98,0xB7,0x00,0x1F,0xFD,0xD7,0xCE,0xF9,0x37,0xEF,0xFF,0xFD,
0xD7,0x18,0x35,0xB7,0xDF,0xFF,0xFD,0xB5,0xDE,0xFD,0xB7,0xC0,0x3F,0xFD,0x79,0xD0,
0x1D,0xB5,0xFF,0xBF,0xFA,0xFF,0xDE,0xFD,0x75,0xFF,0xBF,0xF7,0x01,0x9E,0xFC,0xF9,
0xFE,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

const uint8_t BackLightIco[] = 
{
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE,0xDB,
0xFB,0xFF,0xBF,0xFD,0xFF,0xF0,0xC7,0xBB,0xB0,0x01,0x80,0x1F,0xFE,0xDD,0xDB,0x7F,
0xFF,0xB7,0x7F,0xFC,0xE1,0xEA,0xFC,0x07,0x80,0x1F,0xF2,0xFF,0xFB,0xFD,0xF7,0xB7,
0x7F,0xFC,0x07,0x00,0x1C,0x07,0xB0,0x7F,0xFD,0xF7,0xEE,0xFF,0xFF,0xBF,0xFF,0xFC,
0x07,0xEE,0xF0,0x01,0xA0,0x3F,0xFD,0xF7,0xEE,0xF7,0xFD,0xB7,0xBF,0xFC,0x07,0xDE,
0xDC,0x0F,0xBB,0x7F,0xFD,0xF7,0xBE,0xDD,0xED,0xBC,0xFF,0xFD,0xE7,0x7F,0x13,0xF1,
0x63,0x1F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

const uint8_t BackLightTimeIco[] = 
{
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE,0xDB,
0xFB,0xFF,0xFB,0xB0,0x1F,0xF0,0xC7,0xBB,0xB0,0xFB,0xDF,0xDF,0xFE,0xDD,0xDB,0x76,
0xFB,0xFF,0xDF,0xFC,0xE1,0xEA,0xF6,0x81,0xA0,0x5F,0xF2,0xFF,0xFB,0xF6,0xFB,0xAF,
0x5F,0xFC,0x07,0x00,0x10,0xFB,0xAF,0x5F,0xFD,0xF7,0xEE,0xF6,0xDB,0xA0,0x5F,0xFC,
0x07,0xEE,0xF6,0xEB,0xAF,0x5F,0xFD,0xF7,0xEE,0xF6,0xEB,0xAF,0x5F,0xFC,0x07,0xDE,
0xD0,0xFB,0xA0,0x5F,0xFD,0xF7,0xBE,0xD6,0xFB,0xBF,0xDF,0xFD,0xE7,0x7F,0x1F,0xE3,
0xBF,0x1F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

const uint8_t TurnoffTimeIco[] = 
{
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD,0xF7,
0xDF,0xFF,0xFB,0xB0,0x1F,0xFE,0xEF,0xD8,0x70,0xFB,0xDF,0xDF,0xFF,0xFF,0xDB,0x76,
0xFB,0xFF,0xDF,0xF8,0x03,0x0B,0x76,0x81,0xA0,0x5F,0xFF,0xBF,0xDB,0x76,0xFB,0xAF,
0x5F,0xFF,0xBF,0x8B,0x70,0xFB,0xAF,0x5F,0xF0,0x01,0x93,0x76,0xDB,0xA0,0x5F,0xFF,
0xBF,0x5B,0x76,0xEB,0xAF,0x5F,0xFF,0x5F,0xDB,0x76,0xEB,0xAF,0x5F,0xFE,0xEF,0xDB,
0x50,0xFB,0xA0,0x5F,0xFD,0xF7,0xD7,0x56,0xFB,0xBF,0xDF,0xF3,0xF9,0xCF,0x9F,0xE3,
0xBF,0x1F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

const uint8_t IDIco[] = 
{
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFD,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xBE,0xFC,0x21,0xFF,0xFE,0x08,0x7F,0xDF,
0xFD,0xAD,0xFF,0xFF,0xBD,0xBF,0xF0,0x1D,0xAD,0xFF,0xFF,0xBD,0xBF,0xFE,0xFD,0xAD,
0xFF,0xFF,0xBD,0xBF,0xBE,0xFD,0xAD,0xFF,0xFF,0xBD,0xBF,0xDE,0xF8,0x00,0xFF,0xFF,
0xBD,0xBF,0xF0,0x1D,0xAD,0xFF,0xFF,0xBD,0xBF,0xFE,0xFD,0xAD,0xFF,0xFE,0x08,0x7F,
0xDE,0xFD,0xAD,0xFF,0xFF,0xFF,0xFF,0xBE,0xFD,0xAD,0xFF,0xFF,0xFF,0xFF,0xE0,0x0B,
0x19,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

const uint8_t ForceIco[] = 
{
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE,0x0F,0xBF,0x7F,0xC6,0xEE,0xBF,0x7F,
0xF6,0xEE,0x0B,0x7F,0xF6,0x0D,0xBB,0x7F,0xC7,0xBC,0x03,0x7F,0xDC,0x07,0xBB,0x7F,
0xDD,0xB6,0x0B,0x7F,0xC5,0xB6,0xAB,0x7F,0xF4,0x06,0xAB,0x7F,0xF7,0xBE,0xAF,0x7F,
0xF7,0xAE,0x8F,0x7F,0xCC,0x07,0xBC,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

const uint8_t InoutIco[] = 
{
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xEE,0xDF,0xEF,0xFF,0xF6,0xDE,0xEE,0xFF,
0xFC,0x0E,0xEE,0xFF,0xFE,0xDE,0xEE,0xFF,0xC6,0xDE,0x00,0xFF,0xF6,0xDF,0xEF,0xFF,
0xF0,0x07,0xEF,0xFF,0xF6,0xDD,0xEF,0x7F,0xF6,0xDD,0xEF,0x7F,0xF5,0xDD,0xEF,0x7F,
0xEB,0xFC,0x00,0x7F,0xDC,0x07,0xFF,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

const uint8_t SpacingIco[] = 
{
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xEC,0x07,0xFF,0xFF,0xF7,0xF6,0x10,0x7F,
0xFF,0xF6,0xD7,0xFF,0xE8,0x16,0xD7,0xFF,0xEB,0xD6,0x10,0x7F,0xEB,0xD7,0xB7,0x7F,
0xE8,0x17,0xB7,0x7F,0xEB,0xD6,0x97,0x7F,0xEB,0xD6,0xB0,0x7F,0xE8,0x16,0xB7,0xFF,
0xEF,0xF6,0x97,0xFF,0xEF,0xC4,0x70,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

const uint8_t LengthIco[] = 
{
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFB,0xFF,0xF7,0xFF,0xFB,0xCE,0x00,0x7F,
0xFB,0x3E,0xDD,0xFF,0xF8,0xFE,0x00,0x7F,0xFB,0xFE,0xDD,0xFF,0xC0,0x06,0xC1,0xFF,
0xFB,0x7E,0xFF,0xFF,0xFB,0x7E,0x80,0xFF,0xFB,0xBE,0xDE,0xFF,0xFA,0xDE,0xED,0xFF,
0xF9,0xE6,0xF3,0xFF,0xFB,0xFD,0x8C,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

const uint8_t MovementIco[] = 
{
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE,0xDF,0xFE,0xF8,
0x03,0xFF,0xF1,0xC3,0x8E,0xFB,0xBB,0xFF,0xFD,0xBB,0xFE,0xFB,0xDB,0xFF,0xFD,0x57,
0xFC,0x10,0x01,0xFF,0xF0,0xEF,0x06,0xDB,0xBB,0xFF,0xFD,0xD7,0xDE,0xD8,0x03,0xFF,
0xF9,0xA1,0xBE,0xDB,0xBB,0xFF,0xF8,0xDD,0xAE,0xD8,0x03,0xFF,0xF5,0xAB,0x76,0xDF,
0xBF,0xFF,0xFD,0xF7,0x05,0xD8,0x03,0xFF,0xFD,0xCF,0xFD,0xDF,0xBF,0xFF,0xFD,0x3F,
0xFB,0x30,0x01,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

const uint8_t IndelayIco[] = 
{
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC,0xFF,0xFF,0x9F,
0xFB,0xFF,0xFF,0x7F,0x08,0x70,0xFB,0xFF,0xFF,0xBF,0xEF,0x76,0xFB,0xFF,0xFF,0xBF,
0xDF,0x76,0x81,0xFF,0xFF,0x5F,0xBB,0x76,0xFB,0xFF,0xFF,0x5F,0x8B,0x10,0xFB,0xFF,
0xFF,0x5F,0xEB,0x76,0xDB,0xFF,0xFE,0xEF,0xEB,0x76,0xEB,0xFF,0xFE,0xEF,0xAB,0x76,
0xEB,0xFF,0xFD,0xF7,0xD8,0x10,0xFB,0xFF,0xFB,0xFB,0xAF,0xF6,0xFB,0xFF,0xF7,0xFD,
0x70,0x1F,0xE3,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

const uint8_t OutnumIco[] = 
{
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xBF,0xEE,0xF8,
0x03,0xFF,0xFB,0xBB,0xAA,0xFB,0xBB,0xFF,0xFB,0xBB,0xEE,0xFB,0xDB,0xFF,0xFB,0xBB,
0x02,0x10,0x01,0xFF,0xF8,0x03,0xCD,0xBB,0xBB,0xFF,0xFF,0xBF,0xA6,0xB8,0x03,0xFF,
0xFF,0xBF,0x6A,0xBB,0xBB,0xFF,0xF7,0xBD,0xDE,0xB8,0x03,0xFF,0xF7,0xBD,0x06,0xBF,
0xBF,0xFF,0xF7,0xBD,0xB7,0x78,0x03,0xFF,0xF0,0x01,0xCE,0xBF,0xBF,0xFF,0xFF,0xFD,
0x35,0xD0,0x01,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
};

const uint8_t ABIco[] = 
{
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xBF,0xBF,0xFF,0xFF,0xFF,0x7F,
0x7F,0xDC,0x3F,0xFF,0xFF,0x7E,0xFF,0xEE,0xDF,0xFF,0xFF,0x3D,0xFF,0xF6,0xDF,0xFF,
0xFE,0xBB,0xFF,0xFA,0x3F,0xFF,0xFE,0xBD,0xFF,0xF6,0xDF,0xFF,0xFE,0x1E,0xFF,0xEE,
0xDF,0xFF,0xFE,0xDF,0x7F,0xDE,0xDF,0xFF,0xFC,0xCF,0xBF,0xBC,0x3F,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

const uint8_t ExitIco[] = 
{
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xEC,0x0F,0xEF,0xFF,0xF5,0xEE,0xEE,0xFF,
0xFC,0x0E,0xEE,0xFF,0xFD,0xEE,0xEE,0xFF,0xC4,0x0E,0x00,0xFF,0xF5,0x77,0xEF,0xFF,
0xF5,0xAF,0xEF,0xFF,0xF5,0x5D,0xEF,0x7F,0xF4,0xED,0xEF,0x7F,0xF5,0xED,0xEF,0x7F,
0xEB,0xFC,0x00,0x7F,0xDC,0x07,0xFF,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

const uint8_t PoweroffIco[] = 
{
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xCF,0xFF,0xFF,0xFF,0xC0,0xFF,
0xFF,0xFE,0x07,0xFF,0xFF,0xF3,0x00,0xFF,0xFF,0xF0,0x0F,0xFF,0xFF,0xE0,0x0C,0xFF,
0xFF,0xC0,0x1F,0xFF,0xFF,0xE0,0x3E,0x7F,0xFF,0x83,0x3F,0xFF,0xFF,0xC5,0xFE,0x7F,
0xFF,0x8E,0x7F,0xFF,0xFF,0xCF,0xFE,0x7F,0xFF,0xFC,0xFF,0xFF,0xFF,0xDF,0xFE,0x7F,
0xFF,0xFD,0xFF,0xFF,0xFF,0xDF,0xFE,0x7F,0xFF,0xFB,0x7F,0xFF,0xFF,0xDF,0x3E,0x7F,
0xFF,0xF6,0x47,0xFF,0xFF,0xDF,0x1E,0xFF,0xFF,0xE4,0x03,0xFF,0xFF,0xDF,0x1E,0xFF,
0xFF,0xE0,0x03,0xFF,0xFF,0xDF,0x1E,0xFF,0xFF,0xC0,0x73,0xFF,0xFF,0xDE,0x3C,0xFF,
0xFF,0x80,0x33,0xFF,0xFF,0xCE,0x3D,0xFF,0xFF,0x88,0x3B,0xFF,0xFF,0xCC,0x79,0xFF,
0xFF,0x90,0x3B,0xFF,0xFF,0xCC,0x73,0xFF,0xFF,0x90,0x3B,0xFF,0xFF,0xE0,0x07,0xFF,
0xFF,0x1C,0x70,0x3F,0xFF,0xE0,0x8F,0xFF,0xFF,0x38,0x00,0x1F,0xFF,0xF1,0xDF,0xF7,
0xFF,0x00,0x00,0x3F,0xFF,0xF1,0x8F,0xF3,0xFF,0x00,0x1B,0xFF,0xFF,0xE3,0x8F,0xE3,
0xF0,0x07,0xFB,0xFF,0xFF,0xC7,0x9F,0xE3,0xE0,0x3F,0xF3,0xFF,0xFF,0xCF,0x9F,0xC7,
0xC7,0x3F,0xE7,0xFF,0xFF,0x9F,0x8F,0x0F,0xFF,0xBF,0xC7,0xFF,0xFF,0x3F,0x80,0x1F,
0xFF,0x9F,0xCF,0xFF,0xFE,0x7F,0xE0,0xFF,0xFF,0xBF,0xFF,0xFF,0xFE,0x7F,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFC,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

const uint8_t PowerOffTimeIco[] = 
{
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFD,0xDF,0x7E,0xFF,0xC7,0xBE,0xFB,0xFF,0xFD,0xDF,0x7E,0xF8,0x3F,0xDD,0xFB,0x0F,
0xFB,0xEF,0x0E,0xFB,0xFF,0xFF,0xFB,0x6F,0xFB,0xEE,0xF8,0x3B,0xFF,0x00,0x61,0x6F,
0xF7,0xF7,0x0A,0xB8,0x03,0xF7,0xFB,0x6F,0xE8,0x0B,0xBA,0xBB,0xFF,0xF7,0xF1,0x6F,
0xFE,0xEF,0xBA,0xBB,0xFE,0x00,0x32,0x6F,0xFE,0xEE,0x08,0x3A,0x03,0xF7,0xEB,0x6F,
0xFD,0xEF,0xBA,0xBA,0xFB,0xEB,0xFB,0x6F,0xFD,0xEF,0xAE,0xF6,0xFB,0xDD,0xFB,0x6B,
0xFB,0xEF,0x9E,0xF6,0x03,0xBE,0xFA,0xEB,0xF7,0x1F,0xBE,0xEE,0xFA,0x7F,0x39,0xF3,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

const uint8_t PowerOffTimeIco1[] = 
{
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFD,0xDF,0x7E,0xFF,0xC7,0xBE,0xFB,0xFF,0xE3,0xDF,0x7E,0xF8,0x3F,0xDD,0xFB,0x0F,
0xFB,0xD7,0x0E,0xFB,0xFF,0xFF,0xFB,0x6F,0xFB,0x5A,0xF8,0x3B,0xFF,0x00,0x61,0x6F,
0xE1,0x5B,0x0A,0xB8,0x03,0xF7,0xFB,0x6F,0xFB,0x5B,0xBA,0xBB,0xFF,0xF7,0xF1,0x6F,
0xF2,0xDF,0xBA,0xBB,0xFE,0x00,0x32,0x6F,0xF1,0xDA,0x08,0x3A,0x03,0xF7,0xEB,0x6F,
0xEB,0xF7,0xBA,0xBA,0xFB,0xEB,0xFB,0x6F,0xFB,0xEF,0xAE,0xF6,0xFB,0xDD,0xFB,0x6B,
0xFB,0x9F,0x9E,0xF6,0x03,0xBE,0xFA,0xEB,0xFA,0x7F,0xBE,0xEE,0xFA,0x7F,0x39,0xF3,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

const uint8_t AlwaysOnIco[] =
{
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xF6,0xDF,0xEF,0xFF,0xFA,0xBC,0x00,0x7F,
0xC0,0x07,0xFF,0xFF,0xDF,0xF7,0x01,0xFF,0xF0,0x1F,0x7D,0xFF,0xF7,0xDF,0x01,0xFF,
0xF0,0x1F,0xFF,0xFF,0xFE,0xFC,0x00,0x7F,0xE0,0x0D,0xFF,0x7F,0xEE,0xEF,0x03,0xFF,
0xEE,0xCF,0x7B,0x7F,0xFE,0xFC,0xFC,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

const uint8_t ChineseIco[] = 
{
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE,0xFF,0xDF,0xFF,0xFE,0xFF,0xEF,0xFF,
0xFE,0xFC,0x00,0x7F,0xE0,0x0F,0x7D,0xFF,0xEE,0xEF,0x7D,0xFF,0xEE,0xEF,0xBB,0xFF,
0xEE,0xEF,0xBB,0xFF,0xE0,0x0F,0xD7,0xFF,0xEE,0xEF,0xEF,0xFF,0xFE,0xFF,0xD7,0xFF,
0xFE,0xFF,0x39,0xFF,0xFE,0xFC,0xFE,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

const uint8_t EnglishIco[] = 
{
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFB,0xBF,0xDF,0xFF,0xC0,0x07,0xEF,0xFF,
0xFB,0xBC,0x00,0x7F,0xFE,0xFF,0x7D,0xFF,0xE0,0x0F,0x7D,0xFF,0xEE,0xEF,0xBB,0xFF,
0xEE,0xEF,0xBB,0xFF,0xEE,0xEF,0xD7,0xFF,0xC0,0x07,0xEF,0xFF,0xFD,0x7F,0xD7,0xFF,
0xF3,0x9F,0x39,0xFF,0xCF,0xE4,0xFE,0x7F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

uint8_t MultiButtonValue[4];
uint8_t MultiKey;
uint8_t TransmitFail = 0;
uint8_t Enableupdate = 0;
uint8_t EnableRead = 1;
int8_t LanguageIndex = 0;

uint16_t CalcuResult = 0;
uint8_t CRC_Result[2] = "";
uint8_t BatFlag = 0;
uint32_t Lock_Code; 
