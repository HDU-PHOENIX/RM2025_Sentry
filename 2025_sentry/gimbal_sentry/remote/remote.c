#include "main.h"
#include "remote.h"
#include "baspcan.h"
uint8_t pdata[18];

RC_Ctl_t RC_CtrlData;     //代码里面没有加失联保护

extern int16_t pitch_4310_remote;
extern int16_t yaw_6020_remote;

void RemoteDataProcess(uint8_t *pData) //在UART的回调函数里被调用，将数据pData的信息解码后给RC
{
 if(pData == NULL)
 {
 return;
 }
 
 RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
 RC_CtrlData.rc.ch0=RC_CtrlData.rc.ch0-1024;
 RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) 
& 0x07FF;
 RC_CtrlData.rc.ch1=RC_CtrlData.rc.ch1-1024;
 RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
 ((int16_t)pData[4] << 10)) & 0x07FF;
 RC_CtrlData.rc.ch2=RC_CtrlData.rc.ch2-1024;   
 RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 
0x07FF;
 RC_CtrlData.rc.ch3=RC_CtrlData.rc.ch3-1024;
 RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
 RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);
 RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
 RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
 RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8); 
 RC_CtrlData.mouse.press_l = pData[12];
 RC_CtrlData.mouse.press_r = pData[13];
 RC_CtrlData.key.v = ((int16_t)pData[14]);// | ((int16_t)pData[15] << 8);
 RC_CtrlData.wheel=(pData[16] | pData[17] << 8) - 1024;

 
}//解码函数,所有的解码函数都一个样子，就像所有的PID算法都是一个模样，复制即可，

