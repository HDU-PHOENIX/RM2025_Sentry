#ifndef __SHOOT_H__
#define __SHOOT_H__

#include "main.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "Remote.h"
#include "motor_define.h"

void Shoot_Reload_Choose();
void Shoot_Remote_Control();
void Shoot_Move();
void Shoot_PID_Init_ALL();
void Shoot_PID_Calc();
void Shoot_PID_Clean_ALL();
void Shoot_Stop();
void Set_M3508_Shoot_Voltage(CAN_HandleTypeDef* hcan,motor_3508_t M3508_Shoot[2],motor_2006_t M2006_Rammer);
#endif


