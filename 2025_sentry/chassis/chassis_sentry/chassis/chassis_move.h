#ifndef  __CHASSIS_MOVE_H__
#define  __CHASSIS_MOVE_H__

#include "can.h"
#include "motor_define.h"

void Chassis_Remote_Control_normal();
void Chassis_Remote_Control_gyro();
float Find_Angle();
void Chassis_Solution();
void Chassis_Motor_Solution();
void Chassis_PID_Calc();
void Set_M3508_Chassis_Voltage(CAN_HandleTypeDef* hcan,motor_3508_t M3508_Chassis[4]);
void Chassis_Move();
void chassis_remote_control();
void Chassis_Move_Auto();


#endif