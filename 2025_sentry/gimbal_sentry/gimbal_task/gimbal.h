#ifndef  __GIMBAL_H__
#define  __GIMBAL_H__
#include "can.h"
#include "motor_define.h"

typedef struct
{
    float Yaw;
    float Pitch;
}Gimbal_t;
void Gimbal_Remote_Control_normal();
void Gimbal_Remote_Control_gyro();
void Gimbal_Calculate();
void Gimbal_PID_Calc();
void Set_GM6020_Gimbal_Voltage(CAN_HandleTypeDef* hcan,motor_6020_t GM6020_Yaw,motor_6020_t GM6020_Pitch);
void Gimbal_Move();


#endif