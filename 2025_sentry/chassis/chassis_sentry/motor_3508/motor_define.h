#ifndef  __MOTOR_DEFINE_H__
#define  __MOTOR_DEFINE_H__
#include "main.h"
#include "pid.h"

typedef struct
{
	uint16_t can_id;//电机ID
    uint16_t rotor_angle;//电机角度
    int16_t  rotor_speed;//电机速度
    int16_t  torque_current;//电机扭矩
    uint8_t  temp;//温度
    int16_t Set_Speed;//设定速度
    uint16_t Set_Angle;//设定角度
    pid_struct_t PID;//PID各种参数
}motor_3508_t;
typedef struct
{
    uint16_t can_id;//电机ID
    int16_t  set_voltage;//设定的电压值
    uint16_t rotor_angle;//电机角度
    int16_t  rotor_speed;//电机速度
    int16_t  torque_current;//电机扭矩
    uint8_t  temp;//温度
    int16_t Set_Speed;//设定速度
    float Set_Angle;//设定角度
    pid_struct_t Speed_PID;    
    pid_struct_t Angle_PID;
}motor_6020_t;
typedef struct{
	uint16_t angle;
	uint16_t speed;
	uint16_t target_angle;
	uint16_t target_speed;
	pid_struct_t Speed_PID;
	pid_struct_t Angle_PID;
}motor_2006_t;
typedef struct
{
    float vx;
    float vy;
    float vw;
}Chassis_Speed_t;
void Chassis_PID_Init_All();


#endif