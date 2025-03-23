#include "main.h"
#include "pid.h"
#include "motor_define.h"

extern motor_3508_t motor_3508[4];
extern motor_6020_t motor_yaw,motor_pitch;

//void pitch_yaw_PID_Init()
//{
//    PID_init(&(motor_pitch.Angle_PID),2.5,0.01,0,16308,16308);//40,0.5,150,16308,16308
//    PID_init(&(motor_pitch.Speed_PID),50,0.3,0,16308,16308);//3,0,0,16308,16308
//    PID_init(&(motor_yaw.Angle_PID),3.5,0,0,16308,16308);
//    PID_init(&(motor_yaw.Speed_PID),45,0.25,0,16308,16308);
//}
//    pid_init(&(motor_yaw.Angle_PID),3.5,0,0,16308,16308);
//    pid_init(&(motor_yaw.Speed_PID),45,0.25,0,16308,16308);

//float PID_CHASSIS_3508_1_SPEED_K[3] = {6,0.5,0};
//float PID_CHASSIS_3508_2_SPEED_K[3] = {6,0.5,0};
//float PID_CHASSIS_3508_3_SPEED_K[3] = {6,0.5,0};
//float PID_CHASSIS_3508_4_SPEED_K[3] = {6,0.5,0};
//float PID_YAW_4310_SPEED_K[3] = {0,0,0};
//float PID_YAW_4310_ANGLE_K[3] = {1.1,0.0025,50};
void Chassis_PID_Init_All()
{
//    pid_init(&(motor_3508[0].PID),10,1,10,2000,2200);+
//    pid_init(&(motor_3508[1].PID),10,1,10,2000,2200);
//    pid_init(&(motor_3508[2].PID),10,1,10,2000,2200);
//    pid_init(&(motor_3508[3].PID),10,1,10,2000,2200);
	pid_init(&(motor_3508[0].PID),6,0.8,0,16000,16307);
    pid_init(&(motor_3508[1].PID),6,0.8,0,16000,16307);
    pid_init(&(motor_3508[2].PID),6,0.8,0,16000,16307);
    pid_init(&(motor_3508[3].PID),6,0.8,0,16000,16307);
	//pid_init(&(motor_3508[3].PID),10,1,10,16308,16308);
}