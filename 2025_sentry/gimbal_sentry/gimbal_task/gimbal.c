#include "main.h"
#include "remote.h"
#include "motor_define.h"
#include "gimbal.h"
#include "pid.h"
#include "can.h"

#define Gimbal_Pithch_Set 120  //
#define Gimbal_Yaw_Set 50  //遥控器灵敏度
#define GYROSCOPE_SPEED 12.83 //小陀螺速度
#define Gimbal_Pitch_MAX 1.75
#define Gimbal_Pitch_MIN 0.62 //pitch角度的上下限

extern RC_Ctl_t RC_CtrlData;
extern motor_3508_t motor_3508[4];
extern motor_6020_t motor_yaw;
extern pid_struct_t gimbal_4310_speed,gimbal_4310_angle;
motor_6020_t motor_pitch;
Gimbal_t gimbal_remote;


void Gimbal_Remote_Control_normal()
{
	gimbal_remote.Pitch = (float)RC_CtrlData.rc.ch1/Gimbal_Pithch_Set;
    gimbal_remote.Yaw = (float)RC_CtrlData.rc.ch0/Gimbal_Yaw_Set;
	motor_yaw.Set_Angle -= gimbal_remote.Yaw;
}
void Gimbal_Remote_Control_gyro(){
	gimbal_remote.Pitch = (float)RC_CtrlData.rc.ch1/Gimbal_Pithch_Set;
    gimbal_remote.Yaw = (float)RC_CtrlData.rc.ch0/Gimbal_Yaw_Set;
	motor_yaw.Set_Angle = motor_yaw.Set_Angle+GYROSCOPE_SPEED-gimbal_remote.Yaw;
}
void Gimbal_Calculate()
{
//    if(RC_CtrlData.rc.s2==1)
//    {
//        motor_yaw.Set_Angle = motor_yaw.Set_Angle+GYROSCOPE_SPEED-gimbal_remote.Yaw;
//    }else if(RC_CtrlData.rc.s2==2)
//    {
//        motor_yaw.Set_Angle -= gimbal_remote.Yaw;
//    }
    while(motor_yaw.Set_Angle <= 0)
		{
			motor_yaw.Set_Angle += 8191;		
		}
	while(motor_yaw.Set_Angle >= 8192)
		{
			motor_yaw.Set_Angle -= 8191;
		}


    motor_pitch.Set_Angle += gimbal_remote.Pitch;
    motor_pitch.Set_Angle =float_Limit_Min_Max(motor_pitch.Set_Angle,Gimbal_Pitch_MIN,Gimbal_Pitch_MAX);
}
void Gimbal_PID_Calc()
{
    PID_Calc_Angle(&(motor_yaw.Angle_PID),motor_yaw.Set_Angle,motor_yaw.rotor_angle);	
    PID_Calc_Angle(&(motor_pitch.Angle_PID),motor_pitch.Set_Angle,motor_pitch.rotor_angle);
    motor_pitch.Set_Speed = motor_pitch.Angle_PID.output;
    motor_yaw.Set_Speed = motor_yaw.Angle_PID.output;
    PID_Calc_Speed(&(motor_pitch.Speed_PID),motor_pitch.Set_Speed,motor_pitch.rotor_speed);
    PID_Calc_Speed(&(motor_yaw.Speed_PID),motor_yaw.Set_Speed,motor_yaw.rotor_speed);
}
void Set_GM6020_Gimbal_Voltage(CAN_HandleTypeDef* hcan,motor_6020_t GM6020_Yaw,motor_6020_t GM6020_Pitch)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t             tx_data[8] = {0};
    
    tx_header.StdId = 0X1FF;//标识符（见手册P6）
    tx_header.IDE   = CAN_ID_STD;//标准ID
    tx_header.RTR   = CAN_RTR_DATA;//数据帧
    tx_header.DLC   = 8;//字节长度
    tx_data[4] = ((int16_t)GM6020_Yaw.Speed_PID.output>>8)&0xff;
    tx_data[5] = ((int16_t)GM6020_Yaw.Speed_PID.output)&0xff;
    
	
	
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}

void Gimbal_Move()
{
    Gimbal_Calculate();

    Gimbal_PID_Calc();

    //Set_GM6020_Gimbal_Voltage(&hcan1,motor_yaw,motor_pitch);
}