#include "main.h"
#include "FreeRTOS.h"
#include "Task.h"
#include "DM4310.h"
#include "pid.h"
#include "can.h"
#include "remote.h"
#include "motor_define.h"
#include  "baspcan.h"
#include "math.h"
#include "gimbal.h"
#include "vision.h"
#include "cmsis_os.h"
#include "LK_9025.h"
#include "shoot.h"
//motor_9025

extern uint8_t heat_flag;

extern motor_3508_t motor_3508_shoot[2];
extern int16_t Shoot_Speed;
extern int16_t pitch_4310_remote;
extern int16_t yaw_6020_remote;
extern uint8_t s[2];

extern pid_struct_t gimbal_4310_speed,gimbal_4310_angle;
extern motor_6020_t motor_yaw;
extern RC_Ctl_t RC_CtrlData;

extern  DM4310_TypeDef DM4310_pitch;
extern computer_all computer_from;

extern float YAW_Vision,PITCH_Vision;
float target_speed_yaw=0,target_speed_pitch=0;
//float PID_CHASSIS_3508_1_SPEED_K[3] = {6,0.5,0};
//float PID_CHASSIS_3508_2_SPEED_K[3] = {6,0.5,0};
//float PID_CHASSIS_3508_3_SPEED_K[3] = {6,0.5,0};
//float PID_CHASSIS_3508_4_SPEED_K[3] = {6,0.5,0};
//float PID_YAW_4310_SPEED_K[3] = {0,0,0};

//float PID_GIMBAL_6020_SPEED_K[3] = {75,1.25,0.5};//130,0.6,0
//float PID_GIMBAL_6020_ANGLE_K[3] = {0.75,0.000300000014,11.5};//0.4,0.0003,7.8
float chazhi[2]={0};
float angle_rpm_pitch=0.01f;
float speed_rpm_pitch=6.0f;
//float 
float angle_test=0.0f;
float canshu_yaw_zimiao=8191.0f;
float canshu_pitch_zimiao=1.0f;
float PID_YAW_4310_ANGLE_K[3] = {1.1,0.0025,50};

extern motor_9025 motor_9025_yaw;
extern uint16_t count_lost;

extern 	float yaw_imu;

extern uint8_t progress;

//uint8_t gyro_flag=0,gyro_angle=0;

float angle_for_9025=0;

extern motor_3508_t motor_3508_shoot[2];
extern motor_2006_t motor_2006_shoot;

extern motor_3508_t motor_3508_shoot[2];

extern motor_2006_t motor_2006_shoot;

void pid_gimbal(){
	pid_init(&gimbal_4310_speed,0.62,0.015,0,13,14);
	pid_init(&gimbal_4310_angle,53.6,0,35,3,3);
	pid_init(&motor_yaw.Speed_PID,189,3.2,0,25000,25000);
	//pid_init(&motor_yaw.Speed_PID,189,3.2,0,25000,25000);
	motor_yaw.Angle_PID.i_seperate=300;
	motor_yaw.Angle_PID.d_seperate=300;
	pid_init(&motor_yaw.Angle_PID,0.2,0,0.05,25000,25000);
	//pid_init(&motor_yaw.Angle_PID,1.8,0.008,3.04,25000,25000);
}
void pid_caluate_gimbal(){ //顺序很重要，必须在pid计算前面
	
	PID_Calc_Angle(&(motor_yaw.Angle_PID),motor_yaw.Set_Angle,motor_yaw.rotor_angle);	
    PID_Calc_Speed(&(motor_yaw.Speed_PID),motor_yaw.Angle_PID.output,motor_yaw.rotor_speed);
	//PID_Calc_Speed(&(motor_yaw.Speed_PID),target_speed_yaw,motor_yaw.rotor_speed);
	
	
	//PID_Calc_Angle(&gimbal_4310_angle,DM4310_pitch.target_angle,DM4310_pitch.angle);
	//PID_Calc_Speed(&gimbal_4310_speed,gimbal_4310_angle.output,DM4310_pitch.speed_rpm);
	//PID_Calc_Speed(&gimbal_4310_speed,DM4310_pitch.target_speed_rpm,DM4310_pitch.speed_rpm);
}
void motor_6020_sent(){
		CAN_TxHeaderTypeDef tx_header;
		uint8_t             tx_data[8] = {0};
    
		tx_header.StdId = 0x1FF;//标识符（见手册P6）
		tx_header.IDE   = CAN_ID_STD;//标准ID
		tx_header.RTR   = CAN_RTR_DATA;//数据帧
		tx_header.DLC   = 8;//字节长度
		tx_data[4] = ((int16_t)motor_yaw.Speed_PID.output>>8)&0xff;
		tx_data[5] = ((int16_t)motor_yaw.Speed_PID.output)&0xff;
		
		if(HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK){
				if(HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX1)!=HAL_OK){
					HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX2);
				}
		
		}
		
		
		
		if(hcan2.ErrorCode==HAL_CAN_STATE_LISTENING){
			HAL_CAN_Stop(&hcan2);  // ??CAN???
			HAL_CAN_Start(&hcan2); // ????CAN???
		
		}
		if(hcan2.ErrorCode==HAL_CAN_STATE_ERROR){
			HAL_CAN_Stop(&hcan2);  // ??CAN???
			HAL_CAN_Start(&hcan2); // ????CAN??
		
		}
		
		if(hcan1.ErrorCode==HAL_CAN_STATE_LISTENING){
			HAL_CAN_Stop(&hcan1);  // ??CAN???
			HAL_CAN_Start(&hcan1); // ????CAN???
		
		}
		if(hcan1.ErrorCode==HAL_CAN_STATE_ERROR){
			HAL_CAN_Stop(&hcan1);  // ??CAN???
			HAL_CAN_Start(&hcan1); // ????CAN??
		
		}
}
void gimbal_task_start(void const * argument){  //采用了加次数的方式来缓解传的速率过快
	portTickType currentTime;
	uint8_t count_update=4,i=0;
	pid_gimbal();
	uint8_t count = 0;
	Motor_DM4310_Enable_position(&hcan1,0x01);
	DM4310_pitch.target_angle=0.0f; //0.62~1.75 -0.86
	motor_yaw.Set_Angle=2674;
	motor_9025_yaw.target_angle=35500;
	
	Motor_DM4310_Enable(&hcan1,0x01);
	DM4310_pitch.target_speed_rpm=0;
	chazhi[0]=0.02;
	chazhi[1]=1-chazhi[0];
	osDelay(1);
    for(;;)
    {
		currentTime = xTaskGetTickCount();
		vision_transmit();
		
		
		if(hcan2.State==HAL_CAN_STATE_LISTENING){
			
			HAL_CAN_Stop(&hcan2);  // ??CAN???
			HAL_CAN_DeInit(&hcan2); // ????CAN???(??)
			HAL_CAN_Init(&hcan2);   // ?????CAN???
			HAL_CAN_Start(&hcan2);  // ??CAN???       重置
			
			hcan2.Instance->ESR = 0;  // ?????????
			HAL_CAN_ResetError(&hcan2);  // ??????    重置
			
			HAL_CAN_Stop(&hcan2);  // ??CAN???     
			
			
			
			HAL_CAN_Start(&hcan2); // ????CAN???      重置
		
		}
		if(hcan2.State==HAL_CAN_STATE_ERROR){
			
			HAL_CAN_Stop(&hcan2);  // ??CAN???
			HAL_CAN_DeInit(&hcan2); // ????CAN???(??)
			HAL_CAN_Init(&hcan2);   // ?????CAN???
			HAL_CAN_Start(&hcan2);  // ??CAN???
			
			hcan2.Instance->ESR = 0;  // ?????????
			HAL_CAN_ResetError(&hcan2);  // ??????
			
			HAL_CAN_Stop(&hcan2);  // ??CAN???
			
			
			
			HAL_CAN_Start(&hcan2); // ????CAN???
		
		}
		
		if(hcan1.State==HAL_CAN_STATE_LISTENING){
			HAL_CAN_Stop(&hcan1);  // ??CAN???
			HAL_CAN_DeInit(&hcan1); // ????CAN???(??)
			HAL_CAN_Init(&hcan1);   // ?????CAN???
			HAL_CAN_Start(&hcan1);  // ??CAN???
			
			hcan2.Instance->ESR = 0;  // ?????????
			HAL_CAN_ResetError(&hcan1);  // ??????
			
			HAL_CAN_Stop(&hcan1);  // ??CAN???
			
			
			
			HAL_CAN_Start(&hcan1); // ????CAN???
		
		}
		if(hcan1.State==HAL_CAN_STATE_ERROR){
			HAL_CAN_Stop(&hcan1);  // ??CAN???
			HAL_CAN_DeInit(&hcan1); // ????CAN???(??)
			HAL_CAN_Init(&hcan1);   // ?????CAN???
			HAL_CAN_Start(&hcan1);  // ??CAN???
			
			hcan2.Instance->ESR = 0;  // ?????????
			HAL_CAN_ResetError(&hcan1);  // ??????
			
			HAL_CAN_Stop(&hcan1);  // ??CAN???
			
			
			
			HAL_CAN_Start(&hcan1); // ????CAN???
		
		}
		Motor_DM4310_Enable_position(&hcan1,0x01);
	
		
		if(s[0]==1){  //自瞄模式
			if(s[1]==1){
			if(computer_from.zimiao.fine_bool== '1'){
				if(motor_3508_shoot[0].Set_Speed!=0 && heat_flag==1){
					motor_2006_shoot.Set_Speed=7000;
					//motor_2006_shoot.Set_Speed=0;
				}
				if(heat_flag==0){
					motor_2006_shoot.Set_Speed=0;
				}
				//motor_2006_shoot.Set_Speed=0;
			}
			if(heat_flag==0){
				motor_2006_shoot.Set_Speed=0;
			}
			
			
			}
	
			
			
			if(computer_from.zimiao.fine_bool== '1' || computer_from.zimiao.fine_bool== '2'){
				
				DM4310_pitch.target_angle=computer_from.zimiao.pitch;
				//motor_yaw.Set_Angle=(computer_from.zimiao.big_yaw-0)/6.2831852*8191+2674;
				count_lost=0;	
		}				
			if(computer_from.zimiao.fine_bool== '0'){
				if(count_lost>300){
					DM4310_pitch.target_angle=0;
					motor_yaw.Set_Angle=2674;
				}
				
				
				
				}
			}
		else{
			motor_2006_shoot.Set_Speed=0;
		
		}
		if(s[1]!=2 && s[1]!=0){
			if(s[0]==3 && s[1]==1){
			motor_yaw.Set_Angle -= (float)yaw_6020_remote/50.f;
			}
			DM4310_pitch.target_angle=DM4310_pitch.target_angle+(pitch_4310_remote/660.f)*angle_rpm_pitch;
		}
		if(fabs(motor_yaw.Set_Angle-motor_yaw.rotor_angle)>2000){
			
			motor_yaw.Angle_PID.i_seperate=1;
		}
		else{
			motor_yaw.Angle_PID.i_seperate=300;
		
		}

		
		if(DM4310_pitch.target_angle>0.7){
		DM4310_pitch.target_angle=0.7;
}
		if(DM4310_pitch.target_angle<-0.3){
		DM4310_pitch.target_angle=-0.3;
}
		if(motor_yaw.Set_Angle <=1750)
		{
			motor_yaw.Set_Angle = 1750;		
		}
		if(motor_yaw.Set_Angle >= 3451)
		{
			motor_yaw.Set_Angle =3451;
		}		
		//pid_caluate_gimbal();
		//motor_6020_sent();
		position_speed_control(&hcan1,0x01,DM4310_pitch.target_angle,speed_rpm_pitch);
		Set_M3508_Shoot_Voltage(&hcan1,motor_3508_shoot,motor_2006_shoot);
		//HAL_CAN_TxMailbox0CompleteCallback//hcan.Init.AutoRetransmission = ENABLE;
        vTaskDelayUntil(&currentTime,2);//绝对延时
	}
}