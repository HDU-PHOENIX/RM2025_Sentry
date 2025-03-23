#include "main.h"
#include "FreeRTOS.h"
#include "Task.h"
#include "baspcan.h"
#include "motor_define.h"
#include "pid.h"
#include "can.h"
#include "remote.h"
#include "shoot.h"
#include "DM4310.h"
#include "cmsis_os.h"

extern RC_Ctl_t RC_CtrlData;
extern motor_3508_t motor_3508_shoot[2];
extern motor_2006_t motor_2006_shoot;
extern uint8_t s[2];
extern int16_t wheel;
extern uint16_t total_heat;
int16_t Shoot_Speed = 6500, Aummer_Speed = 0;
uint8_t heat_flag=1;
/**
 * @brief 判断是否需要换蛋
 */
//void Set_M2006_Motor_Voltage(CAN_HandleTypeDef* hcan,motor_2006_t M2006_Rammer)
//{
//    CAN_TxHeaderTypeDef tx_header;
//    uint8_t             tx_data[8] = {0};
//    
//    tx_header.StdId = 0x200;//标识符（见手册P6）
//    tx_header.IDE   = CAN_ID_STD;//标准ID
//    tx_header.RTR   = CAN_RTR_DATA;//数据帧
//    tx_header.DLC   = 8;//字节长度

//    tx_data[4] = ((int16_t)M2006_Rammer.PID.output>>8)&0xff;
//    tx_data[5] = ((int16_t)M2006_Rammer.PID.output)&0xff;

//    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
//}

void Set_M3508_Shoot_Voltage(CAN_HandleTypeDef* hcan,motor_3508_t M3508_Shoot[2],motor_2006_t M2006_Rammer)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t             tx_data[8] = {0};
    
    tx_header.StdId = 0x200;//标识符（见手册P6）
    tx_header.IDE   = CAN_ID_STD;//标准ID
    tx_header.RTR   = CAN_RTR_DATA;//数据帧
    tx_header.DLC   = 8;//字节长度


    tx_data[0] = ((int16_t)M3508_Shoot[0].PID.output>>8)&0xff;
    tx_data[1] = ((int16_t)M3508_Shoot[0].PID.output)&0xff;

    tx_data[2] = ((int16_t)M3508_Shoot[1].PID.output>>8)&0xff;
    tx_data[3] = ((int16_t)M3508_Shoot[1].PID.output)&0xff;

    tx_data[4] = ((int16_t)M2006_Rammer.PID.output>>8)&0xff;
    tx_data[5] = ((int16_t)M2006_Rammer.PID.output)&0xff;

	
	
	if(HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)
	{
		if(HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			if(HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX2) != HAL_OK)
			{
			
			}
		}
	}
}

/**
 * @brief 计算PID
 */
void Shoot_PID_Calc()
{
    PID_Calc_Speed(&(motor_3508_shoot[0].PID),motor_3508_shoot[0].Set_Speed,motor_3508_shoot[0].rotor_speed);
    PID_Calc_Speed(&(motor_3508_shoot[1].PID),motor_3508_shoot[1].Set_Speed,motor_3508_shoot[1].rotor_speed);
    PID_Calc_Speed(&(motor_2006_shoot.PID),motor_2006_shoot.Set_Speed,motor_2006_shoot.rotor_speed);
}

/**
 * @brief 初始化PID
 */
void Shoot_PID_Init_ALL()
{
    pid_init(&(motor_3508_shoot[0].PID),10,0.5,0,16380,16380);
    pid_init(&(motor_3508_shoot[1].PID),10,0.5,0,16380,16380);
    pid_init(&(motor_2006_shoot.PID),3.2,0.004,0.0005,16380,16380);
}

void shoot_start(void const * argument){
	portTickType currentTime;
	Shoot_PID_Init_ALL();
	for(;;){
		currentTime = xTaskGetTickCount();
		Motor_DM4310_Enable_position(&hcan1,0x01);
		
//		if(total_heat>180)
//		{
//			heat_flag=0;
//		}
//		if(total_heat<120)
//		{
//			heat_flag=1;
//		}
		
		position_speed_control(&hcan1,0x01,DM4310_pitch.target_angle,6);
		if((s[0]==1 && s[1]==1)){
			motor_3508_shoot[0].Set_Speed = -Shoot_Speed;
			motor_3508_shoot[1].Set_Speed = Shoot_Speed;  //4000
		}
		else if((s[0]==1 && s[1]==3)){
			motor_3508_shoot[0].Set_Speed = -Shoot_Speed;
			motor_3508_shoot[1].Set_Speed = Shoot_Speed;  //4000
		if(wheel>200){
			motor_2006_shoot.Set_Speed=-3000;
		}
		if( wheel<-200 && heat_flag==1){
			motor_2006_shoot.Set_Speed=7000;
		}
		if( heat_flag==0){
			motor_2006_shoot.Set_Speed=0;
		}
		if( wheel<20 && wheel>-20 ){
			motor_2006_shoot.Set_Speed=0;
		}
			
		}
		if(s[0]!=1){
			motor_3508_shoot[0].Set_Speed = 0;
			motor_3508_shoot[1].Set_Speed =0;
		}
//		if(s[0]==2 && s[1]==2){
//			motor_3508_shoot[0].Set_Speed = 0;
//			motor_3508_shoot[1].Set_Speed =0;
//		}
//		if(s[0]!=0&&s[1]!=0){
//		if(wheel>200){
//			motor_2006_shoot.Set_Speed=-5000;
//		}
//		if( wheel<-200){
//			motor_2006_shoot.Set_Speed=5000;
//		}
//	}
		if(s[0]!=1){
			motor_2006_shoot.Set_Speed=0;
		}
//		if(wheel<50 && wheel>-50){
//			motor_2006_shoot.Set_Speed=0;
//		}
		Shoot_PID_Calc();
		Set_M3508_Shoot_Voltage(&hcan1,motor_3508_shoot,motor_2006_shoot);
//		Set_M2006_Motor_Voltage(&hcan1,motor_2006_shoot);
		vTaskDelayUntil(&currentTime,2);
	}
}
