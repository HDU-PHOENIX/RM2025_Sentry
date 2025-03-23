#include "baspcan.h"
#include "main.h"
#include "can.h"
#include "math.h"
#include "cmsis_os.h"
#include "../IMU/INS_task.h"
#include "motor_define.h"
#include "DM4310.h"
#include "LK_9025.h"
#include "judge.h"
#include "vision.h"
#include "shoot.h"
extern uint8_t vision_transmit_buff[32];
//#include "LK_9025.h"
//先设置一个滤波器

motor_3508_t motor_3508_shoot[2];
extern motor_6020_t motor_yaw;
extern motor_2006_t motor_2006_shoot;
extern DM4310_TypeDef DM4310_test;


int16_t pitch_4310_remote;
int16_t yaw_6020_remote;
int16_t wheel;
uint8_t s[2];

uint8_t judge[200];
uint8_t progress;

uint8_t bullet_remaining_num_17mm;
uint8_t bullet_remaining_num_42mm;

uint8_t commd_keyboard;
uint8_t target_robot_ID;

uint8_t shouji;

uint8_t hit_count=0;

extern motor_9025 motor_9025_yaw;

extern int hit_flag;


union target_place {
    struct {
        float place_x;
		float place_y;
    } places;
    uint8_t bytes[sizeof(float) * 2]; // ??float??????
};
union target_place target_robot;

union game_time {
	uint16_t remain_time;
    uint8_t bytes[sizeof(uint16_t)]; // ??float??????
};
union game_time computition_type;

union money_remain {
	uint16_t money;
    uint8_t bytes[sizeof(uint16_t)]; // ??float??????
};
union money_remain money_game;


union game_time computition_type;


union friendplace {
    struct {
        float place_x;
		float place_y;
    } places;
    uint8_t bytes[sizeof(float) * 2]; // ??float??????
};
union friendplace infantry_1_place,infantry_2_place,hero_place,engineer_place,sentry_place;
union friendHP_red {
    struct {
		uint16_t hp;
    } shengming;
    uint8_t bytes[sizeof(uint16_t)]; // ??float??????
};

union friendHP_red infantry_1_hp_red,infantry_2_hp_red,infantry_3_hp_red,hero_hp_red,engineer_hp_red,sentry_hp_red,outpost_red_hp,base_red_hp;

union friendHP_blue {
    struct {
		uint16_t hp;
    } shengming;
    uint8_t bytes[sizeof(uint16_t)]; // ??float??????
};

union friendHP_blue infantry_1_hp_blue,infantry_2_hp_blue,infantry_3q_hp_blue,hero_hp_blue,engineer_hp_blue,sentry_hp_blue,outpost_blue_hp,base_blue_hp;

uint16_t count_i=0;

union shoot_17 {
	uint16_t shoot_remain;
    uint8_t bytes[sizeof(uint16_t)]; // ??float??????
};
union shoot_17 remain_zidan;

union shoot_heat {
	uint16_t heat;
    uint8_t bytes[sizeof(uint16_t)]; // ??float??????
};
union shoot_heat heat_2006,limit_17_2006;


uint16_t total_heat=0;

extern uint8_t heat_flag;

void can_filter_init(void)//?????
{
	 CAN_FilterTypeDef can1_filter_st,can2_filter_st;
	
	can1_filter_st.FilterIdHigh = 0x0000;
	can1_filter_st.FilterIdLow = 0x0000;
	can1_filter_st.FilterMaskIdHigh = 0x0000;
	can1_filter_st.FilterMaskIdLow = 0x0000;
	can1_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	can1_filter_st.FilterActivation = ENABLE;
	can1_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
	can1_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
	can1_filter_st.FilterBank = 0;
	can1_filter_st.SlaveStartFilterBank = 14;
	
	can2_filter_st.FilterIdHigh = 0x0000;
	can2_filter_st.FilterIdLow = 0x0000;
	can2_filter_st.FilterMaskIdHigh = 0x0000;
	can2_filter_st.FilterMaskIdLow = 0x0000;
	can2_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	can2_filter_st.FilterActivation = ENABLE;
	can2_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
	can2_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
	can2_filter_st.FilterBank = 0;
		can2_filter_st.SlaveStartFilterBank = 0;
		//使能CAN通道
	HAL_CAN_ConfigFilter(&hcan1, &can1_filter_st);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan1);
	HAL_Delay(10);
	HAL_CAN_ConfigFilter(&hcan2, &can2_filter_st);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan2);
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//FIFO0和FIFO1要注意，这里是接收的中断回调函数
{                                                    //FIFO0一般简单一些 FIFO1需要更多的函数
  CAN_RxHeaderTypeDef rx_header;
  uint8_t             rx_data[8];

	if(hcan->Instance == CAN1)
  {
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //把数据放入rx_data[8]里面//FIFO是缓存区类似于电脑的缓存，这里的函数是从其中读取数据
	  switch(rx_header.StdId)                  //最后两个位置都是指针，rx_header指向接收消息的头部，rx_data指向缓冲区
	{
	case 0x201:		
		motor_3508_shoot[0].rotor_angle  = ((rx_data[0] << 8) | rx_data[1]);
		motor_3508_shoot[0].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
		motor_3508_shoot[0].torque_current = ((rx_data[4] << 8) | rx_data[5]);   //解开数据
		motor_3508_shoot[0].temp          =   rx_data[6];
		break;
	case 0x202:
		motor_3508_shoot[1].rotor_angle  = ((rx_data[0] << 8) | rx_data[1]);
		motor_3508_shoot[1].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
		motor_3508_shoot[1].torque_current = ((rx_data[4] << 8) | rx_data[5]);   //解开数据
		motor_3508_shoot[1].temp          =   rx_data[6];
		break;
	case 0x203:
		{
			motor_2006_shoot.rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);//接收机械角度（16bit）
			motor_2006_shoot.rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);//接收转速（16bit）
			motor_2006_shoot.torque_current = ((rx_data[4] << 8) | rx_data[5]);//接收实际转矩
		break;
	}
	case 0x01:{
		Motor_DM4310_receive(&DM4310_pitch, rx_data, 0x01);
		break;
	}
	/*测试速度模式的4310*/
	case 0x02:
	{
		Motor_DM4310_receive(&DM4310_pitch, rx_data, 0x02);
		break;
	}
}
	}
  if(hcan->Instance == CAN2)
  {
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); 
	  switch(rx_header.StdId){
			case 0x11F:
	{
		yaw_6020_remote=((int16_t)rx_data[0] | ((int16_t)rx_data[1] << 8)) & 0x07FF; 
		yaw_6020_remote=yaw_6020_remote-1024;
		pitch_4310_remote=(((int16_t)rx_data[1] >> 3) | ((int16_t)rx_data[2] << 5)) & 0x07FF;
		pitch_4310_remote=pitch_4310_remote-1024;
		
		s[0]=((rx_data[3] >> 4) & 0x000C) >> 2;
		s[1]=((rx_data[3] >> 4) & 0x0003);
		wheel=(rx_data[4] | rx_data[5] << 8) - 1024;
		break;
	}
			case 0x141:
				
	{
		motor_9025_yaw.temp=rx_data[1];
		motor_9025_yaw.current=((rx_data[3]<<8)|rx_data[2]);
		motor_9025_yaw.speed=((rx_data[5]<<8)|rx_data[4]);
		motor_9025_yaw.encoder_place=((rx_data[7]<<8)|rx_data[6]);   //0-65535
		break;
	
	}
			case 0x207:
		{
			motor_yaw.rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);//接收机械角度（16bit）
			motor_yaw.rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);//接收转速（16bit）
			motor_yaw.torque_current = ((rx_data[4] << 8) | rx_data[5]);//接收实际转矩
			motor_yaw.temp           =   rx_data[6];//接收电机温度（8bit）
			
			
			if(motor_yaw.Set_Angle <1800||motor_yaw.Set_Angle >= 3500)
			{
				
			}
		break;
		}
		
		
		
		/********************裁判系统信息********************/
			
		
		/*********************位置********************/
			
			case 0x320:
		{
		infantry_1_place.bytes[0]	=rx_data[0] ;
		infantry_1_place.bytes[1]   =rx_data[1];
	
		infantry_1_place.bytes[2]=rx_data[2];
		infantry_1_place.bytes[3]=rx_data[3];
	
		infantry_1_place.bytes[4]=rx_data[4];
		infantry_1_place.bytes[5]=rx_data[5];
	
		infantry_1_place.bytes[6]=rx_data[6];
		infantry_1_place.bytes[7]=rx_data[7];
		break;
		}
		
			case 0x321:
		{
		engineer_place.bytes[0]=rx_data[0];
		engineer_place.bytes[1]=rx_data[1];
	
		engineer_place.bytes[2]=rx_data[2];
		engineer_place.bytes[3]=rx_data[3];
	
		engineer_place.bytes[4]=rx_data[4];
		engineer_place.bytes[5]=rx_data[5];
	
		engineer_place.bytes[6]=rx_data[6];
		engineer_place.bytes[7]=rx_data[7];
		break;
		}
			case 0x322:
		{
		infantry_2_place.bytes[0]=rx_data[0];
		infantry_2_place.bytes[1]=rx_data[1];
	
		infantry_2_place.bytes[2]=rx_data[2];
		infantry_2_place.bytes[3]=rx_data[3];
	
		infantry_2_place.bytes[4]=rx_data[4];
		infantry_2_place.bytes[5]=rx_data[5];
	
		infantry_2_place.bytes[6]=rx_data[6];
		infantry_2_place.bytes[7]=rx_data[7];
		break;
		}
			case 0x323:
		{
		hero_place.bytes[0]=rx_data[0];
		hero_place.bytes[1]=rx_data[1];
	
		hero_place.bytes[2]=rx_data[2];
		hero_place.bytes[3]=rx_data[3];
	
		hero_place.bytes[4]=rx_data[4];
		hero_place.bytes[5]=rx_data[5];

		hero_place.bytes[6]=rx_data[6];
		hero_place.bytes[7]=rx_data[7];
		break;
		}
			case 0x324:
		{
		sentry_place.bytes[0]=rx_data[0];
		sentry_place.bytes[1]=rx_data[1];
	
		sentry_place.bytes[2]=rx_data[2];
		sentry_place.bytes[3]=rx_data[3];
	
		sentry_place.bytes[4]=rx_data[4];
		sentry_place.bytes[5]=rx_data[5];

		sentry_place.bytes[6]=rx_data[6];
		sentry_place.bytes[7]=rx_data[7];
		break;
		}
/*********************************HP血量*************************************/			
			case 0x330:
		{
		hero_hp_red.bytes[0]=rx_data[0];
		hero_hp_red.bytes[1]=rx_data[1];
	
		engineer_hp_red.bytes[0]=rx_data[2];
		engineer_hp_red.bytes[1]=rx_data[3];
	
		infantry_1_hp_red.bytes[0]=rx_data[4];
		infantry_1_hp_red.bytes[1]=rx_data[5];
	
		infantry_2_hp_red.bytes[0]=rx_data[6];
		infantry_2_hp_red.bytes[1]=rx_data[7];
		break;
		}
			case 0x331:
		{
sentry_hp_red.bytes[0]=rx_data[0];
sentry_hp_red.bytes[1]=rx_data[1];

outpost_red_hp.bytes[0]=rx_data[2];
outpost_red_hp.bytes[1]=rx_data[3];

base_red_hp.bytes[0]=rx_data[4];
base_red_hp.bytes[1]=rx_data[5];

//rx_data[6] = 0;
//rx_data[7] = 0;
		break;
		}
		
			case 0x332:
		{
hero_hp_blue.bytes[0]=rx_data[0] ;
hero_hp_blue.bytes[1]=rx_data[1] ;

engineer_hp_blue.bytes[0]=rx_data[2] ;
engineer_hp_blue.bytes[1]=rx_data[3] ;

infantry_1_hp_blue.bytes[0]=rx_data[4] ;
infantry_1_hp_blue.bytes[1]=rx_data[5] ;

infantry_2_hp_blue.bytes[0]=rx_data[6] ;
infantry_2_hp_blue.bytes[1]=rx_data[7] ;
		break;
		}
			case 0x333:
		{
sentry_hp_blue.bytes[0]=rx_data[0] ;
sentry_hp_blue.bytes[1]=rx_data[1];

outpost_blue_hp.bytes[0]=rx_data[2];
outpost_blue_hp.bytes[1]=rx_data[3];

base_blue_hp.bytes[0]=rx_data[4];
base_blue_hp.bytes[1]=rx_data[5];

rx_data[6] = 0;
rx_data[7] = 0;
		break;
		}	
			case 0x350:
		{
			
progress=rx_data[0];
computition_type.bytes[0]=rx_data[1];

computition_type.bytes[1]=rx_data[2];
money_game.bytes[0]=rx_data[3];
money_game.bytes[1]=rx_data[4];
		rx_data[5] = 0;

		rx_data[6] = 0;

		rx_data[7] = 0;
			break;
		}
		case 0x360:
		{
			
	target_robot.bytes[0]=rx_data[0];
    target_robot.bytes[1]=rx_data[1];
	
	target_robot.bytes[2]=rx_data[2];
	target_robot.bytes[3]=rx_data[3];
	
	target_robot.bytes[4]=rx_data[4];
	target_robot.bytes[5]=rx_data[5];
	
	target_robot.bytes[7]=rx_data[6];
	target_robot.bytes[7]=rx_data[7];

			break;
		}
		
		case 0x361:
		{
			
	commd_keyboard=rx_data[0];
    target_robot_ID=rx_data[1];
			break;
		}
		case 0x370:
		{
			
	shouji=rx_data[0];

			break;
		}
		case 0x371:
		{
			remain_zidan.bytes[0]=rx_data[0];
			remain_zidan.bytes[1]=rx_data[1];
			break;
		}

		case 0x372:
		{
			heat_2006.bytes[0]=rx_data[0];
			heat_2006.bytes[1]=rx_data[1];
			limit_17_2006.bytes[0]=rx_data[2];
			limit_17_2006.bytes[1]=rx_data[3];
			//total_heat=heat_2006.heat;
			if(heat_2006.heat>limit_17_2006.heat-20){
				heat_flag=0;
			}
			if(heat_2006.heat<limit_17_2006.heat-100){
				heat_flag=1;
			}
			break;
		}
			
}
 }

}
void vision_transmit_friend1_place (uint8_t *buff)
{
	
		//YAW_Vision=(motor_yaw.rotor_angle-700)/8191*2* 3.1415926;
		memset(vision_transmit_buff,0,sizeof(vision_transmit_buff));  //把vision这个数组设置为0;
	
		vision_transmit_buff[0] = 's';	//起始位
		vision_transmit_buff[1] = 0xB1; //步兵的位置
		//memcpy(&(vision_transmit_buff[2]),, 4);	//
		memcpy(&(vision_transmit_buff[2]),&infantry_1_place.places.place_x, 4);	//
		memcpy(&(vision_transmit_buff[6]),&infantry_1_place.places.place_y, 4);
		memcpy(&(vision_transmit_buff[10]),&infantry_2_place.places.place_x, 4);	//
		memcpy(&(vision_transmit_buff[14]),&infantry_2_place.places.place_y, 4);	
		vision_transmit_buff[31] = 'e'; //结束位
}

void vision_transmit_friend2_place (uint8_t *buff)
{
	
		//YAW_Vision=(motor_yaw.rotor_angle-700)/8191*2* 3.1415926;
		memset(vision_transmit_buff,0,sizeof(vision_transmit_buff));  //把vision这个数组设置为0;
	
		vision_transmit_buff[0] = 's';	//起始位
		vision_transmit_buff[1] = 0xB2; //步兵的位置
		//memcpy(&(vision_transmit_buff[2]),, 4);	//
		memcpy(&(vision_transmit_buff[2]),&hero_place.places.place_x, 4);	//
		memcpy(&(vision_transmit_buff[6]),&hero_place.places.place_y, 4);
		memcpy(&(vision_transmit_buff[10]),&engineer_place.places.place_x, 4);	//
		memcpy(&(vision_transmit_buff[14]),&engineer_place.places.place_y, 4);	
		memcpy(&(vision_transmit_buff[18]),&sentry_place.places.place_x, 4);	//
		memcpy(&(vision_transmit_buff[22]),&sentry_place.places.place_y, 4);
		vision_transmit_buff[31] = 'e'; //结束位
}

void vision_transmit_red_hp (uint8_t *buff)
{
	//红方血量
		//YAW_Vision=(motor_yaw.rotor_angle-700)/8191*2* 3.1415926;
		memset(vision_transmit_buff,0,sizeof(vision_transmit_buff));  //把vision这个数组设置为0;
	
		vision_transmit_buff[0] = 's';	//起始位
		vision_transmit_buff[1] = 0xB5; //步兵的位置
		//memcpy(&(vision_transmit_buff[2]),, 4);	//
		memcpy(&(vision_transmit_buff[2]),&hero_hp_red.shengming.hp, 2);	//
		memcpy(&(vision_transmit_buff[4]),&engineer_hp_red, 2);
		memcpy(&(vision_transmit_buff[6]),&infantry_1_hp_red, 2);	//
		memcpy(&(vision_transmit_buff[8]),&infantry_2_hp_red, 2);	
		//memcpy(&(vision_transmit_buff[10]),&infantry_2_place.places.place_y, 2);	
		memcpy(&(vision_transmit_buff[12]),&sentry_hp_red.shengming.hp, 2);	
		vision_transmit_buff[31] = 'e'; //结束位
}

void vision_transmit_blue_hp (uint8_t *buff)
{
	//蓝方血量
		//YAW_Vision=(motor_yaw.rotor_angle-700)/8191*2* 3.1415926;
		memset(vision_transmit_buff,0,sizeof(vision_transmit_buff));  //把vision这个数组设置为0;
	
		vision_transmit_buff[0] = 's';	//起始位
		vision_transmit_buff[1] = 0xB6; //步兵的位置
		//memcpy(&(vision_transmit_buff[2]),, 4);	//
		memcpy(&(vision_transmit_buff[2]),&hero_hp_blue.shengming.hp, 2);	//
		memcpy(&(vision_transmit_buff[4]),&engineer_hp_blue, 2);
		memcpy(&(vision_transmit_buff[6]),&infantry_1_hp_blue, 2);	//
		memcpy(&(vision_transmit_buff[8]),&infantry_2_hp_blue, 2);	
		//memcpy(&(vision_transmit_buff[10]),&infantry_2_place.places.place_y, 2);	
		memcpy(&(vision_transmit_buff[12]),&sentry_hp_blue.shengming, 2);	
		vision_transmit_buff[31] = 'e'; //结束位
}
/*********************建筑血量******************/
void vision_transmit_building_hp (uint8_t *buff)
{
	
		//YAW_Vision=(motor_yaw.rotor_angle-700)/8191*2* 3.1415926;
		memset(vision_transmit_buff,0,sizeof(vision_transmit_buff));  //把vision这个数组设置为0;
	
		vision_transmit_buff[0] = 's';	//起始位
		vision_transmit_buff[1] = 0xB7; //步兵的位置
		//memcpy(&(vision_transmit_buff[2]),, 4);	//
		memcpy(&(vision_transmit_buff[2]),&outpost_red_hp.shengming.hp, 2);	//
		memcpy(&(vision_transmit_buff[4]),&base_red_hp.shengming.hp, 2);
		memcpy(&(vision_transmit_buff[6]),&outpost_blue_hp.shengming.hp, 2);	//
		memcpy(&(vision_transmit_buff[8]),&base_blue_hp.shengming.hp, 2);	
		vision_transmit_buff[31] = 'e'; //结束位
}
/****************比赛信息****************/
void vision_transmit_game_state (uint8_t *buff)
{
	
		//YAW_Vision=(motor_yaw.rotor_angle-700)/8191*2* 3.1415926;
		memset(vision_transmit_buff,0,sizeof(vision_transmit_buff));  //把vision这个数组设置为0;
	
		vision_transmit_buff[0] = 's';	//起始位
		vision_transmit_buff[1] = 0xB8; //步兵的位置
		//memcpy(&(vision_transmit_buff[2]),, 4);	//
		//memcpy(&(vision_transmit_buff[2]),&outpost_red_hp.shengming.hp, 1);	//
		memcpy(&(vision_transmit_buff[3]),&progress, 1);
		memcpy(&(vision_transmit_buff[4]),&computition_type.remain_time, 2);	//
		memcpy(&(vision_transmit_buff[6]),&money_game.money, 2);	
		vision_transmit_buff[31] = 'e'; //结束位
}
/**********************操作反馈**********************/
void vision_transmit_exercise (uint8_t *buff)
{
	
		//YAW_Vision=(motor_yaw.rotor_angle-700)/8191*2* 3.1415926;
		memset(vision_transmit_buff,0,sizeof(vision_transmit_buff));  //把vision这个数组设置为0;
	
		vision_transmit_buff[0] = 's';	//起始位
		vision_transmit_buff[1] = 0xB9; //步兵的位置
		//memcpy(&(vision_transmit_buff[2]),, 4);	//
		memcpy(&(vision_transmit_buff[2]),&target_robot.places.place_x, 4);	//
		memcpy(&(vision_transmit_buff[6]),&target_robot.places.place_y, 4);
	
		memcpy(&(vision_transmit_buff[10]),&commd_keyboard, 1);	//
		memcpy(&(vision_transmit_buff[11]),&target_robot_ID, 1);	
		vision_transmit_buff[31] = 'e'; //结束位
}

/**********************受击反馈*********************/
void vision_transmit_shouji (uint8_t *buff)
{
	
		//YAW_Vision=(motor_yaw.rotor_angle-700)/8191*2* 3.1415926;
		memset(vision_transmit_buff,0,sizeof(vision_transmit_buff));  //把vision这个数组设置为0;
	
		vision_transmit_buff[0] = 's';	//起始位
		vision_transmit_buff[1] = 0xBA; //步兵的位置
	if(shouji==1){
		hit_count=1;
	}
	if(hit_count==1){
		shouji=1;
		count_i++;
	}
		memcpy(&(vision_transmit_buff[2]),&shouji, 1);	//
	//memcpy(&(vision_transmit_buff[2]),&hit_flag, 1);
	if(count_i>10){
		hit_count=0;
	}
		vision_transmit_buff[31] = 'e'; //结束位
}

/**********************发射状态*********************/
void vision_transmit_shoot (uint8_t *buff)
{
	
		//YAW_Vision=(motor_yaw.rotor_angle-700)/8191*2* 3.1415926;
		memset(vision_transmit_buff,0,sizeof(vision_transmit_buff));  //把vision这个数组设置为0;
	
		vision_transmit_buff[0] = 's';	//起始位
		vision_transmit_buff[1] = 0xBB; //步兵的位置
		//memcpy(&(vision_transmit_buff[2]),, 4);	//
		memcpy(&(vision_transmit_buff[2]),&remain_zidan.shoot_remain, 2);	//
		vision_transmit_buff[31] = 'e'; //结束位
}