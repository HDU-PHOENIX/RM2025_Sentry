#include "main.h"
#include "usbd_cdc_if.h"
#include "vision.h"
#include "motor_define.h"
#include "math.h"
#include "cmsis_os.h"
#include "DM4310.h"
#include "can.h"
#include "LK_9025.h"
#include "baspcan.h"
#include "INS_task.h"


//extern union target_place target_robot;
//extern union game_time computition_type;
//extern union money_remain money_game;
//extern union game_time computition_type;
//extern union friendplace infantry_1_place,infantry_2_place,hero_place,engineer_place;
//extern union friendHP_red infantry_1_hp_red,infantry_2_hp_red,infantry_3_hp_red,hero_hp_red,engineer_hp_red,sentry_hp_red,outpost_red_hp,base_red_hp;
//extern union friendHP_blue infantry_1_hp_blue,infantry_2_hp_blue,infantry_3q_hp_blue,hero_hp_blue,engineer_hp_blue,sentry_hp_blue,outpost_blue_hp,base_blue_hp;


int check_ok=0;
computer_all computer_from;

uint8_t vision_buf[32];
uint8_t vision_transmit_buff[33] = {"s000000000000000000000000000000e"};
//int8_t CDC_Receive_FS
float YAW_Vision,PITCH_Vision;
uint16_t Task_Time=0;
extern motor_6020_t motor_yaw;

extern motor_9025 motor_9025_yaw;

float yaw_9025_vision=0;

extern fp32 IMU_angle[3];
portTickType ulzimiaoLostTime = 0;

uint16_t count_lost=0;

extern 	float yaw_imu;

float shangweiji_imu=0;

int hit_flag=0;

float world_9025_yaw_angle=0;

int iii=0;
uint32_t zimiao_ulGetLostTime( void )
{
    /* */
    return  ulzimiaoLostTime;
}

union TwoFloatsToBytes {
    struct {
        float floatValue1;
        float floatValue2;
    } floats;
    uint8_t bytes[sizeof(float) * 2]; // ??float??????
};

union yawToBytes {
	float angle;
    uint8_t bytes[sizeof(float)]; // ??float??????
}yaw_9025_anglee;

void vision_call_back_handler(uint8_t *buff)//对视觉接收数据进行处理
{
	uint8_t *x_speed,*y_speed;
	union TwoFloatsToBytes converter;
    if( buff[0]=='s' && buff[31]=='e' )
    {
        if(buff[1] == 0xA0) //自瞄控制
        {
            check_ok++;  //接收正确计数
            //vision_missing_time = 0;  //清零失联时间
					        
						memcpy(&(computer_from.zimiao.fine_bool),&buff[2],1);
						memcpy(&(computer_from.zimiao.big_yaw),&buff[3],4);
						memcpy(&(computer_from.zimiao.pitch),&buff[7],4);				 //本来试图使用pid的error计算一样的方法，但是发现速度太快二者几乎一样不变
						memcpy(&(computer_from.zimiao.xiao_yaw),&buff[11],4);
			//computer_from.zimiao.big_yaw=computer_from.zimiao.big_yaw/6.28318*360;
			
			
			if(computer_from.zimiao.fine_bool==0){
				count_lost++;
			}
			
			
			
//			CAN_TxHeaderTypeDef tx_header;
//						uint8_t             tx_data[8];
//						
//						memcpy(tx_data,&computer_from.zimiao.yaw,sizeof(float));  //前四位传送的是x方向的速度
//						
//						//memcpy(tx_data+sizeof(float),&computer_from.computer_vpm.y_speed,sizeof(float)); //后四位传的是y方向的速度
//						
//						tx_header.StdId = 0x310;//标识符（见手册P6）  //
//						tx_header.IDE   = CAN_ID_STD;//标准ID
//						tx_header.RTR   = CAN_RTR_DATA;//数据帧
//						tx_header.DLC   = 8;//字节长度

//						HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);	
			
			
        }
		if(buff[1] == 0xA1) //底盘控制
        {
            check_ok++;  //接收正确计数
            //vision_missing_time = 0;  //清零失联时间
					        
						memcpy(&(computer_from.computer_vpm.x_speed),&buff[2],4);
						memcpy(&(computer_from.computer_vpm.y_speed),&buff[6],4);
			memcpy(&(world_9025_yaw_angle),&buff[10],4);
			//world_9025_yaw_angle
						//传输速度信息给底盘
			
			//osDelay(1);
			
				converter.floats.floatValue1 = computer_from.computer_vpm.x_speed;
				converter.floats.floatValue2 = computer_from.computer_vpm.y_speed;
				yaw_9025_anglee.angle=world_9025_yaw_angle;
			
						CAN_TxHeaderTypeDef tx_header;
						uint8_t             tx_data[8];
						
						memcpy(tx_data,&computer_from.computer_vpm.x_speed,sizeof(float));  //前四位传送的是x方向的速度
						
						memcpy(tx_data+sizeof(float),&computer_from.computer_vpm.y_speed,sizeof(float)); //后四位传的是y方向的速度
						
						tx_header.StdId = 0x309;//标识符（见手册P6）  //
						tx_header.IDE   = CAN_ID_STD;//标准ID
						tx_header.RTR   = CAN_RTR_DATA;//数据帧
						tx_header.DLC   = 8;//字节长度

						HAL_CAN_AddTxMessage(&hcan2, &tx_header, converter.bytes,(uint32_t*)CAN_TX_MAILBOX0);
						
//						tx_header.StdId = 0x308;//标识符（见手册P6）  //
//						tx_header.IDE   = CAN_ID_STD;//标准ID
//						tx_header.RTR   = CAN_RTR_DATA;//数据帧
//						tx_header.DLC   = 8;//字节长度
//						
//						
//						
//						HAL_CAN_AddTxMessage(&hcan2, &tx_header, yaw_9025_anglee.bytes,(uint32_t*)CAN_TX_MAILBOX0);
}
				
        }
		
		if(buff[1] == 0xA2) //比赛交互控制
        {
            check_ok++;  //接收正确计数
            //vision_missing_time = 0;  //清零失联时间
					        
						memcpy(&(computer_from.computer_juece.type),&buff[2],1);
						memcpy(&(computer_from.computer_juece.content),&buff[3],1);		
				
        }
		if(buff[1] == 0xA3) //比赛交互控制
        {
            check_ok++;  //接收正确计数
            //vision_missing_time = 0;  //清零失联时间
					        
						memcpy(&(computer_from.computer_exercise.type),&buff[2],1);
						memcpy(&(computer_from.computer_exercise.content),&buff[3],1);	



						if(computer_from.computer_exercise.type==1){
								iii=100;
						}
				
        }
    }
	/*测试*/	
void vision_transmit_package_AutoaimFeedback (uint8_t *buff)
{
//0.63
	/**/
		PITCH_Vision = DM4310_pitch.angle;
		YAW_Vision=(motor_yaw.rotor_angle-2674)/8191.0f*2* 3.1415926;
	yaw_9025_vision=0;
	yaw_9025_vision=(motor_9025_yaw.encoder_place-64624.79f)/65535.0f*2*3.1415926;
		memset(vision_transmit_buff,0,sizeof(vision_transmit_buff));  //把vision这个数组设置为0;
	
	shangweiji_imu=yaw_imu/360.0f*6.28;
	
	
		vision_transmit_buff[0] = 's';	//起始位
		vision_transmit_buff[1] = 0xB0; //自瞄信息
		memcpy(&(vision_transmit_buff[2]),&YAW_Vision, 4);	//yaw
		memcpy(&(vision_transmit_buff[6]),&PITCH_Vision, 4);	//pitch
	
	
	
	memcpy(&(vision_transmit_buff[13]),&shangweiji_imu, 4);

		vision_transmit_buff[31] = 'e'; //结束位
}

void vision_transmit(){		
	vision_transmit_package_AutoaimFeedback(vision_transmit_buff);
	CDC_Transmit_FS(vision_transmit_buff, 32);
	osDelay(1);
	//while(CDC_Transmit_FS(vision_transmit_buff, 32) != USBD_OK);
}

void vision_transmit_judge(){		
	vision_transmit_friend1_place (vision_transmit_buff);
	CDC_Transmit_FS(vision_transmit_buff, 32);
	osDelay(2);
	//while(CDC_Transmit_FS(vision_transmit_buff, 32) != USBD_OK);
	
	//vision_transmit_package_AutoaimFeedback(vision_transmit_buff);
	//CDC_Transmit_FS(vision_transmit_buff, 32);
	//osDelay(2);
	//while(CDC_Transmit_FS(vision_transmit_buff, 32) != USBD_OK);
	
	vision_transmit_friend2_place (vision_transmit_buff);
	CDC_Transmit_FS(vision_transmit_buff, 32);
	osDelay(2);
	//while(CDC_Transmit_FS(vision_transmit_buff, 32) != USBD_OK);
	
	vision_transmit_red_hp (vision_transmit_buff);
	CDC_Transmit_FS(vision_transmit_buff, 32);
	osDelay(2);
	//while(CDC_Transmit_FS(vision_transmit_buff, 32) != USBD_OK);
	
	vision_transmit_blue_hp (vision_transmit_buff);
	CDC_Transmit_FS(vision_transmit_buff, 32);
	osDelay(2);
	//while(CDC_Transmit_FS(vision_transmit_buff, 32) != USBD_OK);
	
	vision_transmit_building_hp (vision_transmit_buff);
	CDC_Transmit_FS(vision_transmit_buff, 32);
	osDelay(2);
	//while(CDC_Transmit_FS(vision_transmit_buff, 32) != USBD_OK);
	
	vision_transmit_game_state (vision_transmit_buff);
	CDC_Transmit_FS(vision_transmit_buff, 32);
	osDelay(2);
	//while(CDC_Transmit_FS(vision_transmit_buff, 32) != USBD_OK);
	
	vision_transmit_exercise (vision_transmit_buff);
	CDC_Transmit_FS(vision_transmit_buff, 32);
	osDelay(2);
	//while(CDC_Transmit_FS(vision_transmit_buff, 32) != USBD_OK);
	
	vision_transmit_shouji (vision_transmit_buff);
	CDC_Transmit_FS(vision_transmit_buff, 32);
	osDelay(2);
	//while(CDC_Transmit_FS(vision_transmit_buff, 32) != USBD_OK);
	
	vision_transmit_shoot (vision_transmit_buff);
	CDC_Transmit_FS(vision_transmit_buff, 32);
	osDelay(2);
	//while(CDC_Transmit_FS(vision_transmit_buff, 32) != USBD_OK);
}