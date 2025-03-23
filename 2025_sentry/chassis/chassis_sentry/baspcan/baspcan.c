#include "baspcan.h"
#include "main.h"
#include "can.h"
#include "math.h"
#include "cmsis_os.h"
#include "../IMU/INS_task.h"
#include "motor_define.h"
#include "LK_9025.h"
//先设置一个滤波器

extern motor_3508_t motor_3508[4];
extern motor_6020_t motor_yaw,motor_pitch;
extern motor_9025 motor_9025_yaw;

extern Chassis_Speed_t Chassis_Speed;


extern float Auto_speed_x,Auto_speed_y;

extern float zimiao_yaw;

uint8_t xiaotuoluo_flag=0;
uint8_t kkkkk=0;

union TwoFloatsToBytes {
    struct {
        float floatValue1;
        float floatValue2;
    } floats;
    uint8_t bytes[sizeof(float) * 2]; // ??float??????
};

union FloatToBytes {
    struct {
        float yawValue1;
    } yaws;
    uint8_t bytes[sizeof(float)]; // ??float??????
};
union IMU_ToBytes {
    struct {
        float yawValue1;
    } yaws;
    uint8_t bytes[sizeof(float)]; // ??float??????
};
union TwoFloatsToBytes converter_rx;
union FloatToBytes zimiao_rx;
union IMU_ToBytes IMU_9025_YAW;


uint8_t fuhuo_flag=0;
uint8_t zidan_much=0;

float imu_yaw_angle;

union yawToBytes {
	float angle;
    uint8_t bytes[sizeof(float)]; // ??float??????
}yaw_9025_anglee;

float low_yaw_angle=0;

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
	can1_filter_st.SlaveStartFilterBank = 14;
    can1_filter_st.FilterBank = 0;
	
    can2_filter_st.FilterIdHigh = 0x0000;
    can2_filter_st.FilterIdLow = 0x0000;
    can2_filter_st.FilterMaskIdHigh = 0x0000;
    can2_filter_st.FilterMaskIdLow = 0x0000;
    can2_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    can2_filter_st.FilterActivation = ENABLE;
    can2_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can2_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
	can2_filter_st.SlaveStartFilterBank = 14;
    can2_filter_st.FilterBank = 14;
		//使能CAN通道
    HAL_CAN_ConfigFilter(&hcan1, &can1_filter_st);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_Start(&hcan1);
		
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
	  case 0x201:    //ID为0x204+电机ID
	{
		motor_3508[0].rotor_angle  = ((rx_data[0] << 8) | rx_data[1]);
		motor_3508[0].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
		motor_3508[0].torque_current = ((rx_data[4] << 8) | rx_data[5]);   //解开数据
		motor_3508[0].temp          =   rx_data[6];
		break;
		
	}
	  case 0x202:
		{
		motor_3508[1].rotor_angle  = ((rx_data[0] << 8) | rx_data[1]);
		motor_3508[1].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
		motor_3508[1].torque_current = ((rx_data[4] << 8) | rx_data[5]);   //解开数据
		motor_3508[1].temp          =   rx_data[6];
		break;
	}
	case 0x203:
		{
		motor_3508[2].rotor_angle  = ((rx_data[0] << 8) | rx_data[1]);
		motor_3508[2].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
		motor_3508[2].torque_current = ((rx_data[4] << 8) | rx_data[5]);   //解开数据
		motor_3508[2].temp          =   rx_data[6];
		break;
	}
	case 0x204:
		{
		motor_3508[3].rotor_angle  = ((rx_data[0] << 8) | rx_data[1]);
		motor_3508[3].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
		motor_3508[3].torque_current = ((rx_data[4] << 8) | rx_data[5]);   //解开数据
		motor_3508[3].temp          =   rx_data[6];
		break;
	}
		
	}
	}
  if(hcan->Instance == CAN2){
	  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //把数据放入rx_data[8]里面//FIFO是缓存区类似于电脑的缓存，这里的函数是从其中读取数据
  switch(rx_header.StdId){
	  	case 0x141:
	{
		motor_9025_yaw.temp=rx_data[1];
		motor_9025_yaw.current=((rx_data[3]<<8)|rx_data[2]);
		motor_9025_yaw.speed=((rx_data[5]<<8)|rx_data[4]);
		motor_9025_yaw.encoder_place=((rx_data[7]<<8)|rx_data[6]);   //0-65535
		motor_9025_yaw.now_speed=motor_9025_yaw.speed*100;
		break;
	}
//		case 0x308:{
//			yaw_9025_anglee.bytes[0]=rx_data[0];
//			yaw_9025_anglee.bytes[1]=rx_data[1];
//			yaw_9025_anglee.bytes[2]=rx_data[2];
//			yaw_9025_anglee.bytes[3]=rx_data[3];
//			
//			low_yaw_angle=yaw_9025_anglee.angle;
//			
//		}
		case 0x309:{
			converter_rx.bytes[0]=rx_data[0];
			converter_rx.bytes[1]=rx_data[1];
			converter_rx.bytes[2]=rx_data[2];
			converter_rx.bytes[3]=rx_data[3];
			
			converter_rx.bytes[4]=rx_data[4];
			converter_rx.bytes[5]=rx_data[5];
			converter_rx.bytes[6]=rx_data[6];
			converter_rx.bytes[7]=rx_data[7];
//			memcpy(&Auto_speed_x,rx_data,sizeof(float));
//		
//			memcpy(&Auto_speed_y,rx_data+sizeof(float),sizeof(float));
			Auto_speed_x=converter_rx.floats.floatValue1;
			Auto_speed_y=converter_rx.floats.floatValue2;
			break;
		}
		
		case 0x310:{
			zimiao_rx.bytes[0]=rx_data[0];
			zimiao_rx.bytes[1]=rx_data[1];
			zimiao_rx.bytes[2]=rx_data[2];
			zimiao_rx.bytes[3]=rx_data[3];
//			memcpy(&Auto_speed_x,rx_data,sizeof(float));
//		
//			memcpy(&Auto_speed_y,rx_data+sizeof(float),sizeof(float));
			 zimiao_yaw=zimiao_rx.yaws.yawValue1;
			break;
		}
		case 0x404:{
			IMU_9025_YAW.bytes[0]=rx_data[0];
			IMU_9025_YAW.bytes[1]=rx_data[1];
			IMU_9025_YAW.bytes[2]=rx_data[2];
			IMU_9025_YAW.bytes[3]=rx_data[3];
			imu_yaw_angle=IMU_9025_YAW.yaws.yawValue1;
			break;
		}
		case 0x187:{
			xiaotuoluo_flag=rx_data[0];
			fuhuo_flag=rx_data[1];
			zidan_much=rx_data[2];
			//kkkkk=rx_data[7];
			break;
		}
  }
}
 }