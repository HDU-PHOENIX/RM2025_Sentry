#include "baspcan.h"
#include "main.h"
#include "can.h"
#include "math.h"
#include "cmsis_os.h"
#include "../IMU/INS_task.h"
#include "motor_define.h"
#include "DM4310.h"
#include "test.h"
#include "vision.h"
#include "DM4310.h"
#include "LK_9025.h"
#include "shoot.h"

extern computer_all computer_from;

uint8_t xiaotuoluo_flag=0;

uint16_t angle_more=0;
extern float canshu_yaw_zimiao;
extern uint8_t s[2];

uint8_t             xiaotuoluo_tx_data[8];


extern motor_3508_t motor_3508_shoot[2];
extern motor_2006_t motor_2006_shoot;

void test(void const * argument)
{
	portTickType currentTime;
	uint16_t Task_Time=0;
	uint8_t tick=0;
	for(;;)
    {
		currentTime = xTaskGetTickCount();
		vision_transmit_judge();
		Motor_DM4310_Enable_position(&hcan1,0x01);
		Set_M3508_Shoot_Voltage(&hcan1,motor_3508_shoot,motor_2006_shoot);
		xiaotuoluo_flag=computer_from.computer_exercise.type;
								CAN_TxHeaderTypeDef tx_header;
					
						tx_header.StdId = 0x187;//标识符（见手册P6）  //
						tx_header.IDE   = CAN_ID_STD;//标准ID
						tx_header.RTR   = CAN_RTR_DATA;//数据帧
						tx_header.DLC   = 8;//字节长度
				
				
				xiaotuoluo_tx_data[0]=xiaotuoluo_flag;
				xiaotuoluo_tx_data[1]=computer_from.computer_juece.type;
				xiaotuoluo_tx_data[2]=computer_from.computer_juece.content;
				xiaotuoluo_tx_data[3]=0;
				xiaotuoluo_tx_data[4]=0;
				xiaotuoluo_tx_data[5]=0;
				xiaotuoluo_tx_data[6]=0;
				xiaotuoluo_tx_data[7]=0;
				
						HAL_CAN_AddTxMessage(&hcan2, &tx_header, xiaotuoluo_tx_data,(uint32_t*)CAN_TX_MAILBOX0);
		osDelay(100);
        vTaskDelayUntil(&currentTime,2);//绝对延时
    }
}