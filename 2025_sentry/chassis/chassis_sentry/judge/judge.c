/**
  ******************************************************************************
  * @file    judge.c
  * @brief   解析裁判系统发来的数据，以获得机器人当前的状态数据。
  *
  ******************************************************************************
  * @attention
  *
  * 2021.3 已经完成了和tuxin.c的适配。
  *
  *
  *
  ******************************************************************************
  *///0x207

#include "judge.h"
#include "usart.h"
#include "crc.h"
#include "string.h"
#include "can.h"
#include "CRC_bro_init.h"
#include "can.h"
frame_t judge_frame_rx = {0};
frame_t judge_frame_tx;
//Student_Data receive_robot_interactive_data.data = {0};
uint8_t receive_student_data[113];
uint8_t judge_rx_buff[JUDGE_MAX_LENGTH];
//judge_show_data_t    Show_data = {0};//客户端信息
robot_type_t robot_type;

bool Judge_Data_TF = false;//裁判数据是否可用，辅助函数调用
uint8_t Judge_Self_ID;//当前机器人的ID
uint16_t Judge_SelfClient_ID;//发送者机器人对应的客户端ID
/**************裁判系统数据辅助****************/
uint16_t ShootNum=0;//统计发弹量,0x0003触发一次则认为发射了一颗
uint16_t Shoot2Num=0;
bool Hurt_Data_Update = false;//装甲板伤害数据是否更新,每受一次伤害置TRUE,然后立即置FALSE,给底盘闪避用
uint16_t Hurt_num = 0;
#define BLUE  0
#define RED   1
union infantry {
    struct {
        float place;
    } places;
    uint8_t bytes[sizeof(float)]; // ??float??????
};
union infantry infantryplace;


union bulletdata
{
		uint16_t bullet_speed_buff[2];
		float bullet_speed;
}BulletSpeed;
	
//union infantry {
//    struct {
//        float place;
//    } places;
//    uint8_t bytes[sizeof(float)]; // ??float??????
//};
//	
	
uint8_t yaw_flag_hit=0;	

uint16_t choose_client(uint8_t robot_id)
{
    uint16_t client_id;
    switch (robot_id)
    {
    case red_hero:
        client_id = red_hero_client;
        break;
    case red_engineer:
        client_id = red_engineer_client;
        break;
    case red_infantry_3:
        client_id = red_infantry_3_client;
        break;
    case red_infantry_4:
        client_id = red_infantry_4_client;
        break;
    case red_infantry_5:
        client_id = red_infantry_5_client;
        break;
    case red_aerial:
        client_id = red_aerial_client;
        break;

    case blue_hero:
        client_id = blue_hero_client;
        break;
    case blue_engineer:
        client_id = blue_engineer_client;
        break;
    case blue_infantry_3:
        client_id = blue_infantry_3_client;
        break;
    case blue_infantry_4:
        client_id = blue_infantry_4_client;
        break;
    case blue_infantry_5:
        client_id = blue_infantry_5_client;
        break;
    case blue_aerial:
        client_id = blue_aerial_client;
        break;
    default:
        break;
    }
    return client_id;
}

void Judge_Communication_Init(UART_HandleTypeDef* huart)
{
	/* 使能串口DMA */
	SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
	SET_BIT(huart->Instance->CR3, USART_CR3_DMAT);
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);//使能idle中断,用于裁判系统读取
  HAL_UART_Receive_DMA(huart, judge_rx_buff, 2000); //打开DMA接收，数据存入rx_buffer_judge数组中。
	/* 确保DMA TX失能 */
	while(huart->hdmatx->Instance->CR & DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(huart->hdmatx);
	}
	
	huart->hdmatx->Instance->PAR  = (uint32_t) & (huart->Instance->DR);
}

//heat
//从裁判系统读取数据
bool Judge_Read_Data(uint8_t *ReadFromUsart)   //裁判系统有两个口其中一个可能损坏
{
    bool retval_tf = false;//数据正确与否标志,每次调用读取裁判系统数据函数都先默认为错误

    //无数据包，则不作任何处理
    if (ReadFromUsart == NULL)
    {
        return -1;
    }
    //将帧头信息拷贝
    memcpy(&judge_frame_rx.frame_header, ReadFromUsart, LEN_HEADER);

    judge_frame_rx.frame_header.Data_Length = (ReadFromUsart[DATA_LENGTH + 1] << 8 ) | ReadFromUsart[DATA_LENGTH];
    //首先判断帧头
    if(judge_frame_rx.frame_header.SOF == Judge_Data_SOF)
    {
        //判断帧头CRC校验是否正确  
        if(Verify_CRC8_Check_Sum(ReadFromUsart, LEN_HEADER) == true)
        {
            //判断整包CRC校验是否正确
            if(Verify_CRC16_Check_Sum(ReadFromUsart, LEN_HEADER + LEN_CMDID + judge_frame_rx.frame_header.Data_Length + LEN_TAIL) == true)
            {
                retval_tf = true;//都校验过了则说明数据可用

                judge_frame_rx.cmd_id = (uint16_t)ReadFromUsart[6] << 8 | ReadFromUsart[5];
                //解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
                switch(judge_frame_rx.cmd_id)
                {

                case game_status_t:
                    memcpy(&judge_frame_rx.data.game_status, ReadFromUsart + DATA, len_game_status);
                    break ;

                case game_result_t:
                    memcpy(&judge_frame_rx.data.game_result, ReadFromUsart + DATA, len_game_result);
                    break;

                case game_robot_HP_t:
                    memcpy(&judge_frame_rx.data.game_robot_HP, ReadFromUsart + DATA, len_game_robot_HP);
                    break;

                case dart_status_t:
                    memcpy(&judge_frame_rx.data.dart_status, ReadFromUsart + DATA, len_dart_status);
                    break;

                case ICRA_buff_debuff_zone_status_t:
                    memcpy(&judge_frame_rx.data.ICRA_buff_debuff_zone_status, ReadFromUsart + DATA, len_ICRA_buff_debuff_zone_status);
                    break;

                case event_data_t:
                    memcpy(&judge_frame_rx.data.event_data, ReadFromUsart + DATA, len_event_data);
                    break;

                case supply_projectile_action_t:
                    memcpy(&judge_frame_rx.data.supply_projectile_action, ReadFromUsart + DATA, len_supply_projectile_action);
                    break;

                case referee_warning_t:
                    memcpy(&judge_frame_rx.data.referee_warning, ReadFromUsart + DATA, len_referee_warning);
                    break;

                case dart_remaining_time_t:
                    memcpy(&judge_frame_rx.data.dart_remaining_time, ReadFromUsart + DATA, len_dart_remaining_time);
                    break;

                case game_robot_status_t:
                    memcpy(&judge_frame_rx.data.game_robot_status, ReadFromUsart + DATA, len_game_robot_status);
                    Judge_Self_ID = judge_frame_rx.data.game_robot_status.robot_id ;
                    if(Judge_Self_ID < 100)
                    {
                        robot_type = red_robot;
                    }
                    else
                    {
                        robot_type = blue_robot;
                    }
										
//										set_judge_data(&hcan2,CANID_game_robot_status,((uint16_t)judge_frame_rx.data.game_robot_status.robot_id<<8)+judge_frame_rx.data.game_robot_status.shooter_id1_17mm_speed_limit,
//																													 judge_frame_rx.data.game_robot_status.remain_HP,
//																													 judge_frame_rx.data.game_robot_status.max_HP,
//																													 judge_frame_rx.data.game_robot_status.shooter_id1_17mm_cooling_limit);
//										if(judge_frame_rx.data.game_status.game_progress == 4)
//										{
//										set_judge_data(&hcan2,CANID_game_robot_status,((uint16_t)judge_frame_rx.data.game_robot_status.robot_id<<8)+judge_frame_rx.data.game_robot_status.shooter_id1_17mm_speed_limit,
//																													 judge_frame_rx.data.game_robot_status.remain_HP,
//																													 judge_frame_rx.data.game_status.stage_remain_time,
//																													 judge_frame_rx.data.game_robot_status.shooter_id1_17mm_cooling_limit);
//										}
//										else
//										{
//										set_judge_data(&hcan2,CANID_game_robot_status,((uint16_t)judge_frame_rx.data.game_robot_status.robot_id<<8)+judge_frame_rx.data.game_robot_status.shooter_id1_17mm_speed_limit,
//																													 judge_frame_rx.data.game_robot_status.remain_HP,
//																													 0,
//																													 judge_frame_rx.data.game_robot_status.shooter_id1_17mm_cooling_limit);
//										}
											
//										vision_send_pack();
                    break;
			//0x202
                case power_heat_data_t:
                    memcpy(&judge_frame_rx.data.power_heat_data, ReadFromUsart + DATA, len_power_heat_data);
//										set_judge_data(&hcan2,CANID_power_heat_data,judge_frame_rx.data.power_heat_data.shooter_id1_17mm_cooling_heat,judge_frame_rx.data.power_heat_data.chassis_power_buffer,0,0);
                    break;

                case game_robot_pos_t:
                    memcpy(&judge_frame_rx.data.game_robot_pos, ReadFromUsart + DATA, len_game_robot_pos);
                    break;

                case buff_t:
                    memcpy(&judge_frame_rx.data.buff, ReadFromUsart + DATA, len_buff);
                    break;

                case aerial_robot_energy_t:
                    memcpy(&judge_frame_rx.data.aerial_robot_energy, ReadFromUsart + DATA, len_aerial_robot_energy);
                    break;

                case robot_hurt_t:
                    memcpy(&judge_frame_rx.data.robot_hurt, ReadFromUsart + DATA, len_robot_hurt);
				yaw_flag_hit=1;
                    if(judge_frame_rx.data.robot_hurt.hurt_type == 0)//装甲伤害扣血
                    {
                        Hurt_Data_Update = true;	//装甲数据每更新一次则判定为受到一次伤害
                        Hurt_num++;
                    }
                    else
                    {
                        Hurt_Data_Update = false;
                    }
//							CAN_TxHeaderTypeDef tx_header;
//    uint8_t             tx_data[8] = {0};
					
					
					
//		tx_header.StdId = 0x370;//标识符（见手册P6）     
//    tx_header.IDE   = CAN_ID_STD;//标准ID
//    tx_header.RTR   = CAN_RTR_DATA;//数据帧
//    tx_header.DLC   = 8;//字节长度
//	if(judge_frame_rx.data.robot_hurt.hurt_type==0 && yaw_flag_hit==1){
//		tx_data[0] = 1;
//	}
//	else{
//		tx_data[0] = 0;
//	}
//	
//    tx_data[1] = 0;
//	
//	tx_data[2] = 0;
//	tx_data[3] = 0;
//	
//	tx_data[4] = 0;
//	tx_data[5] = 0;
//	
//	tx_data[6] = 0;
//	tx_data[7] = 0;
//	
//	yaw_flag_hit=0;


//    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX2);

	//osDelay(1);
					
					
					
					
					
					
					
                    break;

                case shoot_data_t:

                    memcpy(&judge_frame_rx.data.shoot_data, ReadFromUsart + DATA, len_shoot_data);
//                    JUDGE_ShootNumCount();//发弹量统计
										BulletSpeed.bullet_speed=judge_frame_rx.data.shoot_data.bullet_speed ;
//										set_judge_data(&hcan2,CANID_shoot_data,
//												((uint16_t)judge_frame_rx.data.shoot_data.bullet_type<<8)+judge_frame_rx.data.shoot_data.shooter_id,
//												judge_frame_rx.data.shoot_data.bullet_freq,
//												BulletSpeed.bullet_speed_buff[0],
//												BulletSpeed.bullet_speed_buff[1]);
								
                    break;

                case bullet_remaining_t:
                    memcpy(&judge_frame_rx.data.bullet_remaining, ReadFromUsart + DATA, len_bullet_remaining);
                    break;

                case rfid_status_t:
                    memcpy(&judge_frame_rx.data.rfid_status, ReadFromUsart + DATA, len_rfid_status);
                    break;

                case dart_client_cmd_t:
                    memcpy(&judge_frame_rx.data.dart_client_cmd, ReadFromUsart + DATA, len_dart_client_cmd);
                    break;
								/*学生通信*/
                case student_interactive_header_data_t:
						         memcpy(&judge_frame_rx.data.student_interactive_header_data, ReadFromUsart + DATA, len_student_interactive_header_data);
						         memcpy(&judge_frame_rx.data.robot_interactive_data, ReadFromUsart + STU_DATA, len_student_interactive_data);
                    break;
                case robot_interactive_data_t:
                    memcpy(&judge_frame_rx.data.robot_interactive_data, ReadFromUsart + DATA, len_robot_interactive_data);
                    break;
                case smallmap_communicate_t:
                    memcpy(&judge_frame_rx.data.smallmap_communicate, ReadFromUsart + STU_DATA, len_smallmap_communicate);
                    break;
                case robot_command_t:
                    memcpy(&judge_frame_rx.data.robot_command, ReadFromUsart + STU_DATA, len_robot_command);
                    break;
								case client_map_command_t:
                    memcpy(&judge_frame_rx.data.client_map_command, ReadFromUsart + STU_DATA, len_client_map_command);
                    break;
								case teammate_position_t:
									  memcpy(&judge_frame_rx.data.robot_position, ReadFromUsart + STU_DATA, len_teammate_msg);
								    break;
                default:
                    break;
                }
            }
        }
        //首地址加帧长度,指向CRC16下一字节,用来判断是否为0xA5,用来判断一个数据包是否有多帧数据
        if(*(ReadFromUsart + sizeof(frame_header_t) + LEN_CMDID + judge_frame_rx.frame_header.Data_Length + LEN_TAIL) == 0xA5)
        {
            //如果一个数据包出现了多帧数据,则再次读取
            Judge_Read_Data(ReadFromUsart + sizeof(frame_header_t) + LEN_CMDID + judge_frame_rx.frame_header.Data_Length + LEN_TAIL);
        }
    }
    if (retval_tf == true)
    {
        Judge_Data_TF = true;//辅助函数用
    }
    else		//只要CRC16校验不通过就为FALSE
    {
        Judge_Data_TF = false;//辅助函数用
    }
	/*敌方位置*/
	
	
	
//	/*******1号步兵发送 ******/
	

//    uint8_t             tx_data[8] = {0};
//    
//    tx_header.StdId = 0x11F;//标识符（见手册P6）
//    tx_header.IDE   = CAN_ID_STD;//标准ID
//    tx_header.RTR   = CAN_RTR_DATA;//数据帧
//    tx_header.DLC   = 8;//字节长度
//	
//	tx_data[0] = pData[0];
//    tx_data[1] = pData[1];
//	
//	tx_data[2] = pData[2];
//	tx_data[3]= pData[5];
//	
//	tx_data[4]=pData[16];
//	tx_data[5]=pData[17];

//    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
//	
//	/*******2号步兵发送 ******/
//		CAN_TxHeaderTypeDef tx_header;
//    uint8_t             tx_data[8] = {0};
//    
//    tx_header.StdId = 0x11F;//标识符（见手册P6）
//    tx_header.IDE   = CAN_ID_STD;//标准ID
//    tx_header.RTR   = CAN_RTR_DATA;//数据帧
//    tx_header.DLC   = 8;//字节长度
//	
//	tx_data[0] = pData[0];
//    tx_data[1] = pData[1];
//	
//	tx_data[2] = pData[2];
//	tx_data[3]= pData[5];
//	
//	tx_data[4]=pData[16];
//	tx_data[5]=pData[17];

//    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
//	
//	/*******工程发送 ******/
//		CAN_TxHeaderTypeDef tx_header;
//    uint8_t             tx_data[8] = {0};
//    
//    tx_header.StdId = 0x11F;//标识符（见手册P6）
//    tx_header.IDE   = CAN_ID_STD;//标准ID
//    tx_header.RTR   = CAN_RTR_DATA;//数据帧
//    tx_header.DLC   = 8;//字节长度
//	
//	tx_data[0] = pData[0];
//    tx_data[1] = pData[1];
//	
//	tx_data[2] = pData[2];
//	tx_data[3]= pData[5];
//	
//	tx_data[4]=pData[16];
//	tx_data[5]=pData[17];

//    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
//	
//	/****英雄****/
//		CAN_TxHeaderTypeDef tx_header;
//    uint8_t             tx_data[8] = {0};
//    
//    tx_header.StdId = 0x11F;//标识符（见手册P6）
//    tx_header.IDE   = CAN_ID_STD;//标准ID
//    tx_header.RTR   = CAN_RTR_DATA;//数据帧
//    tx_header.DLC   = 8;//字节长度
//	
//	tx_data[0] = pData[0];
//    tx_data[1] = pData[1];
//	
//	tx_data[2] = pData[2];
//	tx_data[3]= pData[5];
//	
//	tx_data[4]=pData[16];
//	tx_data[5]=pData[17];

//    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
//	*/

		/*我方位置*/
	
	
	
//	/*******1号步兵发送 ******/
//	CAN_TxHeaderTypeDef tx_header;
//    uint8_t             tx_data[8] = {0};
//    
//    tx_header.StdId = 0x11F;//标识符（见手册P6）
//    tx_header.IDE   = CAN_ID_STD;//标准ID
//    tx_header.RTR   = CAN_RTR_DATA;//数据帧
//    tx_header.DLC   = 8;//字节长度
//	
//	tx_data[0] = pData[0];
//    tx_data[1] = pData[1];
//	
//	tx_data[2] = pData[2];
//	tx_data[3]= pData[5];
//	
//	tx_data[4]=pData[16];
//	tx_data[5]=pData[17];

//    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
//	
//	/*******2号步兵发送 ******/
//		CAN_TxHeaderTypeDef tx_header;
//    uint8_t             tx_data[8] = {0};
//    
//    tx_header.StdId = 0x11F;//标识符（见手册P6）
//    tx_header.IDE   = CAN_ID_STD;//标准ID
//    tx_header.RTR   = CAN_RTR_DATA;//数据帧
//    tx_header.DLC   = 8;//字节长度
//	
//	tx_data[0] = pData[0];
//    tx_data[1] = pData[1];
//	
//	tx_data[2] = pData[2];
//	tx_data[3]= pData[5];
//	
//	tx_data[4]=pData[16];
//	tx_data[5]=pData[17];

//    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
//	
//	/*******工程发送 ******/
//		CAN_TxHeaderTypeDef tx_header;
//    uint8_t             tx_data[8] = {0};
//    
//    tx_header.StdId = 0x11F;//标识符（见手册P6）
//    tx_header.IDE   = CAN_ID_STD;//标准ID
//    tx_header.RTR   = CAN_RTR_DATA;//数据帧
//    tx_header.DLC   = 8;//字节长度
//	
//	tx_data[0] = pData[0];
//    tx_data[1] = pData[1];
//	
//	tx_data[2] = pData[2];
//	tx_data[3]= pData[5];
//	
//	tx_data[4]=pData[16];
//	tx_data[5]=pData[17];

//    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
//	
//	/****英雄****/
//		CAN_TxHeaderTypeDef tx_header;
//    uint8_t             tx_data[8] = {0};
//    
//    tx_header.StdId = 0x11F;//标识符（见手册P6）
//    tx_header.IDE   = CAN_ID_STD;//标准ID
//    tx_header.RTR   = CAN_RTR_DATA;//数据帧
//    tx_header.DLC   = 8;//字节长度
//	
//	tx_data[0] = pData[0];
//    tx_data[1] = pData[1];
//	
//	tx_data[2] = pData[2];
//	tx_data[3]= pData[5];
//	
//	tx_data[4]=pData[16];
//	tx_data[5]=pData[17];

//    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
//	*/

		/*红方血量*/
		
//	CAN_TxHeaderTypeDef tx_header;
//	uint8_t             tx_data[8] = {0};



//	tx_header.StdId = 0x11F;//标识符（见手册P6）
//	tx_header.IDE   = CAN_ID_STD;//标准ID
//	tx_header.RTR   = CAN_RTR_DATA;//数据帧
//	tx_header.DLC   = 8;//字节长度

//	tx_data[0] = pData[0];
//	tx_data[1] = pData[1];

//	tx_data[2] = pData[2];
//	tx_data[3]= pData[5];

//	tx_data[4]=pData[16];
//	tx_data[5]=pData[17];

//    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
		
		/*蓝方血量*/
		
		//		CAN_TxHeaderTypeDef tx_header;
//    uint8_t             tx_data[8] = {0};
//    
//    tx_header.StdId = 0x11F;//标识符（见手册P6）
//    tx_header.IDE   = CAN_ID_STD;//标准ID
//    tx_header.RTR   = CAN_RTR_DATA;//数据帧
//    tx_header.DLC   = 8;//字节长度
//	
//	tx_data[0] = pData[0];
//    tx_data[1] = pData[1];
//	
//	tx_data[2] = pData[2];
//	tx_data[3]= pData[5];
//	
//	tx_data[4]=pData[16];
//	tx_data[5]=pData[17];

//    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);



		/*建筑血量*/
		
		//		CAN_TxHeaderTypeDef tx_header;
//    uint8_t             tx_data[8] = {0};
//    
//    tx_header.StdId = 0x11F;//标识符（见手册P6）
//    tx_header.IDE   = CAN_ID_STD;//标准ID
//    tx_header.RTR   = CAN_RTR_DATA;//数据帧
//    tx_header.DLC   = 8;//字节长度
//	
//	tx_data[0] = pData[0];
//    tx_data[1] = pData[1];
//	
//	tx_data[2] = pData[2];
//	tx_data[3]= pData[5];
//	
//	tx_data[4]=pData[16];
//	tx_data[5]=pData[17];

//    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);

	/*比赛信息*/
		
		//		CAN_TxHeaderTypeDef tx_header;
//    uint8_t             tx_data[8] = {0};
//    
//    tx_header.StdId = 0x11F;//标识符（见手册P6）
//    tx_header.IDE   = CAN_ID_STD;//标准ID
//    tx_header.RTR   = CAN_RTR_DATA;//数据帧
//    tx_header.DLC   = 8;//字节长度
//	
//	tx_data[0] = pData[0];
//    tx_data[1] = pData[1];
//	
//	tx_data[2] = pData[2];
//	tx_data[3]= pData[5];
//	
//	tx_data[4]=pData[16];
//	tx_data[5]=pData[17];

//    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);

	/*受击打标识*/
		
		//		CAN_TxHeaderTypeDef tx_header;
//    uint8_t             tx_data[8] = {0};
//    
//    tx_header.StdId = 0x11F;//标识符（见手册P6）
//    tx_header.IDE   = CAN_ID_STD;//标准ID
//    tx_header.RTR   = CAN_RTR_DATA;//数据帧
//    tx_header.DLC   = 8;//字节长度
//	
//	tx_data[0] = pData[0];
//    tx_data[1] = pData[1];
//	
//	tx_data[2] = pData[2];
//	tx_data[3]= pData[5];
//	
//	tx_data[4]=pData[16];
//	tx_data[5]=pData[17];

//    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);


			/*发射状态量*/
		
		//		CAN_TxHeaderTypeDef tx_header;
//    uint8_t             tx_data[8] = {0};
//    
//    tx_header.StdId = 0x11F;//标识符（见手册P6）
//    tx_header.IDE   = CAN_ID_STD;//标准ID
//    tx_header.RTR   = CAN_RTR_DATA;//数据帧
//    tx_header.DLC   = 8;//字节长度
//	
//	tx_data[0] = pData[0];
//    tx_data[1] = pData[1];
//	
//	tx_data[2] = pData[2];
//	tx_data[3]= pData[5];
//	
//	tx_data[4]=pData[16];
//	tx_data[5]=pData[17];

//    HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
    return retval_tf;//对数据正误做处理

}

#define send_max_len     200
unsigned char CliendTxBuffer[send_max_len];
//void JUDGE_Show_Data(void)
//{
//    //	static uint8_t datalength,i;
//    //	uint8_t judge_led = 0xff;//初始化led为全绿
//    //	static uint8_t auto_led_time = 0;
//    //	static uint8_t buff_led_time = 0;

//    determine_ID();//判断发送者ID和其对应的客户端ID

//    Show_data.frame_header.SOF = 0xA5;
//    Show_data.frame_header.Data_Length = sizeof(ext_student_interactive_header_data_t) + sizeof(operate_data_t);
//    Show_data.frame_header.Seq = 0;
//    memcpy(CliendTxBuffer, &Show_data.frame_header, sizeof(frame_header_t));//写入帧头数据
//    Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(frame_header_t));//写入帧头CRC8校验码

//    Show_data.cmd_id = 0x0301;

//    Show_data.student_interactive_header.data_cmd_id = 0xD180;//发送给客户端的cmd，官方固定
//    //ID已经是自动读取了
//    Show_data.student_interactive_header.sender_ID 	 = Judge_Self_ID;//发送者的ID
//    Show_data.student_interactive_header.receiver_ID = Judge_SelfClient_ID;//发送者的ID，只能为发送者机器人对应的客户端
//}

#define Teammate_max_len     200
unsigned char TeammateTxBuffer[Teammate_max_len];
bool Send_Color = 0;
bool First_Time_Send_Commu = true;
uint16_t send_time = 0;



bool Color;
bool is_red_or_blue(void)//判断自己红蓝方
{
    Judge_Self_ID = judge_frame_rx.data.game_robot_status.robot_id;//读取当前机器人ID

    if(judge_frame_rx.data.game_robot_status.robot_id > 10)
    {
        return BLUE;
    }
    else
    {
        return RED;
    }
}

void determine_ID(void)
{
    Color = is_red_or_blue();
    if(Color == BLUE)
    {
        Judge_SelfClient_ID = 0x0164 + (Judge_Self_ID - 100); //计算客户端id
    }
    else if(Color == RED)
    {
        Judge_SelfClient_ID = 0x0100 + Judge_Self_ID;//计算客户端ID
    }
}

/********************裁判数据辅助判断函数***************************/

/**
  * @brief  数据是否可用
  * @param  void
  * @retval  TRUE可用   FALSE不可用
  * @attention  在裁判读取函数中实时改变返回值
  */
bool JUDGE_sGetDataState(void)
{
    return Judge_Data_TF;
}

/**
  * @brief  读取瞬时功率
  * @param  void
  * @retval 实时功率值
  * @attention
  */
float JUDGE_fGetChassisPower(void)
{
    return (judge_frame_rx.data.power_heat_data.chassis_power);
}

/**
  * @brief  读取底盘功率限制
  * @param  void
  * @retval 实时底盘功率限制
  * @attention
  */
uint8_t JUDGE_usGetPowerLimit(void)
{
    return (judge_frame_rx.data.game_robot_status.chassis_power_limit_nothing);
}
/**
  * @brief  读取剩余焦耳能量
  * @param  void
  * @retval 剩余缓冲焦耳能量
  * @attention
  */
uint16_t JUDGE_fGetRemainEnergy(void)
{
    return (judge_frame_rx.data.power_heat_data.chassis_power_buffer);
}

/**
  * @brief  读取当前等级
  * @param  void
  * @retval 当前等级
  * @attention
  */
uint8_t JUDGE_ucGetRobotLevel(void)
{
    return	judge_frame_rx.data.game_robot_status.robot_level;
}


/**
  * @brief  读取枪口热量
  * @param  void
  * @retval 
  * @attention  实时热量
  */
uint16_t JUDGE_usGetRemoteHeat17_1(void)
{
    return judge_frame_rx.data.power_heat_data.shooter_id1_17mm_cooling_heat;//代码的左
}
uint16_t JUDGE_usGetRemoteHeat17_2(void)
{
    return judge_frame_rx.data.power_heat_data.shooter_id2_17mm_cooling_heat;//代码的右
}
/**
  * @brief  读取比赛类型
  * @param  void
  * @retval void
  * @attention  联盟赛超级对抗赛
  */
uint8_t JUDGE_Game_Type(void)
{
    return judge_frame_rx.data.game_status.game_type;
}

/**
  * @brief  读取比赛阶段
  * @param  void
  * @retval void
  * @attention  实时比赛阶段
  */
uint8_t JUDGE_Game_Status(void)
{
    return judge_frame_rx.data.game_status.game_progress;
}

/**
  * @brief  读取射速
  * @param  void
  * @retval 17mm
  * @attention  双枪管
  */
float bullet_speed1,bullet_speed2=0;

void JUDGE_usGetSpeed17_double(void)
{
    
	if(judge_frame_rx.data.shoot_data.shooter_id==2)
	{
		bullet_speed2=judge_frame_rx.data.shoot_data.bullet_speed;
	}
	else if(judge_frame_rx.data.shoot_data.shooter_id==1)
	{
	  bullet_speed1=judge_frame_rx.data.shoot_data.bullet_speed;
	}
	
}
/**
  * @brief  读取允许发弹量
  * @param  void
  * @retval void
  * @attention  实时弹量
  */
float JUDGE_Remaining(void)
{
    return judge_frame_rx.data.bullet_remaining.bullet_remaining_num_17mm;
}


/**
  * @brief  读取枪口热量
  * @param  void
  * @retval 当前等级热量上限
  * @attention
  */
uint16_t JUDGE_usGetHeatLimit1(void)
{
    return judge_frame_rx.data.game_robot_status.shooter_id1_17mm_cooling_limit;
}

uint16_t JUDGE_usGetHeatLimit2(void)
{
    return judge_frame_rx.data.game_robot_status.shooter_id2_17mm_cooling_limit;
}


/**
  * @brief  当前等级对应的枪口每秒冷却值
  * @param  void
  * @retval 当前等级冷却速度
  * @attention
  */
uint16_t JUDGE_usGetShootCold1(void)
{
    return judge_frame_rx.data.game_robot_status.shooter_id1_17mm_cooling_rate;
}

uint16_t JUDGE_usGetShootCold2(void)
{
    return judge_frame_rx.data.game_robot_status.shooter_id2_17mm_cooling_rate;
}



bool Judge_If_Death(void)
{
    if(judge_frame_rx.data.game_robot_status.remain_HP == 0 && JUDGE_sGetDataState() == true)
    {
        return true;
    }
    else
    {
        return false;
    }
}


float Last_HP = 600 ;
float Now_HP = 600;
uint8_t Hurt_Flag = 0;
bool  JUDGE_Is_Hurt(void)
{
	Now_HP = judge_frame_rx.data.game_robot_status.remain_HP;
	if(Now_HP < Last_HP)
    Hurt_Flag =TRUE;
	else
	{
	  Hurt_Flag = FALSE;
	}
		Last_HP = Now_HP;
	return Hurt_Flag;

}
float JUDGE_Read_HP(void)
{
	return judge_frame_rx.data.game_robot_status.remain_HP;
}




uint16_t get_outpost_HP(void)//获取前哨站血量
{
	
	if(judge_frame_rx.data.game_robot_status.robot_id>100) //blue
	{
		
		return judge_frame_rx.data.game_robot_HP.blue_outpost_HP;
		
	}
	
	else  //red
	{
		
		return judge_frame_rx.data.game_robot_HP.red_outpost_HP;
		
	}
	
}

void get_enemy_position(void)
{
//	if(judge_frame_rx.data.student_interactive_header_data.data_cmd_id == 0x20F)
//	{
//	if(judge_frame_rx.data.student_interactive_header_data.sender_ID == Judge_Self_ID + 2)
//	{
//		VisionTransmit.enemy1_x_position.buff[0] = judge_frame_rx.data.robot_interactive_data.data[0];
//		VisionTransmit.enemy1_x_position.buff[1] = judge_frame_rx.data.robot_interactive_data.data[1];
//		VisionTransmit.enemy1_x_position.buff[2] = judge_frame_rx.data.robot_interactive_data.data[2];
//		VisionTransmit.enemy1_x_position.buff[3] = judge_frame_rx.data.robot_interactive_data.data[3];
//		VisionTransmit.enemy1_y_position.buff[0] = judge_frame_rx.data.robot_interactive_data.data[4];
//		VisionTransmit.enemy1_y_position.buff[1] = judge_frame_rx.data.robot_interactive_data.data[5];
//		VisionTransmit.enemy1_y_position.buff[2] = judge_frame_rx.data.robot_interactive_data.data[6];
//		VisionTransmit.enemy1_y_position.buff[3] = judge_frame_rx.data.robot_interactive_data.data[7];
//		VisionTransmit.enemy2_x_position.buff[0] = judge_frame_rx.data.robot_interactive_data.data[8];
//		VisionTransmit.enemy2_x_position.buff[1] = judge_frame_rx.data.robot_interactive_data.data[9];
//		VisionTransmit.enemy2_x_position.buff[2] = judge_frame_rx.data.robot_interactive_data.data[10];
//		VisionTransmit.enemy2_x_position.buff[3] = judge_frame_rx.data.robot_interactive_data.data[11];
//		VisionTransmit.enemy2_y_position.buff[0] = judge_frame_rx.data.robot_interactive_data.data[12];
//		VisionTransmit.enemy2_y_position.buff[1] = judge_frame_rx.data.robot_interactive_data.data[13];
//		VisionTransmit.enemy2_y_position.buff[2] = judge_frame_rx.data.robot_interactive_data.data[14];
//		VisionTransmit.enemy2_y_position.buff[3] = judge_frame_rx.data.robot_interactive_data.data[15];
//		VisionTransmit.enemy3_x_position.buff[0] = judge_frame_rx.data.robot_interactive_data.data[16];
//		VisionTransmit.enemy3_x_position.buff[1] = judge_frame_rx.data.robot_interactive_data.data[17];
//		VisionTransmit.enemy3_x_position.buff[2] = judge_frame_rx.data.robot_interactive_data.data[18];
//		VisionTransmit.enemy3_x_position.buff[3] = judge_frame_rx.data.robot_interactive_data.data[19];
//		VisionTransmit.enemy3_y_position.buff[0] = judge_frame_rx.data.robot_interactive_data.data[20];
//		VisionTransmit.enemy3_y_position.buff[1] = judge_frame_rx.data.robot_interactive_data.data[21];
//		VisionTransmit.enemy3_y_position.buff[2] = judge_frame_rx.data.robot_interactive_data.data[22];
//		VisionTransmit.enemy3_y_position.buff[3] = judge_frame_rx.data.robot_interactive_data.data[23];
//		VisionTransmit.enemy4_x_position.buff[0] = judge_frame_rx.data.robot_interactive_data.data[24];
//		VisionTransmit.enemy4_x_position.buff[1] = judge_frame_rx.data.robot_interactive_data.data[25];
//		VisionTransmit.enemy4_x_position.buff[2] = judge_frame_rx.data.robot_interactive_data.data[26];
//		VisionTransmit.enemy4_x_position.buff[3] = judge_frame_rx.data.robot_interactive_data.data[27];
//		VisionTransmit.enemy4_y_position.buff[0] = judge_frame_rx.data.robot_interactive_data.data[28];
//		VisionTransmit.enemy4_y_position.buff[1] = judge_frame_rx.data.robot_interactive_data.data[29];
//		VisionTransmit.enemy4_y_position.buff[2] = judge_frame_rx.data.robot_interactive_data.data[30];
//		VisionTransmit.enemy4_y_position.buff[3] = judge_frame_rx.data.robot_interactive_data.data[31];
//		VisionTransmit.enemy5_x_position.buff[0] = judge_frame_rx.data.robot_interactive_data.data[32];
//		VisionTransmit.enemy5_x_position.buff[1] = judge_frame_rx.data.robot_interactive_data.data[33];
//		VisionTransmit.enemy5_x_position.buff[2] = judge_frame_rx.data.robot_interactive_data.data[34];
//		VisionTransmit.enemy5_x_position.buff[3] = judge_frame_rx.data.robot_interactive_data.data[35];
//		VisionTransmit.enemy5_y_position.buff[0] = judge_frame_rx.data.robot_interactive_data.data[36];
//		VisionTransmit.enemy5_y_position.buff[1] = judge_frame_rx.data.robot_interactive_data.data[37];
//		VisionTransmit.enemy5_y_position.buff[2] = judge_frame_rx.data.robot_interactive_data.data[38];
//		VisionTransmit.enemy5_y_position.buff[3] = judge_frame_rx.data.robot_interactive_data.data[39];
//		VisionTransmit.enemy7_x_position.buff[0] = judge_frame_rx.data.robot_interactive_data.data[40];
//		VisionTransmit.enemy7_x_position.buff[1] = judge_frame_rx.data.robot_interactive_data.data[41];
//		VisionTransmit.enemy7_x_position.buff[2] = judge_frame_rx.data.robot_interactive_data.data[42];
//		VisionTransmit.enemy7_x_position.buff[3] = judge_frame_rx.data.robot_interactive_data.data[43];
//		VisionTransmit.enemy7_y_position.buff[0] = judge_frame_rx.data.robot_interactive_data.data[44];
//		VisionTransmit.enemy7_y_position.buff[1] = judge_frame_rx.data.robot_interactive_data.data[45];
//		VisionTransmit.enemy7_y_position.buff[2] = judge_frame_rx.data.robot_interactive_data.data[46];
//		VisionTransmit.enemy7_y_position.buff[3] = judge_frame_rx.data.robot_interactive_data.data[47];	
//	}
//	}
}


