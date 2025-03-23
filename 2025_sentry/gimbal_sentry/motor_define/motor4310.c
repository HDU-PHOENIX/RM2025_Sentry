#include "DM4310.h"

//#include "bsp_can.h"
#include "main.h"
#include "can.h"

 #define P_MIN -3.1415923f
 #define P_MAX 3.1415923f
 #define V_MIN -30.0f
 #define V_MAX 30.0f
 #define KP_MIN 0.0f
 #define KP_MAX 500.0f
 #define KD_MIN 0.0f
 #define KD_MAX 5.0f
 #define T_MIN -10.0f
 #define T_MAX 10.0f
 
 DM4310_TypeDef DM4310_pitch;
 
uint8_t *pbuf,*vbuf;
uint8_t Txtemp_1[8];
void Motor_DM4310_Enable(CAN_HandleTypeDef* hcan,uint16_t id)   //使能
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = id;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = 0xFF;
	Txtemp[1] = 0xFF;
	Txtemp[2] = 0xFF;
	Txtemp[3] = 0xFF;
	Txtemp[4] = 0xFF;
	Txtemp[5] = 0xFF;
	Txtemp[6] = 0xFF;
	Txtemp[7] = 0xFC;  //FD是退出电机，即失能
	
	uint8_t count = 0;
	while( HAL_CAN_GetTxMailboxesFreeLevel( &hcan2 ) == 0 && count < 100){
			count++;
	};
		if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{
		
	}
}	

void Motor_DM4310_Enable_position(CAN_HandleTypeDef* hcan,uint16_t id)   //使能
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x100+id;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = 0xFF;
	Txtemp[1] = 0xFF;
	Txtemp[2] = 0xFF;
	Txtemp[3] = 0xFF;
	Txtemp[4] = 0xFF;
	Txtemp[5] = 0xFF;
	Txtemp[6] = 0xFF;
	Txtemp[7] = 0xFC;  //FD是退出电机，即失能
	
	uint8_t count = 0;
	while( HAL_CAN_GetTxMailboxesFreeLevel( &hcan2 ) == 0 && count < 100){
			count++;
	};
		if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{
		if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX1)!=HAL_OK)
	{
		if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX2)!=HAL_OK)
	{
		
	}
	}
	}
}

void Motor_DM4310_Enable_speed(CAN_HandleTypeDef* hcan,uint16_t id)   //使能
{
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x200+id;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	Txtemp[0] = 0xFF;
	Txtemp[1] = 0xFF;
	Txtemp[2] = 0xFF;
	Txtemp[3] = 0xFF;
	Txtemp[4] = 0xFF;
	Txtemp[5] = 0xFF;
	Txtemp[6] = 0xFF;
	Txtemp[7] = 0xFC;  //FD是退出电机，即失能
	
	uint8_t count = 0;
	while( HAL_CAN_GetTxMailboxesFreeLevel( &hcan2 ) == 0 && count < 100){
			count++;
	};
		if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{
		
	}
}	



void DM4310_angle(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel,float _torq){
	CAN_TxHeaderTypeDef tx_header;
    uint8_t             tx_data[8] = {0};
    
    tx_header.StdId = 0x101;//标识符（见手册P6）
    tx_header.IDE   = CAN_ID_STD;//标准ID
    tx_header.RTR   = CAN_RTR_DATA;//数据帧
    tx_header.DLC   = 8;//字节长度


	tx_data[0]=
    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}


//void Motor_DM4310_send(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq)
//{
//  uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
//	CAN_TxHeaderTypeDef _TxHeader;
//	uint8_t Txtemp[8];
//	_TxHeader.StdId = id;
//	_TxHeader.IDE = CAN_ID_STD;
//	_TxHeader.RTR = CAN_RTR_DATA;
//	_TxHeader.DLC = 0x08;
//	pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
//  vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
//	kp_tmp  = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
//	kd_tmp  = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
//  tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);
//	
//	Txtemp[0] = (pos_tmp >> 8);
//	Txtemp[1] = pos_tmp;
//	Txtemp[2] = (vel_tmp >> 4);
//	Txtemp[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
//	Txtemp[4] = kp_tmp;
//	Txtemp[5] = (kd_tmp >> 4);
//	Txtemp[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
//	Txtemp[7] = tor_tmp;

//	uint8_t count = 0;
//		if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
//	{
//		
//	}
//}	

//void Motor_DM4310_send(CAN_HandleTypeDef *hcan, uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq)
//{
//    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
//    CAN_TxHeaderTypeDef _TxHeader;
//    uint8_t Txtemp[8];
//    uint32_t TxMailboxX = CAN_TX_MAILBOX0; // CAN????
//    _TxHeader.StdId = id;
//    _TxHeader.IDE = CAN_ID_STD;
//    _TxHeader.RTR = CAN_RTR_DATA;
//    _TxHeader.DLC = 0x08;

//    pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
//    vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
//    kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
//    kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
//    tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

//    Txtemp[0] = (pos_tmp >> 8);
//    Txtemp[1] = pos_tmp;
//    Txtemp[2] = (vel_tmp >> 4);
//    Txtemp[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
//    Txtemp[4] = kp_tmp;
//    Txtemp[5] = (kd_tmp >> 4);
//    Txtemp[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
//    Txtemp[7] = tor_tmp;

//    // ??????????
//    uint32_t tsr = hcan->Instance->TSR;
//    if ((tsr & CAN_TSR_TME0) != RESET)
//    {
//        TxMailboxX = CAN_TX_MAILBOX0;
//    }
//    else if ((tsr & CAN_TSR_TME1) != RESET)
//    {
//        TxMailboxX = CAN_TX_MAILBOX1;
//    }
//    else if ((tsr & CAN_TSR_TME2) != RESET)
//    {
//        TxMailboxX = CAN_TX_MAILBOX2;
//    }
//    else
//    {
//        // ??????????????
//        do
//        {
//            tsr = hcan->Instance->TSR;
//        } while ((tsr & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0);

//        if ((tsr & CAN_TSR_TME0) != RESET)
//        {
//            TxMailboxX = CAN_TX_MAILBOX0;
//        }
//        else if ((tsr & CAN_TSR_TME1) != RESET)
//        {
//            TxMailboxX = CAN_TX_MAILBOX1;
//        }
//        else
//        {
//            TxMailboxX = CAN_TX_MAILBOX2;
//        }
//    }

//    // ?????CAN????
//#if DEBUGMODE
//    if (HAL_CAN_AddTxMessage(hcan, &_TxHeader, Txtemp, (uint32_t *)TxMailboxX) != HAL_OK)
//    {
//        Error_Handler();
//    }
//#else
//    HAL_CAN_AddTxMessage(hcan, &_TxHeader, Txtemp, (uint32_t *)TxMailboxX);
//#endif
//}

void Motor_DM4310_send(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel, float _KP, float _KD, float _torq)
{
  uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = id;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
	kp_tmp  = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);
	
	Txtemp[0] = (pos_tmp >> 8);
	Txtemp[1] = pos_tmp;
	Txtemp[2] = (vel_tmp >> 4);
	Txtemp[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	Txtemp[4] = kp_tmp;
	Txtemp[5] = (kd_tmp >> 4);
	Txtemp[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	Txtemp[7] = tor_tmp;

	uint8_t count = 0;
		if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{
		if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX1)!=HAL_OK)
	{
			if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX2)!=HAL_OK)
	{
		
}	
		
}	
}	
}

void Motor_DM4310_position(CAN_HandleTypeDef* hcan,uint8_t CAN_ID,float position,float speed)
{
	uint32_t position_int,speed_int;
	CAN_TxHeaderTypeDef _TxHeader;
	uint8_t Txtemp[8];
	_TxHeader.StdId = 0x101;
	_TxHeader.IDE = CAN_ID_STD;
	_TxHeader.RTR = CAN_RTR_DATA;
	_TxHeader.DLC = 0x08;
	
	position_int=float_to_uint(position,P_MIN, P_MAX,16);
	speed_int=float_to_uint(speed,V_MIN, V_MAX,16);
	
	Txtemp[0]=position_int;
	Txtemp[1]=position_int>>8;
	Txtemp[2]=(position_int>>8)>>8;
	Txtemp[3]=((position_int>>8)>>8)>>8;
	
	Txtemp[4]=speed_int;
	Txtemp[5]=speed_int>>8;
	Txtemp[6]=(speed_int>>8)>>8;
	Txtemp[7]=((speed_int>>8)>>8)>>8;
	
	uint8_t count = 0;
	while( HAL_CAN_GetTxMailboxesFreeLevel( &hcan2 ) == 0 && count < 100){
			count++;
	};
		if(HAL_CAN_AddTxMessage(&hcan1,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK)
	{
		
	}
}



//void Motor_DM4310_receive(DM4310_TypeDef* motor, uint8_t* temp, uint8_t CAN_ID)
//{
//	int p_int;
//	int v_int;
//	int t_int;
//		
//	motor->ID = CAN_ID; 
//	p_int=(temp[1]<<8)|temp[2];
//	v_int=(temp[3]<<4)|(temp[4]>>4);
//	t_int=((temp[4]&0xF)<<8)|temp[5];

//	motor->angle = uint_to_float(p_int, P_MIN, P_MAX, 16);
//	motor->speed_rpm = uint_to_float(v_int, V_MIN, V_MAX, 12);
//	motor->torque= uint_to_float(t_int, T_MIN, T_MAX, 12);	//输出力矩
//	motor->tempure = (float)(temp[6]);

//	
//	motor->msg_cnt++;
//	motor->msg_missing_time=0;

//}

void Motor_DM4310_receive(DM4310_TypeDef* Motor, uint8_t* temp, uint8_t CAN_ID)
{


	int p_int;
	int v_int;
	int t_int;
	
	Motor->ID = CAN_ID; 
	p_int=(temp[1]<<8)|temp[2];
	v_int=(temp[3]<<4)|(temp[4]>>4);
	t_int=((temp[4]&0xF)<<8)|temp[5];

	Motor->angle = uint_to_float(p_int, P_MIN, P_MAX, 16);
	Motor->speed_rpm = uint_to_float(v_int, V_MIN, V_MAX, 12);
	Motor->torque= uint_to_float(t_int, T_MIN, T_MAX, 12);	//输出力矩
	Motor->tempure = (float)(temp[6]);
	Motor->msg_cnt++;
	Motor->msg_missing_time=0;

}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
    
    
int float_to_uint(float x, float x_min, float x_max, int bits)
{ 
  float span = x_max - x_min;
  float offset = x_min;
  return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

void position_speed_control(CAN_HandleTypeDef* hcan,uint16_t id, float _pos, float _vel){
    pbuf=(uint8_t*)&_pos;
    vbuf=(uint8_t*)&_vel;
    CAN_TxHeaderTypeDef _TxHeader;
    _TxHeader.StdId = 0x100+id;
    _TxHeader.IDE = CAN_ID_STD;
    _TxHeader.RTR = CAN_RTR_DATA;
    _TxHeader.DLC = 0x08;
    
    Txtemp_1[0] = *pbuf;
    Txtemp_1[1] = *(pbuf+1);
    Txtemp_1[2] = *(pbuf+2);
    Txtemp_1[3] = *(pbuf+3);
    Txtemp_1[4] = *vbuf;
    Txtemp_1[5] = *(vbuf+1);
    Txtemp_1[6] = *(vbuf+2);
    Txtemp_1[7] = *(vbuf+3);
	
    
    if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp_1,(uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)
	{
		if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp_1,(uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			if(HAL_CAN_AddTxMessage(hcan,&_TxHeader,Txtemp_1,(uint32_t*)CAN_TX_MAILBOX2) != HAL_OK)
			{
			
			}
		}
	}
}

void speed_4310_control(CAN_HandleTypeDef* hcan,uint16_t id, float _vel){   //float类型的传输
    
	uint8_t *vbuf;
	CAN_TxHeaderTypeDef _TxHeader;
    _TxHeader.StdId = 0x200+id;
    _TxHeader.IDE = CAN_ID_STD;
    _TxHeader.RTR = CAN_RTR_DATA;
    _TxHeader.DLC = 0x04;
	uint8_t Txtemp[4];
	
	vbuf=(uint8_t*)&_vel;
	
	Txtemp[0] = *vbuf;
	Txtemp[1] = *(vbuf+1);
	Txtemp[2] = *(vbuf+2);
	Txtemp[3] = *(vbuf+3);
	
	HAL_CAN_AddTxMessage(&hcan1,&_TxHeader,Txtemp,(uint32_t*)CAN_TX_MAILBOX0);
}