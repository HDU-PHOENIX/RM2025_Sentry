#ifndef  __LK_9025_H__
#define  __LK_9025_H__
#include "main.h"
#include "pid.h"


//自带闭环，不需要再加PID
typedef struct{
	int8_t temp;
	int16_t current;
	int16_t speed;
	uint16_t encoder_place;    //-32768-32768
	float target_angle;
	float now_angle;
	float target_speed;
	float now_speed;
	pid_struct_t LK_9025_speed;
	pid_struct_t LK_9025_angle;
	pid_struct_t LK_9025_current;
}motor_9025;



void Motor_LK9025_Enable(CAN_HandleTypeDef* hcan,uint16_t id);
void Motor_LK9025_disable(CAN_HandleTypeDef* hcan,uint16_t id);
void Motor_LK9025_control_speed(CAN_HandleTypeDef* hcan,uint16_t id,int32_t speed);
void Motor_LK9025_control_angle(CAN_HandleTypeDef* hcan,uint16_t id,int32_t angle);
void Motor_LK9025_control_FN(CAN_HandleTypeDef* hcan,uint16_t id,int16_t speed);
void Motor_LK9025_angle_get();

/*       询问得到编码器值，角度值                                                           */
void Motor_LK9025_for_encode(CAN_HandleTypeDef* hcan,uint16_t id);
void Motor_LK9025_for_angle(CAN_HandleTypeDef* hcan,uint16_t id);



void Motor_LK9025_sent_for_angle(CAN_HandleTypeDef* hcan,uint16_t id);
void Motor_LK9025_clear_error(CAN_HandleTypeDef* hcan,uint16_t id);




void Motor_LK9025_clear_error(CAN_HandleTypeDef* hcan,uint16_t id);
void Motor_LK9025_angle_more(CAN_HandleTypeDef* hcan,uint16_t id,int32_t anglecrement);



#endif