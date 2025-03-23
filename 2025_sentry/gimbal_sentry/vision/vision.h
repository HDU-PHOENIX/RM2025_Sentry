#ifndef  __VISION_H__
#define  __VISION_H__
#include "main.h"

typedef struct
{
	uint8_t type;   //0：无  1：买活   2：买弹丸
	uint8_t content;  //具体内容
}computer_on;

typedef struct
{
	uint8_t type;//0：无  1：小陀螺  2：云台连发控制
	uint8_t content;   //具体内容
}computer_off;

typedef struct
{
	float x_speed;   //x的速度，y的速度
	float y_speed;
}computer_chassis;

typedef struct
{
	char fine_bool;   //是否追踪
	float big_yaw;    //yaw轴角度
	float pitch;    //pitch轴角度
	float xiao_yaw;
}computer_zimiao;

typedef struct
{
	computer_zimiao zimiao;
	computer_chassis computer_vpm;
	computer_on computer_juece;
	computer_off computer_exercise;
}computer_all;



typedef struct
{
	float x_speed;   //x的速度，y的速度
	float y_speed;
}motor_chassis;


void vision_call_back_handler(uint8_t *buff);
void vision_transmit_package_AutoaimFeedback (uint8_t *buff);
void vision_transmit();
void vision_transmit_judge();

#endif