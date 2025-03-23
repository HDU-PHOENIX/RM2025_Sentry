#ifndef  __BASPCAN_H__
#define  __BASPCAN_H__
#include "main.h"
void can_filter_init(void);

//typedef struct{
//	int8_t temp;
//	int16_t current;
//	int16_t speed;
//	uint16_t encoder_place;    //-32768-32768
//}motor_9025;
void vision_transmit_friend1_place (uint8_t *buff);
void vision_transmit_friend2_place (uint8_t *buff);
void vision_transmit_red_hp (uint8_t *buff);
void vision_transmit_blue_hp (uint8_t *buff);
void vision_transmit_building_hp (uint8_t *buff);
void vision_transmit_game_state (uint8_t *buff);
void vision_transmit_exercise (uint8_t *buff);
void vision_transmit_shouji (uint8_t *buff);
void vision_transmit_shoot (uint8_t *buff);


#endif