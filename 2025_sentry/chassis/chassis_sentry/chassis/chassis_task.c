#include "main.h"
#include "chassis_move.h"
#include "remote.h"
#include "FreeRTOS.h"
#include "Task.h"
#include "motor_pid_init.h"
#include "DM4310.h"
#include "LK_9025.h"
#include "pid.h"
#include "cmsis_os.h"
#include "judge.h"
#include "INS_task.h"

#include "string.h"

#define Gimbal_Pitch_ZERO 6720

//extern chass_t Chassis;
extern RC_Ctl_t RC_CtrlData;

extern motor_3508_t motor_3508[4];
extern motor_6020_t motor_yaw,motor_pitch;
int16_t FN_9025=300;
float speed_9025=0;

extern frame_t judge_frame_rx;
float remote_9025_xishu=14;



extern DM4310_TypeDef DM4310_pitch;
int32_t yaw_speed=0;
int32_t speed_move_target=300;
pid_struct_t DM4310_pitch_speed,DM4310_pitch_angle;
//pid_struct_t LK_9025_speed,LK_9025_angle;
float angle_9025=19660,test_9025=0;

extern motor_9025 motor_9025_yaw;

int angle_more=0;
int remote_angle=0;
uint32_t command;

extern uint8_t fuhuo_flag;
extern uint8_t zidan_much;

uint8_t gyro_flag=1;
float gyro_angle=0;

extern float imu_yaw_angle;

int32_t speed_try=0;

void Handle_Angle8192_PID_Over_Zero(float *tar, float *cur)
{
	if(*tar - *cur > 4096)    //4096 ：半圈机械角度
	{
		*cur += 8192;        //8192:整圈
	}
	else if(*tar - *cur < -4096)
	{
		*cur = *cur - 8192;
	}
	else
	{
		//*cur = *cur;
		// do nothing
	}
}

void motor4310_calc(){
	//PID_Calc_Angle(&DM4310_pitch_angle,)
	PID_Calc_Speed(&DM4310_pitch_speed,2,DM4310_pitch.speed_rpm);
}

typedef struct {
    uint32_t confirm_resurrection : 1;          // bit 0: ??????
    uint32_t confirm_instant_resurrection : 1;  // bit 1: ??????????
    uint32_t bullet_exchange_value : 11;        // bit 2-12: ????
    uint32_t remote_bullet_request_count : 4;   // bit 13-16: ???????????
    uint32_t remote_health_request_count : 4;   // bit 17-20: ??????????
    uint32_t reserved : 11;                     // bit 21-31: ???
} SentinelDecisionCommand;

// ???????????
typedef struct {
    uint16_t frame_header;          // ??,??? 0xA5
    uint16_t data_length;           // ????
    uint8_t seq;                    // ???
    uint8_t crc8;                   // CRC8 ??
    uint16_t cmd_id;                // ?? ID
    uint8_t data[4];                // ????(????????)
} RefereeFrame;

// ??????????
uint32_t set_sentinel_decision_command(uint32_t confirm_resurrection, 
                                       uint32_t confirm_instant_resurrection, 
                                       uint32_t bullet_exchange_value, 
                                       uint32_t remote_bullet_request_count, 
                                       uint32_t remote_health_request_count) {
    SentinelDecisionCommand command;
    command.confirm_resurrection = confirm_resurrection;
    command.confirm_instant_resurrection = confirm_instant_resurrection;
    command.bullet_exchange_value = bullet_exchange_value;
    command.remote_bullet_request_count = remote_bullet_request_count;
    command.remote_health_request_count = remote_health_request_count;
    command.reserved = 0;  // ??????0

    // ???????32???
    return *(uint32_t*)&command;
}

// ?? CRC8 ??
uint8_t calculate_crc8(const uint8_t *data, uint16_t length) {
    uint8_t crc = 0xFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// ?????????
void pack_referee_frame(RefereeFrame *frame, uint16_t cmd_id, uint32_t data) {
    frame->frame_header = 0xA5; // ????? 0xA5
    frame->data_length = 4;     // ????? 4 ??
    frame->seq = 0;             // ???(???????)
    frame->cmd_id = cmd_id;     // ?? ID
    memcpy(frame->data, &data, 4); // ??????

    // ?? CRC8 ??
    frame->crc8 = calculate_crc8((uint8_t *)frame, sizeof(RefereeFrame) - 1);
}

// ??????????
void send_sentinel_decision_command(UART_HandleTypeDef *huart, uint16_t cmd_id, uint32_t command) {
    RefereeFrame frame;
    pack_referee_frame(&frame, cmd_id, command);

    // ?? DMA ????
    HAL_UART_Transmit_DMA(huart, (uint8_t *)&frame, sizeof(RefereeFrame));
}



void StartChassisTask(void const * argument)
{
    portTickType currentTime;
	Chassis_PID_Init_All();
	osDelay(500);
	//pid_init(&motor_9025_yaw.LK_9025_speed,2.1,0.04,0,1200,2045);
	pid_init(&motor_9025_yaw.LK_9025_speed,1.7,0,0,1200,2045);
	//pid_init(&motor_9025_yaw.LK_9025_current,230,40,0,-16.5,16.5);
	pid_init(&motor_9025_yaw.LK_9025_angle,0,0,0,300,300);
	motor_9025_yaw.LK_9025_angle.i_seperate=1200;
	motor_9025_yaw.LK_9025_speed.i_seperate=50;
	motor_9025_yaw.target_angle=0;
	motor_9025_yaw.target_speed=0;
	osDelay(1);
	Motor_LK9025_Enable(&hcan2,0x141);
    currentTime = xTaskGetTickCount();//当前系统时间
	for(;;)
    { 
		//Motor_LK9025_Enable(&hcan2,0x141);
		//Motor_LK9025_Enable(&hcan2,0x141);
		
		if(RC_CtrlData.rc.s1==1 && RC_CtrlData.rc.s2==1)  //自动模式
		{
			Chassis_Move_Auto();
		}
		if(RC_CtrlData.rc.s1==2){
		if(RC_CtrlData.rc.s1==2 && RC_CtrlData.rc.s2==3){
			gyro_flag=1;
			Chassis_Remote_Control_normal();
			Chassis_Move();
			//remote_angle=-RC_CtrlData.rc.ch0/110;
			//Motor_LK9025_angle_more(&hcan2,0x141,remote_angle);
			speed_9025=-RC_CtrlData.rc.ch0*25;
			Motor_LK9025_control_speed(&hcan2,0x141,speed_9025);
			
			osDelay(1);
			}
		else if(RC_CtrlData.rc.s1==2 && RC_CtrlData.rc.s2==2){
			gyro_flag=1;
			Chassis_Remote_Control_normal();
			Chassis_Move();
			Motor_LK9025_angle_more(&hcan2,0x141,0);
//			target_anlge=target_anlge+RC_CtrlData.rc.ch0/660.0f;
		}
		else if(RC_CtrlData.rc.s1==2 && RC_CtrlData.rc.s2==1){ //小陀螺
			Chassis_Remote_Control_normal();
			Chassis_Move();
			if(gyro_flag==1){
				gyro_flag=0;
				gyro_angle=imu_yaw_angle;
			}
			
						angle_more=40000+RC_CtrlData.rc.ch0*12+(gyro_angle-imu_yaw_angle)*7000;//24500//36700
			
			if(angle_more>60000){
				angle_more=60000;
			}
			
			Motor_LK9025_control_speed(&hcan2,0x141,angle_more);
		}
	}
		//PID_Calc_Speed(&motor_9025_yaw.LK_9025_speed,)
		//Chassis_Remote_Control_normal();
		//Chassis_Move();
		// yaw_speed=500;
		//Motor_LK9025_control_speed(&hcan2,0x141,yaw_speed);
		motor_9025_yaw.now_angle=((float)motor_9025_yaw.encoder_place)/65535.0f*360.0f;
//		//PID_Calc_Speed(&motor_9025_yaw.LK_9025_speed,motor_9025_yaw.target_speed,motor_9025_yaw.now_speed);
//		PID_Calc_Speed(&motor_9025_yaw.LK_9025_angle,motor_9025_yaw.target_angle,imu_yaw_angle);
//		PID_Calc_Speed(&motor_9025_yaw.LK_9025_speed,motor_9025_yaw.target_speed,motor_9025_yaw.speed);
//		Motor_LK9025_control_FN(&hcan2,0x141,motor_9025_yaw.LK_9025_speed.output);
        vTaskDelayUntil(&currentTime,5);//绝对延时
    }

}