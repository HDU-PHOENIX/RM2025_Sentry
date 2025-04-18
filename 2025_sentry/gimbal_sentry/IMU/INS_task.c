/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             主要利用陀螺仪bmi088，磁力计ist8310，完成姿态解算，得出欧拉角，
  *             提供通过bmi088的data ready 中断完成外部触发，减少数据等待延迟
  *             通过DMA的SPI传输节约CPU时间.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088, but don't support mpu6500
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  //---------------------------下面是抽象的许铁头在讲话↓-----------------------------------//
  *2022.7.02 天气热的要命 心情很不美丽
  *1、现在的代码yaw轴存在零飘情况 如果启用磁力计能解决，但是磁力计收敛很慢并且前期的零
  *飘十分抽象（该车一次debug值更新零飘1度多，十分抽象）
  *2、启用磁力计就是初始化磁力计并且把ist8310_read_mag(ist8310_real_data.mag)函数取消注释
  *如果你好奇 可以取消注释看看效果 然后就能理解我为什么不启用了（阿门）
  *3、对比官方代码会发现官方例程里gyro_update_flag用了IMU_NOTIFY_SHFITS
  *但是相信我 不是这里抄错了用成了IMU_UPDATE_SHFITS 如果你好奇为什么 可以改回去试试（微笑.jpg）
  *4、那个官方所言的解决零飘的代码没个卵用 你只要细看代码 就会发现值根本没传出来（不过官方宏定义了
  *这个函数并且在校准函数里用了他 但是并没有调用那个校准函数 总之就是官方摆烂例程没写也不知道他
  *到底在干什么于是放弃使用了）
  *5、珍爱生命 远离陀螺仪 阿门
  */
	//--------------------------2022.8.30----------------------------------------------------//
	/*1、用磁力计解决了零漂问题，但出了很多其他问题：
	第一磁力计需要校准，多取几组数据matlab画个图，看一下中心偏多少纠正过来就行了。
	第二磁力计受外就磁场干扰严重，有个贴片靠近C板5cm左右就会飞掉。
	磁力计就交给下一位有缘人了，我要用线性矫正了。
	*/
	
	/*
	尝试使用卡尔曼滤波解决零漂问题
	
	*/
	
	
	
	

#include "INS_task.h"

#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "bsp_imu_pwm.h"
#include "bmi088driver.h"
#include "ist8310driver.h"
#include "bsp_spi.h"
#include "../PID/pid.h"
#include "LK_9025.h"
#include  "baspcan.h"
#include "vision.h"
//#include "GimbleTask.h"
#include "DM4310.h"
#include "motor_define.h"
#include "MahonyAHRS.h"
#include "arm_math.h"


#define    TIME_STAMP_1MS        1
#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm给定
#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
    {0.0f, 1.0f, 0.0f},                     \
    {-1.0f, 0.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \


#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \
	
	float yaw_imu;		
/**
  * @brief          旋转陀螺仪,加速度计和磁力计,并计算零漂,因为设备有不同安装方式
  * @param[out]     gyro: 加上零漂和旋转
  * @param[out]     accel: 加上零漂和旋转
  * @param[out]     mag: 加上零漂和旋转
  * @param[in]      bmi088: 陀螺仪和加速度计数据
  * @param[in]      ist8310: 磁力计数据
  * @retval         none
  */
//static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310);

/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_temp_control(fp32 temp);

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_cmd_spi_dma(void);


fp32 get_yaw_ist8310(ist8310_real_data_t *ist8310,fp32 pitch,fp32 roll);
void AHRS_init(fp32 quat[4], fp32 accel[3], fp32 mag[3]);
void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3], fp32 mag[3]);
void get_angle(fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll);

extern SPI_HandleTypeDef hspi1;

static TaskHandle_t INS_task_local_handler;

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2, 0xFF, 0xFF, 0xFF};



volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;
volatile bool_t imu_init_finish_flag = 0;
volatile bool_t gimbal_return_finish_flag = 0;

bmi088_real_data_t bmi088_real_data;
ist8310_real_data_t ist8310_real_data;

fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 gyro_offset[3];
fp32 gyro_cali_offset[3];

fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];
fp32 accel_cali_offset[3];

fp32 mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
fp32 mag_offset[3];
fp32 mag_cali_offset[3];

static uint8_t first_temperate;
static fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static PidTypeDef imu_temp_pid;
static const float timing_time = 0.001f;   //tast run time , unit s.任务运行的时间 单位 s

//加速度计低通滤波
static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};

fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.欧拉角 单位 rad
fp32 IMU_angle[3] = {0.0f, 0.0f, 0.0f};
fp32 Last_ol[3];

float *IMU_kf_result;
float yaw_add_one_tick=-0.00004;//-0.000000005769f
float yaw_offset;
				//消零飘
int pre_tick = 0;
int tick;
union IMUToBytes {
    struct {
        float floatValue1;
    } floats;
    uint8_t bytes[sizeof(float)]; // ??float??????
};
union IMUToBytes IMU_CAN;


extern computer_all computer_from;
extern uint8_t s[2];

float angle_init=130;


extern float angle_for_9025;

float yaw_zimiao=0;
int yaw_flag=1;

int gyro_flag=1;
float gyro_angle=0;

extern motor_6020_t motor_yaw;

float angle_speed_9025=0;

void IMU_InitArgument(void)
{
    while(BMI088_init())
    {
        osDelay(100);
    }
		while(ist8310_init())
		{
			osDelay(100);
		}
    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
    pid_param_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT,3e38,0,0,0,0);

    AHRS_init(INS_quat, bmi088_real_data.accel, ist8310_real_data.mag);

    accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = bmi088_real_data.accel[0];//p
    accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = bmi088_real_data.accel[1];//yoll
    accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = bmi088_real_data.accel[2];//z

    //get the handle of task
    //获取当前任务的任务句柄，
    INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));

    //set spi frequency
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }

    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

    imu_start_dma_flag = 1;
}
/**
  * @brief          imu task, init bmi088, ist8310, calculate the euler angle
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          imu任务, 初始化 bmi088, ist8310, 计算欧拉角
  * @param[in]      pvParameters: NULL
  * @retval         none
  */

float add_angle_yaw = 0;

void pid_caluate_gimbal_imu(){ //顺序很重要，必须在pid计算前面
	
	PID_Calc_Angle(&(motor_yaw.Angle_PID),motor_yaw.Set_Angle,motor_yaw.rotor_angle);	
    PID_Calc_Speed(&(motor_yaw.Speed_PID),motor_yaw.Angle_PID.output,motor_yaw.rotor_speed);
	//PID_Calc_Speed(&(motor_yaw.Speed_PID),target_speed_yaw,motor_yaw.rotor_speed);
	
	
	//PID_Calc_Angle(&gimbal_4310_angle,DM4310_pitch.target_angle,DM4310_pitch.angle);
	//PID_Calc_Speed(&gimbal_4310_speed,gimbal_4310_angle.output,DM4310_pitch.speed_rpm);
	//PID_Calc_Speed(&gimbal_4310_speed,DM4310_pitch.target_speed_rpm,DM4310_pitch.speed_rpm);
}

void motor_6020_sent_imu(){
		CAN_TxHeaderTypeDef tx_header;
		uint8_t             tx_data[8] = {0};
    
		tx_header.StdId = 0x1FF;//标识符（见手册P6）
		tx_header.IDE   = CAN_ID_STD;//标准ID
		tx_header.RTR   = CAN_RTR_DATA;//数据帧
		tx_header.DLC   = 8;//字节长度
		tx_data[4] = ((int16_t)motor_yaw.Speed_PID.output>>8)&0xff;
		tx_data[5] = ((int16_t)motor_yaw.Speed_PID.output)&0xff;
		
		if(HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0)!=HAL_OK){
				if(HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX1)!=HAL_OK){
					HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX2);
				}
		
		}
		
		
		
		if(hcan2.ErrorCode==HAL_CAN_STATE_LISTENING){
			HAL_CAN_Stop(&hcan2);  // ??CAN???
			HAL_CAN_Start(&hcan2); // ????CAN???
		
		}
		if(hcan2.ErrorCode==HAL_CAN_STATE_ERROR){
			HAL_CAN_Stop(&hcan2);  // ??CAN???
			HAL_CAN_Start(&hcan2); // ????CAN??
		
		}
		
		if(hcan1.ErrorCode==HAL_CAN_STATE_LISTENING){
			HAL_CAN_Stop(&hcan1);  // ??CAN???
			HAL_CAN_Start(&hcan1); // ????CAN???
		
		}
		if(hcan1.ErrorCode==HAL_CAN_STATE_ERROR){
			HAL_CAN_Stop(&hcan1);  // ??CAN???
			HAL_CAN_Start(&hcan1); // ????CAN??
		
		}
}

void INS_task(void const *pvParameters)
{
    uint16_t count = 0;
		portTickType currentTime;
    IMU_InitArgument();
    while(1)
    {
				currentTime = xTaskGetTickCount(); //当前系统时间
        //wait spi DMA tansmit done
		position_speed_control(&hcan1,0x01,DM4310_pitch.target_angle,6);
        //等待SPI DMA传输
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
        }

        if(!imu_init_finish_flag)
            count++;

        if(!imu_init_finish_flag && count > 500)
        {
            imu_init_finish_flag = 1;
            count = 700;
        }

        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
        }

        if(accel_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
        }

        if(accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
            imu_temp_control(bmi088_real_data.temp);
        }
        //加速度计低通滤波
        /*accel_fliter_1[0] = accel_fliter_2[0];
        accel_fliter_2[0] = accel_fliter_3[0];

        accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + bmi088_real_data.accel[0] * fliter_num[2];

        accel_fliter_1[1] = accel_fliter_2[1];
        accel_fliter_2[1] = accel_fliter_3[1];

        accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + bmi088_real_data.accel[1] * fliter_num[2];

        accel_fliter_1[2] = accel_fliter_2[2];
        accel_fliter_2[2] = accel_fliter_3[2];

        accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + bmi088_real_data.accel[2] * fliter_num[2];*/
				
//				imu_cali_slove(bmi088_real_data.gyro,accel_fliter_3,ist8310_real_data.mag,&bmi088_real_data,&ist8310_real_data);
        //AHRS_update(INS_quat, timing_time, bmi088_real_data.gyro, accel_fliter_3, ist8310_real_data.mag);
		AHRS_update(INS_quat, timing_time, bmi088_real_data.gyro, accel_fliter_3, ist8310_real_data.mag);

        get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);
        //get_angle(INS_quat, INS_angle, INS_angle, INS_angle);
        tick = HAL_GetTick();
				if (pre_tick)
				yaw_offset +=  yaw_add_one_tick * (tick - pre_tick);
				
				//yaw_offset =  -yaw_offset;
				
        IMU_angle[0] = 180.0f*INS_angle[0]/PI + yaw_offset; //yaw   yaw_offset
		//IMU_angle[0] = 180.0f*INS_angle[0]/PI;
				yaw_imu=IMU_angle[0];
        IMU_angle[1] = 180.0f*INS_angle[1]/PI; //pitch
        IMU_angle[2] = 180.0f*INS_angle[2]/PI; //roll
		//IMU_angle[0]=IMU_angle[0]+180;
				pre_tick = tick; 
				
				IMU_CAN.floats.floatValue1=IMU_angle[0];
      
		
				
		
			if(s[0]==1){  //自瞄模式
				
										/****************************************小陀螺*******************************************/	

		angle_for_9025=computer_from.zimiao.big_yaw/6.28318*360;					
//		if(s[1]==1 || s[1]==2)	{
	if(computer_from.computer_exercise.type==1)
	{
									
				if(gyro_flag==1){
					yaw_zimiao=yaw_imu;
					gyro_flag=0;
				}
								
	if(computer_from.zimiao.fine_bool== '1' || computer_from.zimiao.fine_bool== '2'){
				//(angle_for_9025-yaw_imu)*30
		//yaw_zimiao=yaw_zimiao+angle_for_9025;
		if(angle_for_9025+yaw_zimiao<yaw_imu+25 && angle_for_9025+yaw_zimiao>yaw_imu-25){
					motor_yaw.Set_Angle=(computer_from.zimiao.big_yaw-0)/6.2831852*8191+2674;
					angle_speed_9025=40000+(yaw_zimiao-yaw_imu)*7500;
					if(angle_speed_9025>60000){
						angle_speed_9025=60000;
					}
					Motor_LK9025_control_speed(&hcan2,0x141,angle_speed_9025);
			}
		else{
					motor_yaw.Set_Angle=2674;
					angle_speed_9025=40000+(yaw_zimiao-yaw_imu)*7500;
					if(angle_speed_9025>60000){
						angle_speed_9025=60000;
					}
					Motor_LK9025_control_speed(&hcan2,0x141,angle_speed_9025);
		}
			
//			if(angle_for_9025+yaw_zimiao>yaw_imu+15){
//					angle_speed_9025=40000+(yaw_zimiao+angle_for_9025-yaw_imu)*7000;
//				
////					if(angle_speed_9025<24000){
////						angle_speed_9025=24000;
////					
////					}
//						Motor_LK9025_control_speed(&hcan2,0x141,angle_speed_9025);
//						}
//			else if(angle_for_9025+yaw_zimiao<yaw_imu-15){
//					angle_speed_9025=40000+(yaw_zimiao+angle_for_9025-yaw_imu)*7000;
//						Motor_LK9025_control_speed(&hcan2,0x141,angle_speed_9025);
//					}
//			else{
//					//Motor_LK9025_control_speed(&hcan2,0x141,0);
//					angle_speed_9025=40000+(yaw_zimiao-yaw_imu)*7000;
//				
//					motor_yaw.Set_Angle=(computer_from.zimiao.big_yaw-0)/6.2831852*8191+2674;
//					Motor_LK9025_control_speed(&hcan2,0x141,angle_speed_9025);
//				}	
				
		}
		if(computer_from.zimiao.fine_bool== '0' || computer_from.zimiao.fine_bool== 0){
			angle_speed_9025=40000+(yaw_zimiao-yaw_imu)*7500;
				//gyro_flag=1;
			if(angle_speed_9025>60000){
				angle_speed_9025=60000;
			
			}
			Motor_LK9025_control_speed(&hcan2,0x141,angle_speed_9025);
		}
				
	}
	if(computer_from.computer_exercise.type==0){
					gyro_flag=1;	
					
				if(computer_from.zimiao.fine_bool== '1' || computer_from.zimiao.fine_bool== '2'){
				
				if(yaw_flag==1){
					yaw_zimiao=yaw_imu;
					yaw_flag=0;
				}
				if(angle_for_9025+yaw_zimiao>yaw_imu+40 || angle_for_9025+yaw_zimiao<yaw_imu-40){
						motor_yaw.Set_Angle=2674;
				}
				
				
				
				
				
				
				
				
				if(angle_for_9025+yaw_zimiao>yaw_imu+8){
						Motor_LK9025_control_speed(&hcan2,0x141,12000);
						}
				else if(angle_for_9025+yaw_zimiao<yaw_imu-8){
				
						Motor_LK9025_control_speed(&hcan2,0x141,-12000);
				}
				else{
					Motor_LK9025_angle_more(&hcan2,0x141,0);
					motor_yaw.Set_Angle=(computer_from.zimiao.big_yaw-0)/6.2831852*8191+2674;
				}	
				
		}if(computer_from.zimiao.fine_bool== '0' || computer_from.zimiao.fine_bool== 0){
				Motor_LK9025_angle_more(&hcan2,0x141,0);
				yaw_flag=1;
		}	
					
					
						
			}
		}
			
		if(fabs(motor_yaw.Set_Angle-motor_yaw.rotor_angle)>2000){
			
			motor_yaw.Angle_PID.i_seperate=1;
		}
		else{
			motor_yaw.Angle_PID.i_seperate=300;
		}
		
		if(motor_yaw.Set_Angle <=1750)
		{
			motor_yaw.Set_Angle = 1750;		
		}
		if(motor_yaw.Set_Angle >= 3451)
		{
			motor_yaw.Set_Angle =3451;
		}		
		pid_caluate_gimbal_imu();
		motor_6020_sent_imu();
					CAN_TxHeaderTypeDef tx_header;
						uint8_t             tx_data[8];
						
						memcpy(tx_data,&yaw_imu,sizeof(float));  //前四位传送的是x方向的速度
						
						//memcpy(tx_data+sizeof(float),&computer_from.computer_vpm.y_speed,sizeof(float)); //后四位传的是y方向的速度
						
						tx_header.StdId = 0x404;//标识符（见手册P6）  //
						tx_header.IDE   = CAN_ID_STD;//标准ID
						tx_header.RTR   = CAN_RTR_DATA;//数据帧
						tx_header.DLC   = 8;//字节长度

						HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);	
				
				
			//ttcm	
				
				
        vTaskDelayUntil(&currentTime, TIME_STAMP_1MS);
    }
}
fp32 get_yaw_ist8310(ist8310_real_data_t *ist8310,fp32 pitch,fp32 roll)
{
	fp32 mx,my;
	mx = ist8310->mag[0] * cos(pitch) + ist8310->mag[1] * sin(pitch) * sin(roll) - ist8310->mag[2] * sin(pitch) * cos(roll);
	my = ist8310->mag[1] * cos(roll) + ist8310->mag[2] * sin(roll);
	return atan2(my,mx);
}


void AHRS_init(fp32 quat[4], fp32 accel[3], fp32 mag[3])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;

}

/**
  * @brief          计算陀螺仪零漂
  * @param[out]     gyro_offset:计算零漂
  * @param[in]      gyro:角速度数据
  * @param[out]     offset_time_count: 自动加1
  * @retval         none
  */
void gyro_offset_calc(fp32 gyro_offset[3], fp32 gyro[3], uint16_t *offset_time_count)
{
    if (gyro_offset == NULL || gyro == NULL || offset_time_count == NULL)
    {
        return;
    }

        gyro_offset[0] = gyro_offset[0] - 0.0003f * gyro[0];
        gyro_offset[1] = gyro_offset[1] - 0.0003f * gyro[1];
        gyro_offset[2] = gyro_offset[2] - 0.0003f * gyro[2];
        (*offset_time_count)++;
}

/**
  * @brief          calculate gyro zero drift
  * @param[out]     cali_scale:scale, default 1.0
  * @param[out]     cali_offset:zero drift, collect the gyro ouput when in still
  * @param[out]     time_count: time, when call gyro_offset_calc 
  * @retval         none
  */
/**
  * @brief          校准陀螺仪
  * @param[out]     陀螺仪的比例因子，1.0f为默认值，不修改
  * @param[out]     陀螺仪的零漂，采集陀螺仪的静止的输出作为offset
  * @param[out]     陀螺仪的时刻，每次在gyro_offset调用会加1,
  * @retval         none
  */
void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count)
{
        if( *time_count == 0)
        {
            gyro_offset[0] = gyro_cali_offset[0];
            gyro_offset[1] = gyro_cali_offset[1];
            gyro_offset[2] = gyro_cali_offset[2];
        }
        gyro_offset_calc(gyro_offset, INS_gyro, time_count);

        cali_offset[0] = gyro_offset[0];
        cali_offset[1] = gyro_offset[1];
        cali_offset[2] = gyro_offset[2];
        cali_scale[0] = 1.0f;
        cali_scale[1] = 1.0f;
        cali_scale[2] = 1.0f;

}

void AHRS_update(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3], fp32 mag[3])
{
    MahonyAHRSupdateIMU(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
	
}

//进行四元素解算求得欧拉角
void get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll)
{
    *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
    *pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
    *roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
}
/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        pid_caculate(&imu_temp_pid, temp, bmi088_real_data.temp);
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
        //在没有达到设置的温度，一直最大功率加热
        //in beginning, max power
        if (temp > bmi088_real_data.temp)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                //达到设置温度，将积分项设置为一半最大功率，加速收敛
                //
                first_temperate = 1;
                imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }

        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == INT1_ACCEL_Pin)
    {
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == INT1_GYRO_Pin)
    {
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == DRDY_IST8310_Pin)
    {
        mag_update_flag |= 1 << IMU_DR_SHFITS;

        if(mag_update_flag &= 1 << IMU_DR_SHFITS)
        {
            mag_update_flag &= ~(1 << IMU_DR_SHFITS);
            mag_update_flag |= (1 << IMU_SPI_SHFITS);

//            ist8310_read_mag(ist8310_real_data.mag);
        }
    }
    else if(GPIO_Pin == GPIO_PIN_0)
    {
        //wake up the task
        //唤醒任务
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
    else if(GPIO_Pin == GPIO_PIN_1)
    {
        gimbal_return_finish_flag = 1;
    }
}

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_cmd_spi_dma(void)
{
    //开启陀螺仪的DMA传输
    if( (gyro_update_flag & (1 << IMU_DR_SHFITS) ) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
            && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
        gyro_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
        return;
    }
    //开启加速度计的DMA传输
    if((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
            && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
        return;
    }

    if((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
            && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
        return;
    }
}
uint32_t count_accel = 0;
uint32_t count_gyro = 0;
uint32_t count_temp = 0;
void ins_dma_data_read(void)
{
    if(__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        //gyro read over
        //陀螺仪读取完毕
        if(gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);
						count_gyro++;
            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
        }

        //accel read over
        //加速度计读取完毕
        if(accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);
						count_accel++;
            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        //temperature read over
        //温度读取完毕
        if(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);
						count_temp++;
            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }

        imu_cmd_spi_dma();

        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
        }
    }
}


