#include "main.h"
#include "motor_define.h"
#include "remote.h"
#include "math.h"
#include "pid.h"
#include "can.h"
#include "LK_9025.h"
#include "judge.h"
#include "INS_task.h"
#include "baspcan.h"

#define Gimbal_Yaw_ZERO 85.5468//15.66 
#define WHEEL_PERIMETER 152 //mm底盘周长
#define CHASSIS_DECELE_RATIO 19 //减速比
#define Speed_Set 2000//速度
#define Gimbal_length 25 //mm底盘半径

Chassis_Speed_t Temp_Chassis_Speed;
Chassis_Speed_t Chassis_Speed;

extern RC_Ctl_t RC_CtrlData;
extern motor_3508_t motor_3508[4];
extern motor_6020_t motor_yaw;
extern float Auto_speed_x,Auto_speed_y;
extern bool Judge_Data_TF;

extern motor_9025 motor_9025_yaw;
extern float target_anlge;
extern float test_9025;


#define  CHAS_CURRENT_LIMIT        40000    //四个轮子的速度总和最大值,单个输出*4,限功率调比例可用
float WARNING_REMAIN_POWER = 42;//裁判系统剩余焦耳能量低于这个数值则开始限功率,40扭屁股会超功率,平地开不会超
float Power_Limit_Gain = 1.0f;




float chassis_totaloutput;
float Joule_Residue = 0;//剩余焦耳缓冲能量
int16_t judgDataCorrect = 0;//裁判系统数据是否可用
static int32_t judgDataError_Time = 0;
float fTotalCurrentLimit;//电流分配,平地模式下分配是均匀的
float fChasCurrentLimit = CHAS_CURRENT_LIMIT;//限制4个轮子的速度总和

float output_power=0,out_give_chassis[4]={0};

extern uint8_t xiaotuoluo_flag;

float Angle;
float errr;


    uint16_t max_power_limit=100;
    float chassis_max_power=0;
    float initial_give_power[4];
    float initial_total_power=0;
    float scaled_give_power[4];
    float chassis_power=0.0f;
    float chassis_power_buffer=0.0f;
    float toque_coefficient=1.99688994e-6f;
    float k1=1.453e-07;
    //????k2   1.453e-07;  1.255e-07;           1.653e-07
    float k2=1.23e-07;
    //????k1   1.23e-07;    1.44e-07;           1.43e-07
    float constant=0.5825f;
	
	float temp_chassis=0;
	
extern fp32 IMU_angle[3];

extern float low_yaw_angle;
void Chassis_Power_Limit(void)
{
    //             4.081f;       3.343;?????   2.081f  ????,?????????
    //get_chassis_power_and_buffer(&chassis_power,&chassis_power_buffer);
        max_power_limit=100;
		chassis_max_power=60;
    
    initial_total_power=0;
//        if(chassis_power_buffer<30)
//        chassis_max_power = max_power_limit -5;
//        else if(chassis_power_buffer<10)
//            chassis_max_power = max_power_limit -15;
//        else
//            chassis_max_power = max_power_limit ;

        
    for(uint8_t i=0;i<4;i++)
    {
        initial_give_power[i]=motor_3508[i].PID.output*toque_coefficient*motor_3508[i].rotor_speed
        +k2*motor_3508[i].rotor_speed//*motor_3508[i].rotor_speed
        +k1*motor_3508[i].PID.output*motor_3508[i].PID.output+constant
		;
    
    
    if(initial_give_power[i]<0)
        continue;
    initial_total_power+=initial_give_power[i];
	//initial_total_power=initial_total_power+constant;
    }
    //initial_total_power=initial_total_power+constant;
	
    if(initial_total_power>chassis_max_power)
    {
    float power_scale =chassis_max_power/initial_total_power;
    for(uint8_t i=0;i<4;i++)
        {
            scaled_give_power[i]=initial_give_power[i]*power_scale;
           if(scaled_give_power[i]<0)
              continue;
              
               float b=toque_coefficient * motor_3508[i].rotor_speed;
               //float c=k2*motor_3508[i].rotor_speed*motor_3508[i].rotor_speed-scaled_give_power[i]+constant;
			float c=k2*motor_3508[i].rotor_speed*motor_3508[i].rotor_speed-scaled_give_power[i]+constant;
		   //c=-c;
        if(motor_3508[i].PID.output>0) //
            {
             temp_chassis=(-b+sqrt(b*b-4*k1*c))/(2*k1);  // float temp=(-b+sqrt(b*b-4*k1*c))/(2*k1);// float b=toque_coefficient * Motor_3508[i].speed_rpm;
               //float c=k2*Motor_3508[i].speed_rpm*Motor_3508[i].speed_rpm-scaled_give_power[i]+constant;
                if(temp_chassis>16000)

                {
                  motor_3508[i].PID.output=16000;  
                }
                else motor_3508[i].PID.output=temp_chassis;
                
            }
                else 
                {
           temp_chassis=(-b-sqrt(b*b-4*k1*c))/(2*k1);
                if(temp_chassis<-16000)
                {
                motor_3508[i].PID.output=-16000;
                }
                else motor_3508[i].PID.output=temp_chassis;
                }
      }
   }
	output_power=0;
	for(uint8_t i=0;i<4;i++)
    {
        out_give_chassis[i]=motor_3508[i].PID.output*toque_coefficient*motor_3508[i].rotor_speed
        +k2*motor_3508[i].rotor_speed
        +k1*motor_3508[i].PID.output*motor_3508[i].PID.output*toque_coefficient*toque_coefficient+constant
        ;
    
    
    if(initial_give_power[i]<0)
        continue;
    output_power+=initial_give_power[i];
    }
	//output_power=output_power+constant;
}


void Chassis_Remote_Control_normal()
{
        Temp_Chassis_Speed.vx = (float)RC_CtrlData.rc.ch3/400;
        Temp_Chassis_Speed.vy = (float)RC_CtrlData.rc.ch2/400;  //正常模式
        Temp_Chassis_Speed.vw = 0;
}
void Chassis_Remote_Control_gyro()
{
	Temp_Chassis_Speed.vx = (float)RC_CtrlData.rc.ch3/360;//数值得大于287！！！不然超范围了
    Temp_Chassis_Speed.vy = (float)RC_CtrlData.rc.ch2/360;
	Temp_Chassis_Speed.vw = 1;
}
float Find_Angle()
{
	float Angle,Zero;
	Angle = motor_9025_yaw.now_angle;
	Zero = Gimbal_Yaw_ZERO;
	
	if(Angle - Zero > 180)
	{
		Zero+=360;
	}
	else if(Angle - Zero < -180)
	{
		Zero-=360;
	}
	
    errr = Angle - Zero;
	float temp1 = errr * 2 * 3.1415926 / 360;
    if(temp1 > 3.141593f)
        temp1 = 3.141593f;
    else if(temp1 < -3.141593f)
        temp1 = -3.141593f;
    return temp1;
}

float Find_Angle_Auto()
{
	float Angle,Zero;
	low_yaw_angle=low_yaw_angle/6.28*360;
	Angle = motor_9025_yaw.now_angle+low_yaw_angle;
	Zero = Gimbal_Yaw_ZERO;
	
	if(Angle - Zero > 180)
	{
		Zero+=360;
	}
	else if(Angle - Zero < -180)
	{
		Zero-=360;
	}
	
    errr = Angle - Zero;
	float temp1 = errr * 2 * 3.1415926 / 360;
    if(temp1 > 3.141593f)
        temp1 = 3.141593f;
    else if(temp1 < -3.141593f)
        temp1 = -3.141593f;
    return temp1;
}
void Chassis_Solution()
{
    Angle = Find_Angle();
	//Angle = Find_Angle_Auto();
    Chassis_Speed.vx = Temp_Chassis_Speed.vx * cos(Angle) + Temp_Chassis_Speed.vy * sin(Angle);
	Chassis_Speed.vy = -Temp_Chassis_Speed.vx * sin(Angle) + Temp_Chassis_Speed.vy * cos(Angle);
	
//	Chassis_Speed.vx = Temp_Chassis_Speed.vx * cos(Angle) - Temp_Chassis_Speed.vy * sin(Angle);
//	Chassis_Speed.vy = Temp_Chassis_Speed.vx * sin(Angle) + Temp_Chassis_Speed.vy * cos(Angle);

    Chassis_Speed.vw = Temp_Chassis_Speed.vw;
}
void Chassis_Motor_Solution()
{
    float Wheel_Rpm_Ratio = 60.0f/(WHEEL_PERIMETER*3.14f) * CHASSIS_DECELE_RATIO * 10000;//=23885

	if(RC_CtrlData.rc.s1==2){
		Chassis_Speed.vw=0;
		motor_3508[0].Set_Speed = Chassis_Speed.vx * 0.7071068f * Speed_Set + Chassis_Speed.vy * 0.7071068f * Speed_Set;
		motor_3508[1].Set_Speed = Chassis_Speed.vx * 0.7071068f * Speed_Set - Chassis_Speed.vy * 0.7071068f * Speed_Set;
		motor_3508[2].Set_Speed = -Chassis_Speed.vx * 0.7071068f * Speed_Set - Chassis_Speed.vy * 0.7071068f * Speed_Set;
		motor_3508[3].Set_Speed = -Chassis_Speed.vx * 0.7071068f * Speed_Set + Chassis_Speed.vy * 0.7071068f * Speed_Set;	
		if(RC_CtrlData.rc.s2==2){
			motor_3508[0].Set_Speed = Chassis_Speed.vx * 0.7071068f * Speed_Set + Chassis_Speed.vy * 0.7071068f * Speed_Set +RC_CtrlData.rc.ch0*2;
			motor_3508[1].Set_Speed = Chassis_Speed.vx * 0.7071068f * Speed_Set - Chassis_Speed.vy * 0.7071068f * Speed_Set +RC_CtrlData.rc.ch0*2;
			motor_3508[2].Set_Speed = -Chassis_Speed.vx * 0.7071068f * Speed_Set - Chassis_Speed.vy * 0.7071068f * Speed_Set +RC_CtrlData.rc.ch0*2;
			motor_3508[3].Set_Speed = -Chassis_Speed.vx * 0.7071068f * Speed_Set + Chassis_Speed.vy * 0.7071068f * Speed_Set +RC_CtrlData.rc.ch0*2;
		}
		if(RC_CtrlData.rc.s2==1){
			motor_3508[0].Set_Speed = Chassis_Speed.vx * 0.7071068f * Speed_Set + Chassis_Speed.vy * 0.7071068f * Speed_Set +4000;
			motor_3508[1].Set_Speed = Chassis_Speed.vx * 0.7071068f * Speed_Set - Chassis_Speed.vy * 0.7071068f * Speed_Set +4000;
			motor_3508[2].Set_Speed = -Chassis_Speed.vx * 0.7071068f * Speed_Set - Chassis_Speed.vy * 0.7071068f * Speed_Set +4000;
			motor_3508[3].Set_Speed = -Chassis_Speed.vx * 0.7071068f * Speed_Set + Chassis_Speed.vy * 0.7071068f * Speed_Set +4000;//3800
			

		}
	}
	if(RC_CtrlData.rc.s2==0 || RC_CtrlData.rc.s1==0){
		motor_3508[0].Set_Speed = 0;
		motor_3508[1].Set_Speed = 0;
		motor_3508[2].Set_Speed = 0;
		motor_3508[3].Set_Speed = 0;
	
	}

}
void Chassis_PID_Calc()
{
    PID_Calc_Speed(&motor_3508[0].PID,motor_3508[0].Set_Speed,motor_3508[0].rotor_speed);
    PID_Calc_Speed(&motor_3508[1].PID,motor_3508[1].Set_Speed,motor_3508[1].rotor_speed);
    PID_Calc_Speed(&motor_3508[2].PID,motor_3508[2].Set_Speed,motor_3508[2].rotor_speed);
    PID_Calc_Speed(&motor_3508[3].PID,motor_3508[3].Set_Speed,motor_3508[3].rotor_speed);    
}
void Set_M3508_Chassis_Voltage(CAN_HandleTypeDef* hcan,motor_3508_t M3508_Chassis[4])
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t             tx_data[8] = {0};
    
    tx_header.StdId = 0x200;//标识符（见手册P6）
    tx_header.IDE   = CAN_ID_STD;//标准ID
    tx_header.RTR   = CAN_RTR_DATA;//数据帧
    tx_header.DLC   = 8;//字节长度
		
	
    tx_data[0] = ((int16_t)M3508_Chassis[0].PID.output>>8)&0xff;
    tx_data[1] = ((int16_t)M3508_Chassis[0].PID.output)&0xff;

    tx_data[2] = ((int16_t)M3508_Chassis[1].PID.output>>8)&0xff;
    tx_data[3] = ((int16_t)M3508_Chassis[1].PID.output)&0xff;

    tx_data[4] = ((int16_t)M3508_Chassis[2].PID.output>>8)&0xff;
    tx_data[5] = ((int16_t)M3508_Chassis[2].PID.output)&0xff;

    tx_data[6] = ((int16_t)M3508_Chassis[3].PID.output>>8)&0xff;
    tx_data[7] = ((int16_t)M3508_Chassis[3].PID.output)&0xff;

    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}
void Chassis_Move()
{
    /********************************底盘解算********************************/
    Chassis_Solution();

    /********************************电机解算********************************/
    Chassis_Motor_Solution();
	//逆解算  Vx = 0.25 * (V1 + V2 + V3 + V4)， Vy = 0.25 * (-V1 + V2 - V3 + V4)	

    /********************************电机PID计算********************************/
	Chassis_PID_Calc();
		/*********************限功率部分*************************/

	Chassis_Power_Limit();
	
    Set_M3508_Chassis_Voltage(&hcan1,motor_3508);
}


void Chassis_Move_Auto()
{
	//Angle = Find_Angle_Auto();
	Angle = Find_Angle();
	
	//Auto_speed_y = -Auto_speed_y;
	
	Chassis_Speed.vx = Auto_speed_x * cos(Angle) + (-Auto_speed_y) * sin(Angle);
	Chassis_Speed.vy = -Auto_speed_x * sin(Angle) + (-Auto_speed_y) * cos(Angle);
	//Chassis_Speed.vx = Temp_Chassis_Speed.vx * cos(Angle) + Temp_Chassis_Speed.vy * sin(Angle);
	//Chassis_Speed.vy = -Temp_Chassis_Speed.vx * sin(Angle) + Temp_Chassis_Speed.vy * cos(Angle);
	
//	Chassis_Speed.vx = -Auto_speed_x;
//	Chassis_Speed.vy = -Auto_speed_y;
	
//	Chassis_Speed.vx = Temp_Chassis_Speed.vx * cos(Angle) - Temp_Chassis_Speed.vy * sin(Angle);
//	Chassis_Speed.vy = Temp_Chassis_Speed.vx * sin(Angle) + Temp_Chassis_Speed.vy * cos(Angle);
	
	
    Chassis_Speed.vw = Temp_Chassis_Speed.vw;
	
	if(xiaotuoluo_flag==0){
		
	
	motor_3508[0].Set_Speed = Chassis_Speed.vx * 0.7071068f * Speed_Set + Chassis_Speed.vy * 0.7071068f * Speed_Set;
	motor_3508[1].Set_Speed = Chassis_Speed.vx * 0.7071068f * Speed_Set - Chassis_Speed.vy * 0.7071068f * Speed_Set;
	motor_3508[2].Set_Speed = -Chassis_Speed.vx * 0.7071068f * Speed_Set - Chassis_Speed.vy * 0.7071068f * Speed_Set;
	motor_3508[3].Set_Speed = -Chassis_Speed.vx * 0.7071068f * Speed_Set + Chassis_Speed.vy * 0.7071068f * Speed_Set;	
	}
	if(xiaotuoluo_flag==1){
		
	Angle = Find_Angle();
	
	//Auto_speed_y = -Auto_speed_y;
	
	Chassis_Speed.vx = Auto_speed_x/3 * cos(Angle) + (-Auto_speed_y/3) * sin(Angle);
	Chassis_Speed.vy = -Auto_speed_x/3 * sin(Angle) + (-Auto_speed_y/3) * cos(Angle);
		
	motor_3508[0].Set_Speed = Chassis_Speed.vx * 0.7071068f * Speed_Set + Chassis_Speed.vy * 0.7071068f * Speed_Set+4000;//2500
	motor_3508[1].Set_Speed = Chassis_Speed.vx * 0.7071068f * Speed_Set - Chassis_Speed.vy * 0.7071068f * Speed_Set+4000;//2500
	motor_3508[2].Set_Speed = -Chassis_Speed.vx * 0.7071068f * Speed_Set - Chassis_Speed.vy * 0.7071068f * Speed_Set+4000; //
	motor_3508[3].Set_Speed = -Chassis_Speed.vx * 0.7071068f * Speed_Set + Chassis_Speed.vy * 0.7071068f * Speed_Set+4000;	
	}
		//0x308
	
//	motor_3508[0].Set_Speed = 3300;//2500
//	motor_3508[1].Set_Speed = 3300;//2500
//	motor_3508[2].Set_Speed = 3300; //
//	motor_3508[3].Set_Speed = 3300;	
	
	
	
	Chassis_PID_Calc();
	

	Chassis_Power_Limit();

	Set_M3508_Chassis_Voltage(&hcan1,motor_3508);
}

