#ifndef __pid_h__
#define __pid_h__
#include "main.h"


typedef enum
{
    PID_POSITION = 0,//位置式PID
    PID_DELTA, //增量式PID
} PID_MODE;

typedef struct PidTypeDef
{
    float Dead_Zone; //误差死区阈值
    uint8_t mode;
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出

    float set; //设定值
    float fdb; //反馈值

    float out;
    float lastout;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次
    int angle_max;
    int angle_min;	//角度相邻值 如在一个圆内，0°和360°相邻，则max=360，min=0
    //			在一个电机内 0和8192相邻，则max=8192，min=0
    float I_Separation; //积分分离阈值
    float gama;			//微分先行滤波系数
    float lastdout;		//上一次微分输出


} PidTypeDef;
typedef struct pid_struct_t
{
  float kp;//??
  float ki;//??
  float kd;//??
  float i_max;//????
  float out_max;//????
  float i_seperate;
float d_seperate;
	
  float ref;      // target value
  float fdb;      // feedback value
  float err[2];   // error and last error??
 
  float p_out;//????
  float i_out;//????
  float d_out;//????
  float output;//pid???
}pid_struct_t;
 
void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max);
//void gimbal_PID_init(void);			  
float pid_calc(pid_struct_t *pid, float ref, float fdb); 
extern float pid_caculate(PidTypeDef *pid, const float ref, const float set);
void PID_clear(PidTypeDef *pid);
void pid_reset(PidTypeDef	*pid, float PID[3]);
extern void pid_param_init(PidTypeDef *pid, uint8_t mode, float PID[3], float max_out, float max_iout, float I_Separation, float Dead_Zone, float gama, float angle_max, float angle_min);
int Limit_Min_Max(int value,int min,int max);
void Pid_Protect(pid_struct_t *pid);
float PID_Calc_Angle(pid_struct_t *PID, float ref, float fdb);
float PID_Calc_Speed(pid_struct_t *PID, float ref, float fdb);
float float_Limit_Min_Max(float value,float min,float max);
#endif
