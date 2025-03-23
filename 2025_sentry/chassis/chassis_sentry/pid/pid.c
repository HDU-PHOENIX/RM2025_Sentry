#include "pid.h"
#include "arm_math.h"
#include "math.h"


//pid_struct_t chassis_speed_pid_a,chassis_speed_pid_b,chassis_speed_pid_c,chassis_speed_pid_d;
//pid_struct_t motor_shoot_pid_a,motor_shoot_pid_b;
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

//static float VariableIntegralCoefficient(float error, float absmax, float absmin);
/*参数初始化--------------------------------------------------------------*/
void pid_param_init(PidTypeDef *pid, uint8_t mode, float PID[3], float max_out, float max_iout,  float I_Separation, float Dead_Zone, float gama, float angle_max, float angle_min)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    if(fabs(pid->Dead_Zone) < 1e-5)
        pid->Dead_Zone = 0;
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
    pid->I_Separation = I_Separation;
    pid->Dead_Zone = Dead_Zone;
    pid->gama = gama;
    pid->angle_max = angle_max;
    pid->angle_min = angle_min;
}

float pid_caculate(PidTypeDef *pid, const float ref, const float set)
{
  	uint8_t index;
	if (pid == NULL||isnan(set)||isnan(ref)||isnan(pid->out))
	{
		pid->out = 0;
			return 0.0f;
		
	}
	pid->error[0] = set - ref;
	if(isnan(pid->error[0]) || isinf(pid->error[0]))
	{
		pid->out = 0;
			return 0.0f;
	}
	if(fabsf(pid->error[0]) < pid->Dead_Zone)
	{
		pid->out = 0;
		return 0.0f;
	}
	if(pid->angle_max!=pid->angle_min)
	{
		if( pid->error[0]>(pid->angle_max+pid->angle_min)/2)
			 pid->error[0]-=(pid->angle_max+pid->angle_min);
		else if( pid->error[0]<-(pid->angle_max+pid->angle_min)/2)
			 pid->error[0]+=(pid->angle_max+pid->angle_min);
	}
    if(fabs(pid->error[0])>pid->I_Separation)//误差过大，采用积分分离
    {
    	index=0;
    }
    else
    {
    	index=1;
    }
		
	pid->set = set;
	pid->fdb = ref;

    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout = (pid->Iout + pid->Ki * pid->error[0])*index;
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout=pid->Kd*(1- pid-> gama)*(pid->Dbuf[0])+pid-> gama* pid-> lastdout;
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }

	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];
	pid-> lastdout=pid->Dout;
    return pid->out;
}

void PID_clear(PidTypeDef *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

/*中途更改参数设定(调试)------------------------------------------------------------*/
void pid_reset(PidTypeDef	*pid, float PID[3])
{
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
}
void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,   //积分限幅
              float out_max)//PID?????
{
  pid->kp      = kp;
  pid->ki      = ki;
  pid->kd      = kd;
  pid->i_max   = i_max;
  pid->out_max = out_max;
}
 
float pid_calc(pid_struct_t *pid, float ref, float fdb)//target,feedback
{
	pid->ref = ref;
	pid->fdb = fdb;
	pid->err[1] = pid->err[0];
	pid->err[0] = pid->ref - pid->fdb;
  
	pid->p_out  = pid->kp * pid->err[0];
	pid->i_out += pid->ki * pid->err[0];
	pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
	
	LimitMax(pid->i_out, pid->i_max);
	
	pid->output = pid->p_out + pid->i_out + pid->d_out;
	
	LimitMax(pid->output, pid->out_max);
	
	HAL_Delay(5);
  return pid->output;
}
float Limit_Min_Max(float value,float min,float max)
{
	if(value<min)
		return min;
	else if(value>max)
		return max;
	else return value;
}
void Pid_Protect(pid_struct_t *pid)
{
	if(pid->ref - pid->fdb > 180)
	{
		pid->fdb+=360;
	}
	else if(pid->ref - pid->fdb < -180)
	{
		pid->fdb-=360;
	}
}

float PID_Calc_Angle(pid_struct_t *PID, float ref, float fdb)//
{
  PID->ref = ref;
  PID->fdb = fdb;

	Pid_Protect(PID);//过零点保护
	
	uint8_t index;
	
  if(fabs(PID->err[0]) >= PID-> i_seperate)
	  index = 0;
  else 
	  index = 1;
	if(PID->i_seperate==0){
	  index = 1;
  }
	
  PID->err[1] = PID->err[0];
  PID->err[0] = PID->ref - PID->fdb;
  
  PID->p_out  = PID->kp * PID->err[0];
  PID->i_out += (PID->ki * PID->err[0]* index);
  PID->d_out  = PID->kd * (PID->err[0] - PID->err[1]);
  PID->i_out=Limit_Min_Max(PID->i_out, -PID->i_max, PID->i_max);
  
  PID->output = PID->p_out + PID->i_out + PID->d_out;
  PID->output=Limit_Min_Max(PID->output, -PID->out_max, PID->out_max);
  return PID->output;
}
float PID_Calc_Speed(pid_struct_t *PID, float ref, float fdb)//PIDÔËËãº¯Êý£¨Ä¿±ê£¬Êµ¼Ê£©
{
  PID->ref = ref;
  PID->fdb = fdb;
	uint8_t index;
	
  if(fabs(PID->err[0]) >= PID-> i_seperate)
	  index = 0;
  else 
	  index = 1;
  if(PID->i_seperate==0){
	  index = 1;
  }
  PID->err[1] = PID->err[0];
  PID->err[0] = PID->ref - PID->fdb;
  
  PID->p_out  = PID->kp * PID->err[0];
  PID->i_out += (PID->ki * PID->err[0]* index);
  PID->d_out  = PID->kd * (PID->err[0] - PID->err[1]);
  PID->i_out=Limit_Min_Max(PID->i_out, -PID->i_max, PID->i_max);
  
  PID->output = PID->p_out + PID->i_out + PID->d_out;
  PID->output=Limit_Min_Max(PID->output, -PID->out_max, PID->out_max);
	//HAL_Delay(1);
  return PID->output;
}
