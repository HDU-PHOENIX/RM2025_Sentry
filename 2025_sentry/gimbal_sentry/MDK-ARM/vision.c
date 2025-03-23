#include "main.h"



void vision_call_back_handler_Left_Control(uint8_t *buff)//对视觉接收数据进行处理
{

    if( buff[0]=='s' && buff[31]=='e' )
    {
        if(buff[1] == 0xA0) // 左云台控制
        {
            check_ok++;  //接收正确计数
            vision_missing_time = 0;  //清零失联时间
					        
					
						VisionValue.find_bool = buff[2];  //是否识别到目标
						memcpy(&(VisionValue.vision_left_yaw_value.buff[0]),&buff[3],4);  //yaw 偏移量
						memcpy(&(VisionValue.vision_left_pitch_value.buff[0]),&buff[7],4);  // pitch 偏移量
						memcpy(&(VisionValue.vision_Left_Gimbal_distance.buff[0]),&buff[11],4);  // 装甲板距离
				
        }
    }
		if( (VisionValue.vision_left_yaw_value.value > 100 ) || (VisionValue.vision_left_pitch_value.value > 100 ) )
    {
        VisionValue.vision_left_yaw_value.value = 0 ;
        VisionValue.vision_left_pitch_value.value = 0 ;
    }        
}	
