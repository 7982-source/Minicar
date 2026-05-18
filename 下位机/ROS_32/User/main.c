#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
#include "Encoder.h"
#include "usart2.h"
#include "Motor.h"
#include "bsp_SysTick.h"
#include "scheduler.h"
#include "init.h"
#include "LED.h"
#include "imu901.h"
#include "config.h"
#include "Serial.h"


// 全局变量定义，供scheduler.c使用
int16_t Speed_L = 0;			//定义速度变量
int16_t Speed_R = 0;
IncrementalPID Left_PID;
IncrementalPID Right_PID;
double Left_Target_Speed = 0.0;
double Right_Target_Speed =0.0;

double attitude_pitch = 0.0;

int main(void)
{
	// 初始化PID控制器
	init_pid(&Left_PID, 3.2, 0.0, 0.0);
	init_pid(&Right_PID, 3.0, 0.0, 0.0);

	
  uint8_t ch;
	uint8_t err = All_Init();
	
	if(err == 0)
	{
	  OLED_ShowString(3, 5, "INIT OK");
		OLED_Clear();
	
	}
	/*显示静态字符串*/
	OLED_ShowString(1, 1, "LS:");		//1行1列显示字符串Speed:
	OLED_ShowString(2, 1, "RS:");		//1行1列显示字符串Speed:
	
	while (1)
	{
		
		Duty_Loop();
//		if (imu901_uart_receive(&ch, 1)) 	/*!< 获取串口fifo一个字节 */
//        {
//            if (imu901_unpack(ch)) 			/*!< 解析出有效数据包 */
//            {
//                if (rxPacket.startByte2 == UP_BYTE2) 			/*!< 主动上传的数据包 */
//                {
//                    atkpParsing(&rxPacket);
//									 
//                }
//								
//            }
//					
//        }
	}
}
 
