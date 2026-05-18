#include "config.h"

u8 All_Init(void)
{   
	  uint8_t init_error = 0;
	  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	  //usart2_init(115200);   //IMU串口初始化 
	  //imu901_init();
    Delay_ms(1000);	
	  LED_Init();
		OLED_Init();		//OLED初始化
		enconder_Init();		//编码器初始化
		Loop_Init();
		SysTick_Init();
		Motor_Init();
	  Serial_Init(115200);
  	
    return (uint8_t)init_error;
}
