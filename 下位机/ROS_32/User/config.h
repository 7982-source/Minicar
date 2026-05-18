 #ifndef _CONFIG_H_
 #define _CONFIG_H_
 
#include "stm32f10x.h"
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
#include <stdio.h>
#include "ringbuffer.h"
#include "pid.h"


extern int16_t Speed_L;			//定义速度变量
extern int16_t Speed_R;


extern IncrementalPID Left_PID;
extern IncrementalPID Right_PID;
extern double Left_Target_Speed;
extern double Right_Target_Speed;

extern uint8_t ch;
#endif
