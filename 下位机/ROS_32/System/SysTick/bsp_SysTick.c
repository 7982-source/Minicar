
#include "bsp_SysTick.h"


/*
 * @brief  系统滴答定时器初始化函数
 *         配置SysTick定时器以1ms为周期触发中断
 * @param  无
 * @retval 无
 * @note   使用SystemCoreClock作为时钟源
 *         配置失败时会进入死循环
 */
void SysTick_Init(void)
{
	/* SystemFrequency / 1000    1ms中断一次
	 * SystemFrequency / 100000  10us中断一次
	 * SystemFrequency / 1000000 1us中断一次
	 */
//	if (SysTick_Config(SystemFrequency / 100000))	// ST3.0.0版本
	if (SysTick_Config(SystemCoreClock / 1000))	// ST3.5.0版本
	{ 
		/* Capture error */ 
		while (1);
	}
		
//	SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;	// 关闭滴答定时器  
}

