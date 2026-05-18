#include "stm32f10x.h"
#include "PWM.h"


void Motor_Init(void)
{
    // 开启GPIOA和GPIOB时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    // 配置GPIOA的PA4和PA5引脚 L
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 配置GPIOB的PB7和PB8引脚 R 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_5;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
		
	Left_PWM_Init();
	Right_PWM_Init();
}

void L_SetSpeed(int8_t Speed)
{
	if (Speed >= 0)
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_4);
		GPIO_ResetBits(GPIOA, GPIO_Pin_5);
		Leftspeed_set(Speed);
	}
	else
	{
		GPIO_ResetBits(GPIOA, GPIO_Pin_4);
		GPIO_SetBits(GPIOA, GPIO_Pin_5);
		Leftspeed_set(-Speed);
	}
}

void R_SetSpeed(int8_t Speed)
{
	if (Speed >= 0)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_5);
		GPIO_ResetBits(GPIOB, GPIO_Pin_6);
		Rightspeed_set(Speed); 
	}
	else
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_5);
		GPIO_SetBits(GPIOB, GPIO_Pin_6);
		Rightspeed_set(-Speed); 
	}
}
