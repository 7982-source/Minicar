#include "stm32f10x.h"

// 左轮电机PWM初始化 (TIM2_CH2 -> PA1)
void Right_PWM_Init(void)
{
    // 1. 开启时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    // 2. 配置GPIO
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // 复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 3. 配置时基单元
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;    // 自动重装载寄存器 ARR
    TIM_TimeBaseInitStructure.TIM_Prescaler = 36 - 1;   // 预分频器 PSC (72MHz / 36 / 100 = 20KHz PWM)
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
    
    // 4. 配置输出比较
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0; // 初始占空比为0
    TIM_OC2Init(TIM2, &TIM_OCInitStructure); // **修正: 使用 TIM_OC2Init**
    
    // 5. 启动定时器
    TIM_Cmd(TIM2, ENABLE);
}

// 右轮电机PWM初始化 (TIM4_CH2 -> PB7)
void Left_PWM_Init(void)
{
    // 1. 开启时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // 2. 配置GPIO
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; // TIM4_CH2
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 3. 配置时基单元
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 36 - 1;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

    // 4. 配置输出比较
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC2Init(TIM4, &TIM_OCInitStructure); // **正确: 使用 TIM_OC2Init**
    
    // 5. 启动定时器
    TIM_Cmd(TIM4, ENABLE);
}

// 设置左轮电机速度（占空比）
void Rightspeed_set(uint16_t Compare)
{
    TIM_SetCompare2(TIM2, Compare);
}

// 设置右轮电机速度（占空比）
void Leftspeed_set(uint16_t Compare)
{
    TIM_SetCompare2(TIM4, Compare);
}
