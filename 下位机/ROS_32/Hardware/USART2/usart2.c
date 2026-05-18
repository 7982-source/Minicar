//#include "sys.h"
#include "stm32f10x.h"      
#include "usart2.h"
#include "ringbuffer.h"
#include "OLED.h"

#define UART2_RX_BUFFER_SIZE	256
uint8_t uart2RxBuffer[UART2_RX_BUFFER_SIZE];

ringbuffer_t uart2RxFifo;

/**
 * @brief       串口X初始化
 * @param       无
 * @retval      无
 */
void usart2_init(uint32_t bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//使能USART2，GPIOA时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 , ENABLE);
	//USART2_TX   GPIOA.2
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA2
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.2
   
  //USART2_RX	  GPIOA.3初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.3  

  //Usart2 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART2, &USART_InitStructure); //初始化串口2
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART2, ENABLE);                    //使能串口2

   ringbuffer_init(&uart2RxFifo, uart2RxBuffer, UART2_RX_BUFFER_SIZE);
}


/**
  * @brief  串口2发送
  * @param  data: 发送的数据
  * @param  len: 数据长度
  * @retval uint8_t: 0成功 其他：失败
  */
void usart2_sendData(uint8_t *data){

        USART_SendData(USART2,*data);
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

/**
  * @brief  获取串口2接收fifo的数据
  * @param  buf: 存放的缓冲区
  * @param  len: 需要获取的长度
  * @retval uint16_t: 实际获取到的长度 0表示没有数据可获取
  */
uint16_t usart2_getRxData(uint8_t *buf, uint16_t len)
{
    return ringbuffer_out(&uart2RxFifo, buf, len);
}


/**
 * @brief       串口X中断服务函数
 * @param       无
 * @retval      无
 */

void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2,USART_IT_RXNE))/*!< 接收非空中断 */
	{     
		    //int a=a+1;
				uint8_t res = USART_ReceiveData(USART2);
        ringbuffer_in_check(&uart2RxFifo, (uint8_t *)&res, 1); /*!< 将接收到的数据放入FIFO */	
		    //OLED_ShowNum(4,1,a,5);
	}
}


/*******************************END OF FILE************************************/



