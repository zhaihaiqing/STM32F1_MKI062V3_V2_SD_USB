/* Includes ------------------------------------------------------------------*/
#include "main.h"

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

#define UART_FLAG_TimeOut  0x50000   //超时常量


volatile unsigned char USART2_INTERVAL_TIME = 0;//指令发送间隔时间,超时认为一条指令已接收完毕
volatile unsigned char USART2_CommandLen=0;     //串口接收指令长度


unsigned char Uart1Flag;
UART1_RBUF_ST	uart1_rbuf	=	{ 0, 0, };

unsigned char Uart2Flag;
UART2_RBUF_ST	uart2_rbuf	=	{ 0, 0, };

unsigned char Uart3Flag;
UART3_RBUF_ST	uart3_rbuf	=	{ 0, 0, };


/**********************************************************************************
* Function Name  : UART2_Configuration
* 串口2初始化
* 入口参数
* baudrate:波特率
**********************************************************************************/
void UART2_Configuration(unsigned int baudrate)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* Enable the USART2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//打开GPIO和USART2的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	
	/* 将USART2_Tx的GPIO配置为推挽复用模式 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;     //GPIO_Pin_2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/*  将USART1_Rx的GPIO配置为浮空输入模式
  	由于CPU复位后，GPIO缺省都是浮空输入模式，因此下面这个步骤不是必须的
  	但是，我还是建议加上便于阅读，并且防止其它地方修改了这个口线的设置参数 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;		//GPIO_Pin_3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = baudrate;//设置波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//设置数据位为8
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//设置停止位为1
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件流控位
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//发送和接收
	USART_Init(USART2, &USART_InitStructure);
	
	
	//中断使能
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	//USART_ClearFlag(USART2, USART_FLAG_TC);     /* 清发送标志，Transmission Complete flag */
	
	//使能
	USART_Cmd(USART2, ENABLE);
	
}


//void UART1TX_DMA_Configuration( void )
//{
//	DMA_InitTypeDef DMA_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//  uint32_t temp;
//	/* DMA1 channel4 configuration ---------------------------------------------*/
//  /* Enable DMA1 clock --------------------------------------------------------*/
//	/*DMA1 channel4,USART1_TX*/
//  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
//  DMA_DeInit(DMA1_Channel4);//开启DMA1的第四通道，对应USART1_TX
//	
//  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);//DMA对应的外设地址
//  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&temp;//内存缓存地址
//  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;//DMA的转换模式为DST模式，由内存搬移到外设
//  DMA_InitStructure.DMA_BufferSize = 1;//DMA缓存大小，当发送N个字节的数据后自动产生中断
//  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//接收一次数据后，设备地址禁止后移
//  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址自增
//  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//DMA搬移数据尺寸，Byte就是为8位
//  DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//正常模式
//  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
//  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//内存到内存失能
//  DMA_Init(DMA1_Channel4, &DMA_InitStructure);

//	/* Enable DMA1 channel4 IRQ Channel */
//  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
//	/* Enable USART1_TX request */
//	
//  /* Enable DMA1 Channel4 Transfer Complete interrupt */
//  DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
//	//DMA_ITConfig(DMA1_Channel4, DMA_IT_TE, ENABLE);
//	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //接收缓存区DMA使能
// 	//DMA_Cmd(DMA1_Channel4,  DISABLE);
//}


//void UART1_DMA_SendData(unsigned char *BufAddress,unsigned int Length)
//{
//  DMA_InitTypeDef DMA_InitStructure;
//  
//  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);//外设地址
//  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)BufAddress;//缓存地址
//  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;//内存到外设
//  DMA_InitStructure.DMA_BufferSize = Length;//缓存buf大小
//  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址不自增
//  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址自增
//  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//8bit
//  DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//正常模式
//  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
//  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//内存到内存失能
//  DMA_Init(DMA1_Channel4, &DMA_InitStructure);
//  
//  DMA_Cmd(DMA1_Channel4,  ENABLE);
//  //USART_Cmd(USART1, ENABLE);
//}



/**********************************************************************************
* Function Name  : USART_PutChar
* 串口发送一个字符
* 入口参数
* USARTx:串口号
* ch:数据
**********************************************************************************/
void USART_PutChar(USART_TypeDef* USARTx,unsigned char ch)
{
  unsigned short int timeout = 0;
  USART_SendData(USARTx, (uint8_t) ch);
  while((USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET) && (timeout++ < UART_FLAG_TimeOut));
}
/**********************************************************************************
* Function Name  : USART_PutData
* 串口发送若干个字符
* 入口参数
* USARTx:串口号
* ch:数据
* len:数据长度
**********************************************************************************/
void USART_PutData(USART_TypeDef* USARTx,unsigned char *dat,unsigned short int len)
{
	unsigned short int i;
	for(i = 0;i < len;i++)USART_PutChar(USARTx,(uint8_t)* (dat++));
}
/**********************************************************************************
* Function Name  : USART_PutS
* 串口发送字符串
* 入口参数
* USARTx:串口号
* *s:数据指针
**********************************************************************************/
void USART_PutS(USART_TypeDef* USARTx,unsigned char *s)
{
	while(*s != '\0')USART_PutChar(USARTx,*(s++));
}

/**********************************************************************************
* 串口1接收字符函数，阻塞模式（接收缓冲区中提取）
**********************************************************************************/
uint16_t USART1_GetCharBlock(uint16_t timeout)
{
	UART1_RBUF_ST *p = &uart1_rbuf;
	uint16_t to = timeout;	
	while(p->out == p->in)if(!(--to))return TIMEOUT;
	return (p->buf [(p->out++) & (UART1_RBUF_SIZE - 1)]);
}

/**********************************************************************************
* 串口1接收字符函数，非阻塞模式（接收缓冲区中提取）
**********************************************************************************/
uint16_t USART1_GetChar(void)
{
	UART1_RBUF_ST *p = &uart1_rbuf;			
	if(p->out == p->in) //缓冲区空条件
		return EMPTY;
	return USART1_GetCharBlock(1000);
}

/**********************************************************************************
* 串口2接收字符函数，阻塞模式（接收缓冲区中提取）
**********************************************************************************/
uint16_t USART2_GetCharBlock(uint16_t timeout)
{
	UART2_RBUF_ST *p = &uart2_rbuf;
	uint16_t to = timeout;	
	while(p->out == p->in)if(!(--to))return TIMEOUT;
	return (p->buf [(p->out++) & (UART2_RBUF_SIZE - 1)]);
}

/**********************************************************************************
* 串口2接收字符函数，非阻塞模式（接收缓冲区中提取）
**********************************************************************************/
uint16_t USART2_GetChar(void)
{
	UART2_RBUF_ST *p = &uart2_rbuf;			
	if(p->out == p->in) //缓冲区空条件
		return EMPTY;
	return USART2_GetCharBlock(1000);
}

/**********************************************************************************
* 串口3接收字符函数，阻塞模式（接收缓冲区中提取）
**********************************************************************************/
uint16_t USART3_GetCharBlock(uint16_t timeout)
{
	UART3_RBUF_ST *p = &uart3_rbuf;
	uint16_t to = timeout;
	while(((p->out - p->in)& (UART3_RBUF_SIZE - 1)) == 0)if(!(--to))return TIMEOUT;
	return (p->buf [(p->out++) & (UART3_RBUF_SIZE - 1)]);
}

/**********************************************************************************
* 串口3接收字符函数，非阻塞模式（接收缓冲区中提取）
**********************************************************************************/
uint16_t USART3_GetChar(void)
{
	UART3_RBUF_ST *p = &uart3_rbuf;		
	if(((p->out - p->in) & (UART3_RBUF_SIZE - 1)) == 0) //缓冲区空条件
		return EMPTY;
	return USART3_GetCharBlock(1000);
}

/**********************************************************************************
printf功能定义，(括号中表示从哪个串口输出数据及数据类型)
**********************************************************************************/
PUTCHAR_PROTOTYPE
{
	USART_PutChar(USART2,(uint8_t) ch);
	return ch;
}

/**********************************************************************************
清除串口2接收标志位，按帧进行
**********************************************************************************/
void USART2_ClearBuf_Flag(void)
{
	UART2_RBUF_ST *p = &uart2_rbuf;
	p->out = 0;
	p->in = 0;
	
	Uart2Flag = 0;
	USART2_CommandLen = 0;
	
}





