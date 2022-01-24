
#ifndef __UART_H
#define __UART_H

#define EMPTY 	0xFFFF
#define TIMEOUT 0xFFFF

//#define  USART2_RXDATA_SIZE 9		//串口2接收数据包长度
#define Default_USART2_INTERVAL_TIME 3  //定义串口两个包接收间隔时间限制值,3ms

#define UART1_RBUF_SIZE   256		//要求：2 的整次幂
#if UART1_RBUF_SIZE < 2
#error UART1_RBUF_SIZE is too small.  It must be larger than 1.
#elif ((UART1_RBUF_SIZE & (UART1_RBUF_SIZE-1)) != 0)
#error UART1_RBUF_SIZE must be a power of 2.
#endif

#define UART2_RBUF_SIZE   256		//要求：2 的整次幂
#if UART2_RBUF_SIZE < 2
#error UART2_RBUF_SIZE is too small.  It must be larger than 1.
#elif ((UART2_RBUF_SIZE & (UART2_RBUF_SIZE-1)) != 0)
#error UART2_RBUF_SIZE must be a power of 2.
#endif

#define UART3_RBUF_SIZE   256		//要求：2 的整次幂
#if UART3_RBUF_SIZE < 2
#error UART3_RBUF_SIZE is too small.  It must be larger than 1.
#elif ((UART3_RBUF_SIZE & (UART3_RBUF_SIZE-1)) != 0)
#error UART3_RBUF_SIZE must be a power of 2.
#endif






typedef struct uart1_rbuf_st
{
	unsigned int in;							//输入
	unsigned int out;							//输出
	unsigned char  buf [UART1_RBUF_SIZE];		//缓冲区空间
}UART1_RBUF_ST;

typedef struct uart2_rbuf_st
{
	unsigned int in;							//输入
	unsigned int out;							//输出
	unsigned char  buf [UART2_RBUF_SIZE];		//缓冲区空间
}UART2_RBUF_ST;

typedef struct uart3_rbuf_st
{
	unsigned int in;							//输入
	unsigned int out;							//输出
	unsigned char  buf [UART3_RBUF_SIZE];		//缓冲区空间
}UART3_RBUF_ST;

extern unsigned char    Uart1Flag;
extern UART1_RBUF_ST	uart1_rbuf;

extern unsigned char    Uart2Flag;
extern UART2_RBUF_ST	uart2_rbuf;

extern unsigned char    Uart3Flag;
extern UART3_RBUF_ST	uart3_rbuf;

extern volatile unsigned char USART2_INTERVAL_TIME;
extern volatile unsigned char USART2_CommandLen;

//void UART1TX_DMA_Configuration( void );
//void UART1_DMA_SendData(unsigned char *BufAddress,unsigned int Length);
void UART1_Configuration(unsigned int baudrate);
void UART2_Configuration(unsigned int baudrate);
void UART3_Configuration(unsigned int baudrate);
void NVIC_UART_Configuration(void);
void USART_PutChar(USART_TypeDef* USARTx,unsigned char ch);
void USART_PutData(USART_TypeDef* USARTx,unsigned char *dat,unsigned short int len);
void USART_PutS(USART_TypeDef* USARTx,unsigned char *s);
void USART2_ClearBuf_Flag(void);
unsigned short int USART1_GetCharBlock(unsigned short int timeout);
unsigned short int USART1_GetChar(void);
unsigned short int USART2_GetCharBlock(unsigned short int timeout);
unsigned short int USART2_GetChar(void);



#endif


