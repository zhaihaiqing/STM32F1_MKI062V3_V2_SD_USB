/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "main.h"

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	if (TimingDelay != 0x00)
	{ 
		TimingDelay--;
	}
	SysTick_Count++;
	if(SysTick_Count==500)
	{
		SysTick_Count=0;
		ACC_Flag=1;
	}
	
//	if(SysTick_Count==1950);
//		
//		//LED_ON();
//	else if(SysTick_Count==2000)
//	{
//		SysTick_Count=0;
//		//LED_OFF();
//	}
	if(USART2_INTERVAL_TIME)USART2_INTERVAL_TIME--;
	else if(USART2_CommandLen && !Uart2Flag)Uart2Flag = 1;
}

/*******************************************************************************
* Function Name  : TIM4_IRQHandler
* Description    : 定时器4中断服务函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM4_IRQHandler(void)
{
	unsigned char PWM_SPEED=1;
	 if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	 { 
			//CCR2_Val_Add+=1;						//步进1Hz
			//CCR2_Val=	HSE_VALUE/CCR2_Val_Add - 1; //重新计算初值
			//TIM4->ARR = CCR4_Val;					//装填初值
		    //TIM4->CCR4 = CCR4_Val/2;				    //设置占空比
		  //if(CCR2_Val_Add>=1200) {CCR2_Val_Add=400;TIM_Cmd(TIM4, DISABLE);}
		 //TIM4->ARR    = CCR4_Val;    //自动重装载周期值,设置PWM输出频率
		 //TIM4->CCR4   = CCR4_Val/2;  //设置输出信号的占空比，50%
		 if(P_Val_bit==0) //变亮过程
		 {
			 P_Val=P_Val+PWM_SPEED;
			 if(P_Val>=(CCR4_Val/2 - PWM_SPEED))P_Val_bit=1;
		 }
		 
		  if(P_Val_bit==1)//变暗过程
		 {
			 P_Val=P_Val-PWM_SPEED;
			 if(P_Val<=5)P_Val_bit=0; 
		 }
		 TIM4->CCR4   = P_Val;//将计算后的占空比数据写入对应寄存器
		 TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
   }
}

void USART2_IRQHandler(void)
{
	unsigned char 	Uart_Get_Data;	//串口2接收的数据
	UART2_RBUF_ST *p = &uart2_rbuf;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		Uart_Get_Data = USART_ReceiveData(USART2);
		
		//USART2_RX[RxCounter2++] = Uart_Get_Data;
		//if(Uart_Get_Data == '\0')Uart2Flag = 1;
		
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
		if(!Uart2Flag)
		{
			if( '\0' != Uart_Get_Data )
			{
				if((((p->out - p->in) & (UART2_RBUF_SIZE - 1)) == 0) || USART2_INTERVAL_TIME)
				{
					USART2_INTERVAL_TIME=Default_USART2_INTERVAL_TIME;
					if((p->in - p->out)<UART2_RBUF_SIZE)
					{
						p->buf [p->in & (UART2_RBUF_SIZE-1)] = Uart_Get_Data;	
						p->in++;
					}
					USART2_CommandLen  = (p->in - p->out) & (UART2_RBUF_SIZE - 1);//获取数据长度
				}
			}
			
		}
		
		
	}
}

void RTC_IRQHandler(void)
{
	uint32_t flag=0;
	if(RTC_GetITStatus(RTC_IT_SEC) != RESET )//秒钟中断
	{
		RTC_SEC_FLAG=1;
	}
	if(RTC_GetITStatus(RTC_IT_ALR) != RESET )//清闹钟中断
	{
		RTC_ClearITPendingBit(RTC_IT_ALR);//清闹钟中断
		RTC_Alarm_FLAG=1;
		flag=RTC_GetCounter();
		RTC_WaitForLastTask();
		RTC_SetAlarm(flag+Alarm_InterVal*60);
		RTC_WaitForLastTask();	
	}
	RTC_ClearITPendingBit(RTC_IT_SEC | RTC_IT_OW);//清秒钟中断
	RTC_WaitForLastTask();
}

//void USART2_IRQHandler(void)
//{
//	unsigned char 	Uart_Get_Data;	//串口2接收的数据
//	UART2_RBUF_ST *p = &uart2_rbuf;
//	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
//	{
//		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
//		
//		Uart_Get_Data = USART_ReceiveData(USART2);
//		if((p->in - p->out)<UART2_RBUF_SIZE)
//		{
//			p->buf [p->in & (UART2_RBUF_SIZE-1)] = Uart_Get_Data;	
//			p->in++;
//			//Uart2Flag = 1;
//		}
//		
//	}
//}


//void TIM6_IRQHandler(void)
//{
//	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
//	{
//		TIM_ClearITPendingBit(TIM6, TIM_IT_Update); //清除标志位
//		if(TimingDelay)TimingDelay--;
//	}
//}



/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
