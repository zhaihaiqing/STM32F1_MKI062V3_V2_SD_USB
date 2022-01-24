/* Includes ------------------------------------------------------------------*/
#include "main.h"


/*******************************************************************************
* Function Name  : TIM2_Configuration
* Description    : 定时器2初始化设置
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void TIM2_Configuration(void)
//{
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	TIM_DeInit(TIM2);					//复位TIM2
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
//	
//	//中断优先级NVIC设置
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
//	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2中断
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级0级
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;  //从优先级3级
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
//	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器
//	TIM_ClearFlag(TIM2,TIM_FLAG_Update);
//  TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); //使能指定的TIM2中断,允许更新中断	
//	
//	TIM_InternalClockConfig(TIM2);
//	TIM_TimeBaseStructure.TIM_Period = 400;//设置在下一个更新事件装入活动的自动重装载寄存器周期的值
//  TIM_TimeBaseStructure.TIM_Prescaler = 36863;//设置用来作为TIMx时钟频率除数的预分频值,32M/(32000-1)=1KhZ
//  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//设置时钟分割:TDTS = Tck_tim
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM向上计数模式
//	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);//根据指定的参数初始化TIMx的时间基数单位
//	TIM_Cmd(TIM2, ENABLE);  //使能TIMx	
//}

//void TIM3_Configuration(void)
//{
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	TIM_DeInit(TIM3);					//复位TIM3
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
//	
//	//中断优先级NVIC设置
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
//	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级2级
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
//	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器
//	TIM_ClearFlag(TIM3,TIM_FLAG_Update);
//  TIM_ITConfig(TIM3,TIM_IT_Update,DISABLE ); //使能指定的TIM3中断,允许更新中断	
//	
//	//   29.4912M/(2950-1)=10K
//	TIM_InternalClockConfig(TIM3);
//	TIM_TimeBaseStructure.TIM_Period = 100;//设置在下一个更新事件装入活动的自动重装载寄存器周期的值
//  TIM_TimeBaseStructure.TIM_Prescaler = 2950;//设置用来作为TIMx时钟频率除数的预分频值,主时钟不倍频不分频，=800hZ
//  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//设置时钟分割:TDTS = Tck_tim
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM向上计数模式
//	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);//根据指定的参数初始化TIMx的时间基数单位
//	TIM_Cmd(TIM3, DISABLE);  //使能TIMx	
//}

/**********************************************************************************
* Function Name  : TIM6_Configuration
* TIM6初始化
**********************************************************************************/
void TIM6_Configuration(void)
{
	/* TIM6 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
}
/**********************************************************************************
* Function Name  : TIM6_Reconfiguration
* TIM6重新配置
* nTime:延时值
**********************************************************************************/
void TIM6_Reconfiguration(__IO uint32_t nTime)
{ 
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Period = nTime;
  TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock/2/1000;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
	TIM6->CNT = 0;
		/* TIM IT enable */
  TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

  /* TIM6 enable counter */
  TIM_Cmd(TIM6, ENABLE);
}
/**********************************************************************************
* Function Name  : TIM6_Disable
* TIM6关闭
**********************************************************************************/
void TIM6_Disable(void)
{
  /* TIM IT disable */
	TIM_ITConfig(TIM6, TIM_IT_Update, DISABLE);
  /* TIM6 disable counter */
  TIM_Cmd(TIM6, DISABLE);
}

/*******************************************************************************
* Function Name  : NVIC_TIM6_Configuration
* 捕获中断 初始化
*******************************************************************************/
void NVIC_TIM6_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the TIM6 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn; //捕获中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//抢占式
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

uint16_t CCR4_Val=83;
uint16_t CCR4_Val_Add=1500;//频率值
uint16_t PrescalerValue = 0;

unsigned short int capture=0;
unsigned short int ARR;
volatile unsigned short int P_Val;
volatile unsigned char P_Val_bit=0;

void PWM_Output(void)
{
  /* TIM4 clock enable */
	//TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	//TIM_OCInitTypeDef  TIM_OCInitStructure;
	//TIM_BDTRInitTypeDef TIM4_BDTRInitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//外设复用模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); //重映射 TIM4_CH4->PB9
	AFIO->MAPR &=0xffffefff;//注意此处
	
	TIM_DeInit(TIM4);
  /* -----------------------------------------------------------------------
    TIM4 Configuration: generate 4 PWM signals with 4 different duty cycles:
    The TIM4CLK frequency is set to SystemCoreClock (Hz), to get TIM4 counter
    clock at 7.3728 MHz the Prescaler is computed as following:
     - Prescaler = (TIM4CLK / TIM4 counter clock) - 1
    SystemCoreClock is set to 32 MHz for  Medium-density devices.

    The TIM4 is running at 10 KHz: TIM4 Frequency = TIM4 counter clock/(ARR + 1)
                                                  = 7372800 MHz / 737 = 10 KHz
    TIM4 Channel1 duty cycle = (TIM4_CCR1/ TIM4_ARR)* 100 = 50%          ch1
    TIM4 Channel2 duty cycle = (TIM4_CCR2/ TIM4_ARR)* 100 = 37.5%        ch2
    TIM4 Channel3 duty cycle = (TIM4_CCR3/ TIM4_ARR)* 100 = 25%          ch3
    TIM4 Channel4 duty cycle = (TIM4_CCR4/ TIM4_ARR)* 100 = 12.5%        ch4
  ----------------------------------------------------------------------- */

	/* Compute the prescaler value */
	//计算需要装填的值
	CCR4_Val=	HSE_VALUE/CCR4_Val_Add - 1;
	PrescalerValue = (uint16_t) (SystemCoreClock/HSE_VALUE) - 1;
	P_Val=1;
	/* Time base configuration */
	TIM4->PSC = PrescalerValue;//设置时钟分频系数，此处外部晶振8M经过PLL 4倍频，在此处4分频后T4时钟仍为8M;
	TIM4->DIER|=1<<0;          // 允许产生中断

	/* PWM1 Mode configuration: Channel2 */
	TIM4->CCMR2 |= 6<<12;       //CH4 PWM1模式
	TIM4->CCMR2 |= 1<<11;		//CH4预装载使能
	
	TIM4->ARR    = CCR4_Val;    //自动重装载周期值,设置PWM输出频率
	
	TIM4->CCER  |=1<<12;		//CH4  OC4输出使能
	
	TIM4->CCR4   = P_Val;  			//设置输出信号的占空比，50%
	
	TIM4->CR1    = 0x0080;      //ARPE使能
	TIM4->CR1   |= 0x01;        //使能定时器4

	TIM_Cmd(TIM4, ENABLE);
}


void TIM4_CH4_PWM(void)
{
	//uint32_t TIM4_FR=10000;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	//TIM_BDTRInitTypeDef TIM4_BDTRInitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//外设复用模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); //重映射 TIM4_CH4->PB9
	AFIO->MAPR &=0xffffefff;//注意此处
	
	TIM_DeInit(TIM4);
	
		/* Compute the prescaler value */
	//计算需要装填的值
	CCR4_Val=	HSE_VALUE/CCR4_Val_Add - 1;
	PrescalerValue = (uint16_t) (SystemCoreClock/HSE_VALUE) - 1;
	
	TIM_TimeBaseStructure.TIM_Period = CCR4_Val;  //设置在自动重装载周期值，设置PWM频率
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;  //设置预分频值  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim  
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM 向上计数模式
	
	//TIM_TimeBaseStructure.TIM_RepetitionCounter=0x0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //③初始化 TIMx  
	//
	
	//初始化 TIM4 Channel4 PWM 模式  
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择 PWM 模式 2  
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能 
	TIM_OCInitStructure.TIM_Pulse=CCR4_Val/10;	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性高 
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);  //④初始化外设 TIM4 OC2  
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable); //使能预装载寄存器
	TIM_ARRPreloadConfig(TIM4,ENABLE);
	  
	TIM_Cmd(TIM4, ENABLE);  //⑤使能 TIM4   

}



