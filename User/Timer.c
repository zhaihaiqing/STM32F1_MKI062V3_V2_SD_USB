/* Includes ------------------------------------------------------------------*/
#include "main.h"


/*******************************************************************************
* Function Name  : TIM2_Configuration
* Description    : ��ʱ��2��ʼ������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void TIM2_Configuration(void)
//{
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	TIM_DeInit(TIM2);					//��λTIM2
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
//	
//	//�ж����ȼ�NVIC����
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
//	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2�ж�
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //��ռ���ȼ�0��
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;  //�����ȼ�3��
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
//	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���
//	TIM_ClearFlag(TIM2,TIM_FLAG_Update);
//  TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM2�ж�,��������ж�	
//	
//	TIM_InternalClockConfig(TIM2);
//	TIM_TimeBaseStructure.TIM_Period = 400;//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
//  TIM_TimeBaseStructure.TIM_Prescaler = 36863;//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ,32M/(32000-1)=1KhZ
//  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//����ʱ�ӷָ�:TDTS = Tck_tim
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM���ϼ���ģʽ
//	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);//����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
//	TIM_Cmd(TIM2, ENABLE);  //ʹ��TIMx	
//}

//void TIM3_Configuration(void)
//{
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	TIM_DeInit(TIM3);					//��λTIM3
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
//	
//	//�ж����ȼ�NVIC����
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
//	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //��ռ���ȼ�2��
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
//	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���
//	TIM_ClearFlag(TIM3,TIM_FLAG_Update);
//  TIM_ITConfig(TIM3,TIM_IT_Update,DISABLE ); //ʹ��ָ����TIM3�ж�,��������ж�	
//	
//	//   29.4912M/(2950-1)=10K
//	TIM_InternalClockConfig(TIM3);
//	TIM_TimeBaseStructure.TIM_Period = 100;//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
//  TIM_TimeBaseStructure.TIM_Prescaler = 2950;//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ,��ʱ�Ӳ���Ƶ����Ƶ��=800hZ
//  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//����ʱ�ӷָ�:TDTS = Tck_tim
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM���ϼ���ģʽ
//	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);//����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
//	TIM_Cmd(TIM3, DISABLE);  //ʹ��TIMx	
//}

/**********************************************************************************
* Function Name  : TIM6_Configuration
* TIM6��ʼ��
**********************************************************************************/
void TIM6_Configuration(void)
{
	/* TIM6 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
}
/**********************************************************************************
* Function Name  : TIM6_Reconfiguration
* TIM6��������
* nTime:��ʱֵ
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
* TIM6�ر�
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
* �����ж� ��ʼ��
*******************************************************************************/
void NVIC_TIM6_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the TIM6 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn; //�����ж�
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//��ռʽ
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

uint16_t CCR4_Val=83;
uint16_t CCR4_Val_Add=1500;//Ƶ��ֵ
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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//���踴��ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); //��ӳ�� TIM4_CH4->PB9
	AFIO->MAPR &=0xffffefff;//ע��˴�
	
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
	//������Ҫװ���ֵ
	CCR4_Val=	HSE_VALUE/CCR4_Val_Add - 1;
	PrescalerValue = (uint16_t) (SystemCoreClock/HSE_VALUE) - 1;
	P_Val=1;
	/* Time base configuration */
	TIM4->PSC = PrescalerValue;//����ʱ�ӷ�Ƶϵ�����˴��ⲿ����8M����PLL 4��Ƶ���ڴ˴�4��Ƶ��T4ʱ����Ϊ8M;
	TIM4->DIER|=1<<0;          // ��������ж�

	/* PWM1 Mode configuration: Channel2 */
	TIM4->CCMR2 |= 6<<12;       //CH4 PWM1ģʽ
	TIM4->CCMR2 |= 1<<11;		//CH4Ԥװ��ʹ��
	
	TIM4->ARR    = CCR4_Val;    //�Զ���װ������ֵ,����PWM���Ƶ��
	
	TIM4->CCER  |=1<<12;		//CH4  OC4���ʹ��
	
	TIM4->CCR4   = P_Val;  			//��������źŵ�ռ�ձȣ�50%
	
	TIM4->CR1    = 0x0080;      //ARPEʹ��
	TIM4->CR1   |= 0x01;        //ʹ�ܶ�ʱ��4

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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//���踴��ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); //��ӳ�� TIM4_CH4->PB9
	AFIO->MAPR &=0xffffefff;//ע��˴�
	
	TIM_DeInit(TIM4);
	
		/* Compute the prescaler value */
	//������Ҫװ���ֵ
	CCR4_Val=	HSE_VALUE/CCR4_Val_Add - 1;
	PrescalerValue = (uint16_t) (SystemCoreClock/HSE_VALUE) - 1;
	
	TIM_TimeBaseStructure.TIM_Period = CCR4_Val;  //�������Զ���װ������ֵ������PWMƵ��
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;  //����Ԥ��Ƶֵ  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim  
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM ���ϼ���ģʽ
	
	//TIM_TimeBaseStructure.TIM_RepetitionCounter=0x0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //�۳�ʼ�� TIMx  
	//
	
	//��ʼ�� TIM4 Channel4 PWM ģʽ  
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ�� PWM ģʽ 2  
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ�� 
	TIM_OCInitStructure.TIM_Pulse=CCR4_Val/10;	
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //������Ը� 
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);  //�ܳ�ʼ������ TIM4 OC2  
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable); //ʹ��Ԥװ�ؼĴ���
	TIM_ARRPreloadConfig(TIM4,ENABLE);
	  
	TIM_Cmd(TIM4, ENABLE);  //��ʹ�� TIM4   

}



