#ifndef __TIMER_H
#define __TIMER_H

void TIM6_Configuration(void);
void NVIC_TIM6_Configuration(void);
void TIM6_Reconfiguration(volatile unsigned int nTime);
void TIM6_Disable(void);
void PWM_GPIO(void);
void PWM_Output(void);
void TIM4_CH4_PWM(void);

extern unsigned short     int CCR4_Val;
extern unsigned short     int CCR4_Val_Add;//ÆµÂÊÖµ
extern unsigned short     int PrescalerValue;
extern volatile unsigned short int P_Val;
extern volatile unsigned char P_Val_bit;


#endif

