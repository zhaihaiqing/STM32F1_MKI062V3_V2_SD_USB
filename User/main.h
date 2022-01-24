#ifndef __MAIN_H
#define __MAIN_H

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <string.h>
#include "stm32f10x.h"
#include "malloc.h"
#include "uart.h"
#include "GPIO.h"
#include "Timer.h"
#include "I2C.h"
#include "RTC.h"
#include "IMU.h"
#include "SDCard.h"
#include "diskio.h"
#include "ff.h"



#define debug

#ifdef debug
		#define log_info(...)  printf(__VA_ARGS__)
#else
		#define log_info(...)
#endif



#define SystemResetSoft()			*((uint32_t *)0xe000ed0c)=0x05fa0004; //ʵ��ϵͳ�����λ

#define  USART_RX_ST_LEN  512 

#define LED_ON()    GPIO_SetBits(GPIOB,GPIO_Pin_9)
#define LED_OFF()   GPIO_ResetBits(GPIOB,GPIO_Pin_9)



//λ������,ʵ��51���Ƶ�GPIO���ƹ���
//����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).
//IO�ڲ����궨��
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO�ڵ�ַӳ��
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO�ڲ���,ֻ�Ե�һ��IO��!
//ȷ��n��ֵС��16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //��� 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //���� 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //��� 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //���� 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //��� 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //���� 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //��� 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //����

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //��� 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //����

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //��� 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //����


//����������������������ݣ����泤��Ϊ512�ֽڣ�
typedef struct __attribute__ ((__packed__))
{
	unsigned short int year;
	unsigned char mon;
	unsigned char day;
	unsigned char week;
	unsigned char hour;
	unsigned char min;
	unsigned char sec;
	
	float Ax[42];
	float Ay[42];
	float Az[42];
	
	unsigned char ENTER[2];
		
}ACC_buf_t;

extern volatile unsigned int SysTick_Count;
extern volatile unsigned int TimingDelay;
extern volatile unsigned char USART2_RX[USART_RX_ST_LEN];
extern volatile unsigned char RxCounter2;
extern float Angle_x,Angle_y,Angle_z;

extern volatile unsigned char ACC_Flag;

void Delay(volatile unsigned int nTime);


#endif
