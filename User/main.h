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



#define SystemResetSoft()			*((uint32_t *)0xe000ed0c)=0x05fa0004; //实现系统软件复位

#define  USART_RX_ST_LEN  512 

#define LED_ON()    GPIO_SetBits(GPIOB,GPIO_Pin_9)
#define LED_OFF()   GPIO_ResetBits(GPIOB,GPIO_Pin_9)



//位带操作,实现51类似的GPIO控制功能
//具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
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
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入


//建立缓存区，保存倾角数据，缓存长度为512字节，
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
