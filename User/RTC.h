#ifndef __RTC_H
#define __RTC_H

typedef struct
{
	unsigned short int year;
	unsigned char month;
	unsigned char day;
	unsigned char week;
	unsigned char hour;
	unsigned char min;
	unsigned char sec;
}calendar_obj;

unsigned char Is_Leap_Year(unsigned short int year);
unsigned char RTC_Get_Week(unsigned short int year,unsigned char month,unsigned char date);
unsigned char Internal_RTC_Init(void);
unsigned char RTC_Set(unsigned short int syear,unsigned char smon,unsigned char sday,unsigned char hour,unsigned char min,unsigned char sec);
unsigned char RTC_Get(void);
unsigned char Set_Alarm(unsigned short int Alarm_N);

extern volatile unsigned char RTC_SEC_FLAG;
extern volatile unsigned char RTC_Alarm_FLAG;
extern volatile unsigned char Alarm_InterVal;
extern calendar_obj Calendar;

#endif
