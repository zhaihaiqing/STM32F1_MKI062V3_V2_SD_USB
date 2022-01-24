
#include "main.h"

volatile unsigned char RTC_SEC_FLAG=0;
volatile unsigned char RTC_Alarm_FLAG=0;
volatile unsigned char Alarm_InterVal=5;

unsigned char const table_week[12]={0,3,3,6,1,4,6,2,5,0,3,5};//���������ݱ�ƽ����·����ڱ�
unsigned char const mon_table[12]={31,28,31,30,31,30,31,31,30,31,30,31};//
calendar_obj Calendar;

/*******************************************************************************
* Function Name  : Is_Leap_Year
* Description    : �ж��Ƿ�������
* Input          : year
* Output         : None
* Return         : 1,�����꣬0��������
*******************************************************************************/
unsigned char Is_Leap_Year(unsigned short int year)
{
	if( ( (year%4==0) && (year%100!=0) ) || (year%400==0) )return 1;
	else return 0;
}

/*******************************************************************************
* Function Name  : RTC_Get_Week
* Description    : ����ĳ��ĳ��ĳ�յ�������
* Input          : �꣬�£���
* Output         : None
* Return         : 0:���գ�1:��һ��2:�ܶ�....
*******************************************************************************/
unsigned char RTC_Get_Week(unsigned short int year,unsigned char month,unsigned char day)
{
	int c=0;
	int y=0;
	int week=0;
	if(month==1 || month==2)
	{
		year--;
		month+=12;
	}
	c=year/100;
	y=year-c*100;
	week=(c/4)-2*c+(y+y/4)+(13*(month+1)/5)+day-1;
	while(week<0)week+=7;
	week%=7;
	return week;
}

/*******************************************************************************
* Function Name  : RTC_Set
* Description    : ����ʱ�䣨�����գ�ʱ���룩
* Input          : �꣬�£��գ�ʱ,��,��
* Output         : None
* Return         : 0:ʧ�ܣ�1:�ɹ�
*******************************************************************************/
unsigned char RTC_Set(unsigned short int syear,unsigned char smon,unsigned char sday,unsigned char hour,unsigned char min,unsigned char sec)
{
	unsigned short int t=0;
	unsigned int seccount=0;
	if(syear<1970 || syear>2099)return ERROR;
	for(t=1970;t<syear;t++)//�����е���������
	{
		if(Is_Leap_Year(t))seccount+=31622400;//���������
		else seccount+=31536000;//ƽ�������
	}
	smon-=1;
	for(t=0;t<smon;t++)//��ǰ����·ݵ��������
	{
		seccount+=(unsigned int)mon_table[t]*86400;//�·��������
		if(Is_Leap_Year(syear)&&t==1)seccount+=86400;//���� 2�·�����1�������
	}
	seccount+=(unsigned int)(sday-1)*86400;//��ǰ������ڵ����������
	seccount+=(unsigned int)hour*3600;//Сʱ�������
	seccount+=(unsigned int)min*60;//������
	seccount+=sec;//��������
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP,ENABLE );//ʹ��PWR��BKPʱ��
	PWR_BackupAccessCmd(ENABLE);//ʹ��RTC�ͺ󱸼Ĵ�������
	RTC_SetCounter(seccount);//����RTC��������ֵ
	RTC_WaitForLastTask();//�ȴ����
	return SUCCESS;
}


/*******************************************************************************
* Function Name  : Set_Alarm
* Description    : ��������ʱ�䣬�ڵ�ǰʱ�������+N����
* Input          : N
* Output         : None
* Return         : 
*******************************************************************************/
unsigned char Set_Alarm(unsigned short int Alarm_N)
{
		//unsigned char temp1=0;
		uint32_t flag1=0;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP , ENABLE);//ʹ��PWR��BKP����ʱ��
		PWR_BackupAccessCmd(ENABLE);//ʹ�ܺ󱸼Ĵ�������
		RTC_ITConfig(RTC_IT_SEC|RTC_IT_ALR,ENABLE);//ʹ��RTC�����ж�
		
		
		flag1=RTC_GetCounter();
		RTC_WaitForLastTask();//�ȴ����
		if(Alarm_InterVal==0)RTC_SetAlarm(flag1+10);//ÿ10������һ��;
		//else RTC_SetAlarm(flag1+Alarm_InterVal*60);//ÿN��������һ��
		else RTC_SetAlarm(flag1+Alarm_N*60);//ÿN��������һ��
		RTC_WaitForLastTask();	//�ȴ����
		BKP_WriteBackupRegister(BKP_DR1,0x5050);//��ָ���ĺ󱸼Ĵ�����д������
		return SUCCESS;
}

/*******************************************************************************
* Function Name  : Internal_RTC_Init
* Description    : �ڲ�RTC��ʼ��
* Input          : None
* Output         : None
* Return         : 0��ʧ�ܣ�1���ɹ�
*******************************************************************************/
unsigned char Internal_RTC_Init(void)
{
	unsigned char temp=0;
	uint32_t flag=0;
	
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	/* Enable the RTC Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;              //�����ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   //��RTCȫ���жϵ����ȼ���
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//����ǲ��ǵ�һ������ʱ��
	if(BKP_ReadBackupRegister(BKP_DR1) != 0x5050)//��ָ���ĺ󱸼Ĵ����ж�������
	//ע�⣺���ʹ������ǰ���ù������ٴ�ǿ�ƽ����ٴ�����һ��
	//if(1)//��ָ���ĺ󱸼Ĵ����ж�������
	{
			
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP , ENABLE);//ʹ��PWR��BKP����ʱ��
		PWR_BackupAccessCmd(ENABLE);//ʹ�ܺ󱸼Ĵ�������
		BKP_DeInit();               //��λ��������
		RCC_LSEConfig(RCC_LSE_ON);  //�����ⲿ����ʱ��
		while(RCC_GetFlagStatus(RCC_FLAG_LSERDY)==RESET)//���ָ����RCC��־λ���ã��ȴ�ʱ�Ӿ���
		{
			Delay(10);
			temp++;
			if(temp>=250)return ERROR;//��ʼ��ʧ��
		}
		
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);//����RTCʱ�ӣ�ѡ��LSE
		RCC_RTCCLKCmd(ENABLE);//ʹ��RTCʱ��
		RTC_WaitForLastTask();//�ȴ����
		RTC_WaitForSynchro();//�ȴ�RTC�Ĵ���ͬ��
		RTC_ITConfig(RTC_IT_SEC|RTC_IT_ALR,ENABLE);//ʹ��RTC�����ж�
		
		RTC_WaitForLastTask();//�ȴ�RTC���
		RTC_EnterConfigMode();//��������ģʽ
		RTC_SetPrescaler(32767);//����RTCʱ��Ԥ��Ƶֵ
		RTC_WaitForLastTask();//�ȴ����
		RTC_Set(2017,07,17,12,0,0);//����ʱ��
		RTC_ExitConfigMode();//�˳�����ģʽ
		
		flag=RTC_GetCounter();
		RTC_WaitForLastTask();//�ȴ����
		if(Alarm_InterVal==0)RTC_SetAlarm(flag+10);//ÿ10������һ��;
		else RTC_SetAlarm(flag+Alarm_InterVal*60);//ÿAlarm_InterVal��������һ��
		RTC_WaitForLastTask();	//�ȴ����
		BKP_WriteBackupRegister(BKP_DR1,0x5050);//��ָ���ĺ󱸼Ĵ�����д������
	}
	else
	{
		Set_Alarm(Alarm_InterVal);
	}
	RTC_Get();//����ʱ��
	return SUCCESS;
}

/*******************************************************************************
* Function Name  : RTC_Get
* Description    : ��ȡʱ��
* Input          : None
* Output         : �ꡢ�¡��ա�ʱ���֡��롢����
* Return         : 0��ʧ�ܣ�1���ɹ�
*******************************************************************************/
unsigned char RTC_Get(void)
{
	static unsigned short int daycnt=0;
	unsigned int timecount=0;
	unsigned int temp=0;
	unsigned short int temp1=0;
	timecount =RTC->CNTH;//�õ��������е�ֵ
	timecount<<=16;
	timecount+=RTC->CNTL;
	
	temp=timecount/86400;//�õ�����
	if(daycnt!=temp)//������1��
	{
		daycnt=temp;
		temp1=1970;
		while(temp>365)
		{
			if(Is_Leap_Year(temp1))//����
			{
				if(temp>=365)temp-=366;//�����������
				else{temp1++;break;}
			}
			else temp-=365;//ƽ��
			temp1++;
		}
		Calendar.year=temp1;//�õ��·�
		temp1=0;
		while(temp>=28)//������1����
		{
			if(Is_Leap_Year(Calendar.year) && temp1==1)//�����ǲ�������/2�·�
			{
				if(temp>=29)temp-=29;//�����������
				else break;
			}
			else
			{
				if(temp>=mon_table[temp1])temp-=mon_table[temp1];//ƽ��
				else break;
			}
			temp1++;
		}
		Calendar.month=temp1+1;//�õ��·�
		Calendar.day=temp+1;//�õ�����
	}
	temp=timecount%86400;
	Calendar.hour=temp/3600;
	Calendar.min=(temp%3600)/60;
	Calendar.sec=(temp%3600)%60;
	Calendar.week=RTC_Get_Week(Calendar.year,Calendar.month,Calendar.day);
	
	return SUCCESS;
}
		
		
		
	
	







