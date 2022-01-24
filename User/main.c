/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
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

/**
  ******************************************************************************
  * @Description
  *
  * *�������ǽ�������MKI062V3��֤�壨�����壩�ϵģ�ּ����֤���ٶȴ��������ų���
  *��IMU��Ԫ������ѹ�Ƶȴ������Ĳ��ԣ������������ʵ�֡�
  *
	*2017-7-7��ɼ��ٶȴ���������
	*2017-8-15���Ƭ��RTC����
	*2017-10-15���SD����д���������ļ�ϵͳ��
  **V2�汾ּ�ڲ��Դ��ڴ����ַ������ݵ�����������֤����ATָ����ܡ�
	**����SD����д����
*/

/* Includes ------------------------------------------------------------------*/


#include "main.h"

#include "mass_mal.h"
#include "usb_lib.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "memory.h"	    
#include "usb_bot.h" 


volatile unsigned int SysTick_Count = 0;
volatile unsigned int TimingDelay = 0;
volatile unsigned char USART2_RX[USART_RX_ST_LEN];
volatile unsigned char RxCounter2=0;

volatile unsigned char ACC_Flag=0;
float Angle_x,Angle_y,Angle_z;

//ACC_buf_t ACC_buf;

unsigned int ltsp=0;
FATFS fs;     /* Ponter to the filesystem object */
FIL fil;
FRESULT res;
DIR DirInf;
UINT bw;
char FA_BUF[256]={0};


/*******************************************************************************
* Function Name  : Delay function
* Description    : ��ʱ����������ʱ����sleepģʽ������systick��ʱ��ʵ��
* Input          : ��ʱʱ����ms��
* Output         : None
* Return         : None
*******************************************************************************/
void Delay(volatile unsigned int nTime)
{ 
	TimingDelay = nTime;
	while(TimingDelay != 0);//__WFI;//�ȴ������н���sleepģʽ,�·�ָ����ִ�й��������׽���
}
/*******************************************************************************
* Function Name  : SysTick init function
* Description    : ���ö�ʱ���ȼ�SysTick���ȼ�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Init_SysTick(void)
{
	SysTick_Config(SystemCoreClock / 1000); //���ö�ʱ���ȣ�1ms
	NVIC_SetPriority(SysTick_IRQn, 0x0);    //SysTick�ж����ȼ�����
}

/*******************************************************************************
* Function Name  : Init_Hardware
* Description    : ���Ӳ���豸�ĳ�ʼ��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void Init_Hardware()
{
	UART2_Configuration(115200);log_info("Init_UART2 OK!\r\n");
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);//�ж����ȼ�����
	Init_SysTick();	log_info("Init_SysTick OK!\r\n");
	if(Internal_RTC_Init())log_info("RTC Init OK!\r\n");
	GPIO_Configuration();log_info("GPIO_Configuration OK!\r\n");
	I2C1_Configuration();log_info("I2C1_Configuration OK!\r\n");
	Init_LSM303DLH_ACC();log_info("Init_LSM303DLH_ACC OK!\r\n");
	PWM_Output();log_info("LED PWM_Output OK!\r\n");
	log_info("SD_Init:%d\r\n",SD_Init());
	
}

/*******************************************************************************
* Function Name  : Get_SYS_TIME_Task
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Get_SYS_TIME_Task()
{
		log_info("%d��%d��%d��,%d:%d:%d\r\n",Calendar.year,Calendar.month,Calendar.day,Calendar.hour,Calendar.min,Calendar.sec);
}

/*******************************************************************************
* Function Name  : Update_RTC_Task
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Update_RTC_Task()
{
	char ACC_buf[255]={0};
	unsigned char len=0;
	
	if(RTC_SEC_FLAG)
	{
		RTC_SEC_FLAG=0;
		RTC_Get();
		Read_ACC_Data_Task();
		
		memset(ACC_buf,0,sizeof(ACC_buf));
		//��ʽת��
		len=sprintf(ACC_buf,"%d-%d-%d %d:%d:%d,%f,%f,%f,%f,%f,%f,%f,%f\r\n",Calendar.year,Calendar.month,Calendar.day,
					Calendar.hour,Calendar.min,Calendar.sec,Angle_x,Angle_y,Angle_z,Angle_x,Angle_y,Angle_z,Angle_x,Angle_y);

		res=f_open(&fil, "0:/data.csv", FA_CREATE_NEW  | FA_WRITE );//���ļ��������Ǵ�����дģʽ��
		if (res != FR_OK)
		{
			res=f_open(&fil,"0:/data.csv",FA_OPEN_EXISTING | FA_WRITE);  
			if (res != FR_OK)
			{
				log_info("���ļ�ʧ�� (%d)\r\n", res);
			}
			else 
			{
				log_info("���ļ��ɹ�\r\n");
				ltsp++;
				if(ltsp==1)//����ǵ�һ�У���д�����
				{
					res=f_write (&fil, "ʱ��,X��,Y��,Z��,X��,Y��,Z��,X��,Y��\r\n", sizeof("ʱ��,X��,Y��,Z��,X��,Y��,Z��,X��,Y��\r\n"), &bw);   //��SD����д������
					if (res != FR_OK)log_info("�ļ�д��ʧ�� (%d)\r\n", res);
					else 
					{
						ltsp++;
						log_info("�ļ�д��ɹ�\r\n");
					}
				}

				f_lseek(&fil, fil.fsize);

				res=f_write (&fil, ACC_buf,len, &bw);   //��SD����д������
				if (res != FR_OK)log_info("�ļ�д��ʧ�� (%d)\r\n", res);
				else log_info("�ļ�д��ɹ�\r\n");

				f_close (&fil);
			}
		}
	}
}

void Command_Task(void)
{
	volatile unsigned char *px=USART2_RX;
	unsigned short i=0;
	unsigned char j=0;
	unsigned char tempdata=0;
	
	char buf[64];
	
	if(Uart2Flag)
	{
		memset(buf,0,sizeof(buf));
		for(j=0;j<64;j++)
		{
			i=USART2_GetChar();
			if(i != EMPTY)buf[j]=i;
			else break;
		}
		log_info("%s",buf);
		USART2_ClearBuf_Flag();
			
		if(strncmp(buf,"AT+TIME",7)==0)Get_SYS_TIME_Task();
		else if(strncmp(buf,"AT+ACC",6)==0)Read_ACC_Data_Task();
		else
		log_info("command error\r\n");
	}
	/*
	volatile unsigned char *px=USART2_RX;
	unsigned short i=0;
	unsigned char j=0;
	unsigned char tempdata=0;
	
	if(Uart2Flag)
		{
			if(USART2_CommandLen>=UART2_RBUF_SIZE/2)     //���ƽ����ַ�����Ϊ128�ֽ�
			{
				log_info("USART2_CommandLen:%d\r\n",USART2_CommandLen);
				log_info("USART2_RX Data Count exceed %d!\r\n",UART2_RBUF_SIZE/2);
				USART2_ClearBuf_Flag();
			}
			else
			{
				for(i=0;i<USART2_CommandLen;i++){tempdata=USART2_GetChar();if(tempdata != '\0')USART2_RX[j++]=tempdata;}//��ȡ�������е�����
				USART2_RX[j]=0x0;log_info("%s\r\n",px);j=0;
				
			
				if( ('A' == *px) && ('T' == *(px+1)) && ('+' == *(px+2)) )
				{
					if( ('A' == *(px+3)) && ('C' == *(px+4)) && ('C' == *(px+5)) )Read_ACC_Data_Task();
					if( ('T' == *(px+3)) && ('I' == *(px+4)) && ('M' == *(px+5)) && ('E' == *(px+6)) )Get_SYS_TIME_Task();
				}
				else 
					log_info("Instructions are Err!\r\n");
			
				USART2_ClearBuf_Flag();
			}
		}*/
}

void SD_Read_AND_WriteTest()
{
		unsigned int i=0;
		//unsigned char j=0;
		u32 sd_size=0;
		u32 sector=0;
		unsigned int sector_cnt=1;
		unsigned char SD_RTbuf[512]={0};
		//unsigned char *SD_RTbuf=0;
		
		/*����������������1*/
		sector=0;
		sector_cnt=1;
		//SD_RTbuf=mymalloc(0,512);		//�����ڴ�
		log_info("Test1--->>>Read %d sector: ",sector);
		if(SD_ReadDisk(SD_RTbuf,sector,sector_cnt)==SD_OK)	//��ȡ0����������
		{
			for(sd_size=0;sd_size<512;sd_size++)log_info("%x ",SD_RTbuf[sd_size]);//��ӡ0�������� 
			log_info("\r\n");			
		}
		//myfree(0,SD_RTbuf);//�ͷ��ڴ�	
		
		
//		/*������д��д����2,Ȼ�������2�Ա�*/
//		sector=2;
//		sector_cnt=1;
//		SD_RTbuf=mymalloc(0,512);		//�����ڴ�
//		for(i=0;i<512;i++)SD_RTbuf[i]=i;
//		if(SD_WriteDisk(SD_RTbuf,sector,sector_cnt)==SD_OK)
//		{
//			log_info("Test2--->>>WriteDisk is OK! sector:%d\r\n",sector);
//		}
//		myfree(0,SD_RTbuf);//�ͷ��ڴ�
//		Delay(10);
//		SD_RTbuf=mymalloc(0,512);		//�����ڴ�
//		log_info("Read SD_Card %d sector:\r\n ",sector);
//		if(SD_ReadDisk(SD_RTbuf,sector,sector_cnt)==SD_OK)	//��ȡ0����������
//		{
//			for(sd_size=0;sd_size<512;sd_size++)log_info("%x ",SD_RTbuf[sd_size]);//��ӡ0��������
//			log_info("\r\n");			
//		}
//		myfree(0,SD_RTbuf);//�ͷ��ڴ�	
//		
//		/*������������������������0��1��2��3*/
//		sector=0;
//		sector_cnt=4;
//		SD_RTbuf=mymalloc(0,512*sector_cnt);		//�����ڴ�
//		log_info("Test3--->>>Read 0 and 1 and 2 and 3 sector:\r\n");
//		if(SD_ReadDisk(SD_RTbuf,sector,sector_cnt)==SD_OK)	//��ȡ0����������
//		{
//			for(sd_size=0;sd_size<512*sector_cnt;sd_size++)log_info("%x ",SD_RTbuf[sd_size]);//��ӡ0��������   	   
//		}
//		log_info("\r\n");
//		myfree(0,SD_RTbuf);//�ͷ��ڴ�
//	
//		
//		
//		
//		/*д�������������д����1��2��3��4*/
//		sector=1;
//		sector_cnt=4;
//		SD_RTbuf=mymalloc(0,512*sector_cnt);		//�����ڴ�
//		for(i=0;i<512*sector_cnt;i++)SD_RTbuf[i]=i;
//		if(SD_WriteDisk(SD_RTbuf,sector,sector_cnt)==SD_OK)
//		{
//			log_info("Test4--->>>WriteDisk is OK! sector:%d\r\n",sector);
//		}
//		myfree(0,SD_RTbuf);//�ͷ��ڴ�
//		
//		
//		
//		//������0-9������
//		sector=0;
//		for(i=0;i<10;i++)
//		{
//			log_info("Test5--->>>Read SD_Card %d sector:\r\n",i);
//			SD_RTbuf=mymalloc(0,512);		//�����ڴ�
//			if(SD_ReadDisk(SD_RTbuf,i,sector_cnt)==SD_OK)	//��ȡ0����������
//			{
//				for(sd_size=0;sd_size<512;sd_size++)log_info("%x ",SD_RTbuf[sd_size]);//��ӡ0��������   	   
//			}
//			log_info("\r\n");
//			myfree(0,SD_RTbuf);//�ͷ��ڴ�
//			Delay(10);
//		}
		
		//SD_PowerOFF();
}

void SD_FATFS_TEST()
{
	unsigned int lt=0;
	/************         SD���ļ�ϵͳ����           ****************/	

	/* �����ļ�ϵͳ */
	res=f_mount(&fs,"0:",0); 	//����SD�� 
	if (res != FR_OK)log_info("�����ļ�ϵͳʧ�� (%d)\r\n", res);
	else log_info("�����ļ�ϵͳ�ɹ�\r\n");
	
//	/* �򿪸��ļ��� */
//	res = f_opendir(&DirInf, "/"); /* ���������������ӵ�ǰĿ¼��ʼ */
//	if (res != FR_OK)log_info("�򿪸�Ŀ¼ʧ�� (%d)\r\n", res);
//	else log_info("�򿪸�Ŀ¼�ɹ�\r\n");
	
	/* ���ļ� */
	res=f_open(&fil, "0:/message.txt", FA_CREATE_ALWAYS | FA_WRITE  );//���ļ��������Ǵ�����дģʽ��
	if (res != FR_OK)log_info("���ļ�ʧ�� (%d)\r\n", res);
	else log_info("���ļ��ɹ�\r\n");
	
	/* д���ļ� */
	res=f_write (&fil, "this is a SDCard & FATFS test code!\r\n", sizeof("this is a SDCard & FATFS test code!\r\n"), &bw);   //��SD����д������
	if (res != FR_OK)log_info("�ļ�д��ʧ�� (%d)\r\n", res);
	else log_info("�ļ�д��ɹ�\r\n");
	
	f_close (&fil);
	
	
	res=f_open (&fil, "0:/message.txt", FA_WRITE );
	if (res != FR_OK)log_info("���ļ�ʧ�� (%d)\r\n", res);
	else log_info("���ļ��ɹ�\r\n");
	
	f_lseek(&fil, fil.fsize-1);
	res=f_write (&fil, "Do one thing at a time, and do well.2018��3��2��\r\n", sizeof("Do one thing at a time, and do well.2018��3��2��\r\n"), &bw);   //��SD����д������
	if (res != FR_OK)log_info("�ļ�д��ʧ�� (%d)\r\n", res);
	else log_info("�ļ�д��ɹ�\r\n");
	
	
	f_lseek(&fil, fil.fsize-1);
	res=f_write (&fil, "Do one thing at a time, and do well.\r\n", sizeof("Do one thing at a time, and do well.\r\n"), &bw);   //��SD����д������
	if (res != FR_OK)log_info("�ļ�д��ʧ�� (%d)\r\n", res);
	else log_info("�ļ�д��ɹ�\r\n");
	
	f_close (&fil);
	
	
	res=f_open (&fil, "0:/message.txt", FA_READ );
	if (res != FR_OK)log_info("���ļ�ʧ�� (%d)\r\n", res);
	else log_info("���ļ��ɹ�\r\n");
	
	
	lt=3;
	while(lt)
	{
		lt--;
		f_gets(FA_BUF,255,&fil);
		log_info("%s\r\n",FA_BUF);
	}
	f_close (&fil);
}


/*******************************************************************************
* Function Name  : Main function
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
extern u8 Max_Lun;	//֧�ֵĴ��̸���,0��ʾ1��,1��ʾ2��.
int main(void)
{
	unsigned int lt=0;
	ACC_buf_t ACC_buf;
	
	u8 offline_cnt=0;
	u8 tct=0;
	u8 USB_STA;
	u8 Divece_STA; 
	
	Init_Hardware();
	
	
/************         SD��ֱ�Ӷ�д����           ****************/	
	SD_Read_AND_WriteTest();//SD����д����
	
	SD_FATFS_TEST();

	
	//USB����
	Mass_Memory_Size[0]=(uint64_t)SDCardInfo.CardCapacity*1024;		//�õ�SD���������ֽڣ�����SD����������4G��ʱ��,��Ҫ�õ�����u32����ʾ
	Mass_Block_Size[0] =512;							//��Ϊ������Init����������SD���Ĳ����ֽ�Ϊ512��,��������һ����512���ֽ�.
	Mass_Block_Count[0]=Mass_Memory_Size[0]/Mass_Block_Size[0];
	Delay(1800);
	USB_Port_Set(0); 	//USB�ȶϿ�
	Delay(700);
	USB_Port_Set(1);	//USB�ٴ�����
	log_info("USB Connecting...\r\n");
	Data_Buffer=mymalloc(SRAMIN,BULK_MAX_PACKET_SIZE*2*4);	//ΪUSB���ݻ����������ڴ�
	Bulk_Data_Buff=mymalloc(SRAMIN,BULK_MAX_PACKET_SIZE);	//�����ڴ�
	
 	USB_Interrupts_Config();    
 	Set_USBClock();   
 	USB_Init();	    
	Delay(1800);
	
	//log_info(">>>>>>>>      ACC_buf len:%d      <<<<<<<<<\r\n",sizeof(ACC_buf));
	//RTC_Set(2017,10,31,22,01,45);
	log_info("Hardware ready!\r\n");
	while(1)
	{
		//Update_RTC_Task();//ÿ���Զ�����ϵͳʱ��
		Command_Task();   //���մ���ָ��
		
		

		Delay(1);
		
		if(USB_STA!=USB_STATUS_REG)//״̬�ı��� 
		{	 						   		  	   
			if(USB_STATUS_REG&0x01)//����д		  
			{
				log_info("USB Writing...\r\n");//��ʾUSB����д������	 
			}
			if(USB_STATUS_REG&0x02)//���ڶ�
			{
				log_info("USB Reading...\r\n");//��ʾUSB���ڶ�������  		 
			}	 										  
			if(USB_STATUS_REG&0x04)log_info("USB Write Err \r\n");//��ʾд�����
			if(USB_STATUS_REG&0x08)log_info("USB Read  Err \r\n");//��ʾ��������
			USB_STA=USB_STATUS_REG;//��¼����״̬
		}
		
		if(Divece_STA!=bDeviceState) 
		{
			if(bDeviceState==CONFIGURED)log_info("USB Connected    \r\n");//��ʾUSB�����Ѿ�����
			else log_info("USB DisConnected    \r\n");//��ʾUSB���γ���
			Divece_STA=bDeviceState;
		}
		tct++;
		if(tct==200)
		{
			tct=0;
			
			if(USB_STATUS_REG&0x10)
			{
				offline_cnt=0;//USB������,�����offline������
				bDeviceState=CONFIGURED;
			}else//û�еõ���ѯ 
			{
				offline_cnt++;  
				if(offline_cnt>10)bDeviceState=UNCONNECTED;//2s��û�յ����߱��,����USB���γ���
			}
			USB_STATUS_REG=0;
		}
		
		
	}
	//if(0)SystemResetSoft();
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
