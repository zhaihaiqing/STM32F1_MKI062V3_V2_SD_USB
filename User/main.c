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
  * *本工程是建立在在MKI062V3验证板（开发板）上的，旨在验证加速度传感器、磁场计
  *（IMU单元）、气压计等传感器的测试，相关驱动代码实现。
  *
	*2017-7-7完成加速度传感器代码
	*2017-8-15完成片上RTC测试
	*2017-10-15完成SD卡读写操作（无文件系统）
  **V2版本旨在测试串口处理字符型数据的能力，并验证类似AT指令集功能。
	**测试SD卡读写功能
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
* Description    : 延时函数，空闲时进入sleep模式，采用systick定时器实现
* Input          : 延时时长（ms）
* Output         : None
* Return         : None
*******************************************************************************/
void Delay(volatile unsigned int nTime)
{ 
	TimingDelay = nTime;
	while(TimingDelay != 0);//__WFI;//等待过程中进入sleep模式,下方指令在执行过程中容易进入
}
/*******************************************************************************
* Function Name  : SysTick init function
* Description    : 设置定时长度及SysTick优先级
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Init_SysTick(void)
{
	SysTick_Config(SystemCoreClock / 1000); //设置定时长度，1ms
	NVIC_SetPriority(SysTick_IRQn, 0x0);    //SysTick中断优先级设置
}

/*******************************************************************************
* Function Name  : Init_Hardware
* Description    : 完成硬件设备的初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void Init_Hardware()
{
	UART2_Configuration(115200);log_info("Init_UART2 OK!\r\n");
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);//中断优先级分组
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
		log_info("%d年%d月%d日,%d:%d:%d\r\n",Calendar.year,Calendar.month,Calendar.day,Calendar.hour,Calendar.min,Calendar.sec);
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
		//格式转换
		len=sprintf(ACC_buf,"%d-%d-%d %d:%d:%d,%f,%f,%f,%f,%f,%f,%f,%f\r\n",Calendar.year,Calendar.month,Calendar.day,
					Calendar.hour,Calendar.min,Calendar.sec,Angle_x,Angle_y,Angle_z,Angle_x,Angle_y,Angle_z,Angle_x,Angle_y);

		res=f_open(&fil, "0:/data.csv", FA_CREATE_NEW  | FA_WRITE );//打开文件，以总是创建和写模式打开
		if (res != FR_OK)
		{
			res=f_open(&fil,"0:/data.csv",FA_OPEN_EXISTING | FA_WRITE);  
			if (res != FR_OK)
			{
				log_info("打开文件失败 (%d)\r\n", res);
			}
			else 
			{
				log_info("打开文件成功\r\n");
				ltsp++;
				if(ltsp==1)//如果是第一行，则写入标题
				{
					res=f_write (&fil, "时间,X轴,Y轴,Z轴,X轴,Y轴,Z轴,X轴,Y轴\r\n", sizeof("时间,X轴,Y轴,Z轴,X轴,Y轴,Z轴,X轴,Y轴\r\n"), &bw);   //向SD卡中写入数据
					if (res != FR_OK)log_info("文件写入失败 (%d)\r\n", res);
					else 
					{
						ltsp++;
						log_info("文件写入成功\r\n");
					}
				}

				f_lseek(&fil, fil.fsize);

				res=f_write (&fil, ACC_buf,len, &bw);   //向SD卡中写入数据
				if (res != FR_OK)log_info("文件写入失败 (%d)\r\n", res);
				else log_info("文件写入成功\r\n");

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
			if(USART2_CommandLen>=UART2_RBUF_SIZE/2)     //限制接收字符长度为128字节
			{
				log_info("USART2_CommandLen:%d\r\n",USART2_CommandLen);
				log_info("USART2_RX Data Count exceed %d!\r\n",UART2_RBUF_SIZE/2);
				USART2_ClearBuf_Flag();
			}
			else
			{
				for(i=0;i<USART2_CommandLen;i++){tempdata=USART2_GetChar();if(tempdata != '\0')USART2_RX[j++]=tempdata;}//获取缓冲区中的数据
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
		
		/*单扇区读，读扇区1*/
		sector=0;
		sector_cnt=1;
		//SD_RTbuf=mymalloc(0,512);		//申请内存
		log_info("Test1--->>>Read %d sector: ",sector);
		if(SD_ReadDisk(SD_RTbuf,sector,sector_cnt)==SD_OK)	//读取0扇区的内容
		{
			for(sd_size=0;sd_size<512;sd_size++)log_info("%x ",SD_RTbuf[sd_size]);//打印0扇区数据 
			log_info("\r\n");			
		}
		//myfree(0,SD_RTbuf);//释放内存	
		
		
//		/*单扇区写，写扇区2,然后读扇区2对比*/
//		sector=2;
//		sector_cnt=1;
//		SD_RTbuf=mymalloc(0,512);		//申请内存
//		for(i=0;i<512;i++)SD_RTbuf[i]=i;
//		if(SD_WriteDisk(SD_RTbuf,sector,sector_cnt)==SD_OK)
//		{
//			log_info("Test2--->>>WriteDisk is OK! sector:%d\r\n",sector);
//		}
//		myfree(0,SD_RTbuf);//释放内存
//		Delay(10);
//		SD_RTbuf=mymalloc(0,512);		//申请内存
//		log_info("Read SD_Card %d sector:\r\n ",sector);
//		if(SD_ReadDisk(SD_RTbuf,sector,sector_cnt)==SD_OK)	//读取0扇区的内容
//		{
//			for(sd_size=0;sd_size<512;sd_size++)log_info("%x ",SD_RTbuf[sd_size]);//打印0扇区数据
//			log_info("\r\n");			
//		}
//		myfree(0,SD_RTbuf);//释放内存	
//		
//		/*多扇区连续读，连续读扇区0、1、2、3*/
//		sector=0;
//		sector_cnt=4;
//		SD_RTbuf=mymalloc(0,512*sector_cnt);		//申请内存
//		log_info("Test3--->>>Read 0 and 1 and 2 and 3 sector:\r\n");
//		if(SD_ReadDisk(SD_RTbuf,sector,sector_cnt)==SD_OK)	//读取0扇区的内容
//		{
//			for(sd_size=0;sd_size<512*sector_cnt;sd_size++)log_info("%x ",SD_RTbuf[sd_size]);//打印0扇区数据   	   
//		}
//		log_info("\r\n");
//		myfree(0,SD_RTbuf);//释放内存
//	
//		
//		
//		
//		/*写多个扇区，连续写扇区1、2、3、4*/
//		sector=1;
//		sector_cnt=4;
//		SD_RTbuf=mymalloc(0,512*sector_cnt);		//申请内存
//		for(i=0;i<512*sector_cnt;i++)SD_RTbuf[i]=i;
//		if(SD_WriteDisk(SD_RTbuf,sector,sector_cnt)==SD_OK)
//		{
//			log_info("Test4--->>>WriteDisk is OK! sector:%d\r\n",sector);
//		}
//		myfree(0,SD_RTbuf);//释放内存
//		
//		
//		
//		//多扇区0-9扇区读
//		sector=0;
//		for(i=0;i<10;i++)
//		{
//			log_info("Test5--->>>Read SD_Card %d sector:\r\n",i);
//			SD_RTbuf=mymalloc(0,512);		//申请内存
//			if(SD_ReadDisk(SD_RTbuf,i,sector_cnt)==SD_OK)	//读取0扇区的内容
//			{
//				for(sd_size=0;sd_size<512;sd_size++)log_info("%x ",SD_RTbuf[sd_size]);//打印0扇区数据   	   
//			}
//			log_info("\r\n");
//			myfree(0,SD_RTbuf);//释放内存
//			Delay(10);
//		}
		
		//SD_PowerOFF();
}

void SD_FATFS_TEST()
{
	unsigned int lt=0;
	/************         SD卡文件系统测试           ****************/	

	/* 挂载文件系统 */
	res=f_mount(&fs,"0:",0); 	//挂载SD卡 
	if (res != FR_OK)log_info("挂载文件系统失败 (%d)\r\n", res);
	else log_info("挂载文件系统成功\r\n");
	
//	/* 打开根文件夹 */
//	res = f_opendir(&DirInf, "/"); /* 如果不带参数，则从当前目录开始 */
//	if (res != FR_OK)log_info("打开根目录失败 (%d)\r\n", res);
//	else log_info("打开根目录成功\r\n");
	
	/* 打开文件 */
	res=f_open(&fil, "0:/message.txt", FA_CREATE_ALWAYS | FA_WRITE  );//打开文件，以总是创建和写模式打开
	if (res != FR_OK)log_info("打开文件失败 (%d)\r\n", res);
	else log_info("打开文件成功\r\n");
	
	/* 写入文件 */
	res=f_write (&fil, "this is a SDCard & FATFS test code!\r\n", sizeof("this is a SDCard & FATFS test code!\r\n"), &bw);   //向SD卡中写入数据
	if (res != FR_OK)log_info("文件写入失败 (%d)\r\n", res);
	else log_info("文件写入成功\r\n");
	
	f_close (&fil);
	
	
	res=f_open (&fil, "0:/message.txt", FA_WRITE );
	if (res != FR_OK)log_info("打开文件失败 (%d)\r\n", res);
	else log_info("打开文件成功\r\n");
	
	f_lseek(&fil, fil.fsize-1);
	res=f_write (&fil, "Do one thing at a time, and do well.2018年3月2日\r\n", sizeof("Do one thing at a time, and do well.2018年3月2日\r\n"), &bw);   //向SD卡中写入数据
	if (res != FR_OK)log_info("文件写入失败 (%d)\r\n", res);
	else log_info("文件写入成功\r\n");
	
	
	f_lseek(&fil, fil.fsize-1);
	res=f_write (&fil, "Do one thing at a time, and do well.\r\n", sizeof("Do one thing at a time, and do well.\r\n"), &bw);   //向SD卡中写入数据
	if (res != FR_OK)log_info("文件写入失败 (%d)\r\n", res);
	else log_info("文件写入成功\r\n");
	
	f_close (&fil);
	
	
	res=f_open (&fil, "0:/message.txt", FA_READ );
	if (res != FR_OK)log_info("打开文件失败 (%d)\r\n", res);
	else log_info("打开文件成功\r\n");
	
	
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
extern u8 Max_Lun;	//支持的磁盘个数,0表示1个,1表示2个.
int main(void)
{
	unsigned int lt=0;
	ACC_buf_t ACC_buf;
	
	u8 offline_cnt=0;
	u8 tct=0;
	u8 USB_STA;
	u8 Divece_STA; 
	
	Init_Hardware();
	
	
/************         SD卡直接读写测试           ****************/	
	SD_Read_AND_WriteTest();//SD卡读写测试
	
	SD_FATFS_TEST();

	
	//USB配置
	Mass_Memory_Size[0]=(uint64_t)SDCardInfo.CardCapacity*1024;		//得到SD卡容量（字节），当SD卡容量超过4G的时候,需要用到两个u32来表示
	Mass_Block_Size[0] =512;							//因为我们在Init里面设置了SD卡的操作字节为512个,所以这里一定是512个字节.
	Mass_Block_Count[0]=Mass_Memory_Size[0]/Mass_Block_Size[0];
	Delay(1800);
	USB_Port_Set(0); 	//USB先断开
	Delay(700);
	USB_Port_Set(1);	//USB再次连接
	log_info("USB Connecting...\r\n");
	Data_Buffer=mymalloc(SRAMIN,BULK_MAX_PACKET_SIZE*2*4);	//为USB数据缓存区申请内存
	Bulk_Data_Buff=mymalloc(SRAMIN,BULK_MAX_PACKET_SIZE);	//申请内存
	
 	USB_Interrupts_Config();    
 	Set_USBClock();   
 	USB_Init();	    
	Delay(1800);
	
	//log_info(">>>>>>>>      ACC_buf len:%d      <<<<<<<<<\r\n",sizeof(ACC_buf));
	//RTC_Set(2017,10,31,22,01,45);
	log_info("Hardware ready!\r\n");
	while(1)
	{
		//Update_RTC_Task();//每秒自动更新系统时间
		Command_Task();   //接收串口指令
		
		

		Delay(1);
		
		if(USB_STA!=USB_STATUS_REG)//状态改变了 
		{	 						   		  	   
			if(USB_STATUS_REG&0x01)//正在写		  
			{
				log_info("USB Writing...\r\n");//提示USB正在写入数据	 
			}
			if(USB_STATUS_REG&0x02)//正在读
			{
				log_info("USB Reading...\r\n");//提示USB正在读出数据  		 
			}	 										  
			if(USB_STATUS_REG&0x04)log_info("USB Write Err \r\n");//提示写入错误
			if(USB_STATUS_REG&0x08)log_info("USB Read  Err \r\n");//提示读出错误
			USB_STA=USB_STATUS_REG;//记录最后的状态
		}
		
		if(Divece_STA!=bDeviceState) 
		{
			if(bDeviceState==CONFIGURED)log_info("USB Connected    \r\n");//提示USB连接已经建立
			else log_info("USB DisConnected    \r\n");//提示USB被拔出了
			Divece_STA=bDeviceState;
		}
		tct++;
		if(tct==200)
		{
			tct=0;
			
			if(USB_STATUS_REG&0x10)
			{
				offline_cnt=0;//USB连接了,则清除offline计数器
				bDeviceState=CONFIGURED;
			}else//没有得到轮询 
			{
				offline_cnt++;  
				if(offline_cnt>10)bDeviceState=UNCONNECTED;//2s内没收到在线标记,代表USB被拔出了
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
