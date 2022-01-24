
#include "main.h"

#define I2C1_SLAVE_ADDRESS7   0x30     //自身地址
#define ClockSpeed            400000   //速率400K

#define I2C_FLAG_TimeOut  		0xffff     //超时常量 0x5000
#define I2C_LONG_TimeOut  		(10 * I2C_FLAG_TimeOut)

/*******************************************************************************
* Function Name  : I2C1_Configuration
* 
*******************************************************************************/
void I2C1_Configuration(void)
{
	//定义GPIO结构体，定义I2C结构体
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef  I2C_InitStructure;
  
  /* Enable I2C1 和GPIO clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	/* Reset I2C1 peripheral */
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1,  ENABLE);
	/* Release reset signal of I2C1 IP */
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);
	
  /* Configure I2C1 pins: SCL and SDA --GPIO6 and GPIO7 */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;//外设复用模式
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  /* I2C1 configuration ------------------------------------------------------*/
  I2C_DeInit(I2C1);//I2C初始化
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;//配置为I2C模式
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;//读参数只有在I2C工作在快速模式下（时钟工作频率高于100KHz）下才有意义
  I2C_InitStructure.I2C_OwnAddress1 = I2C1_SLAVE_ADDRESS7;//设置第一个设备的自身地址
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;//使能应答，
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;//AT24C64地址为7位，所以设置7位
  I2C_InitStructure.I2C_ClockSpeed = ClockSpeed;//设置时钟速度，
  I2C_AcknowledgeConfig(I2C1, ENABLE);  //允许1字节1应答模式
 
  I2C_Cmd(I2C1, ENABLE);
  I2C_Init(I2C1, &I2C_InitStructure);
  I2C_AcknowledgeConfig(I2C1,ENABLE);  
}

/****************************************************
**函数名:I2C_Read_Byte
**功能:
**I2C标准驱动
****************************************************/
unsigned char I2C1_Read_Byte(unsigned char Device_Addr,unsigned char Reg_Addr)
{
  unsigned int timeout;
  unsigned char Reg_Value;
  timeout = I2C_FLAG_TimeOut;                //设置超时变量
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) && (timeout--));
  I2C_AcknowledgeConfig(I2C1, ENABLE);       //允许1字节1应答模式

  I2C_GenerateSTART(I2C1, ENABLE);//发送起始位 
  timeout = I2C_FLAG_TimeOut;     //设置超时变量  
  while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) && (timeout--));//EV5,主模式

  I2C_Send7bitAddress(I2C1,  Device_Addr, I2C_Direction_Transmitter);//发送器件地址(写)
  timeout = I2C_FLAG_TimeOut;//设置超时变量  
  while ((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) && (timeout--));

  I2C_SendData(I2C1, Reg_Addr); //发送寄存器地址,
  timeout = I2C_FLAG_TimeOut;   //设置超时变量
  while ((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) && (timeout--));//数据已发送

  I2C_GenerateSTART(I2C1, ENABLE);//产生起始位
  timeout = I2C_FLAG_TimeOut;     //设置超时变量
  while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) && (timeout--));

  I2C_Send7bitAddress(I2C1, Device_Addr, I2C_Direction_Receiver);//发送器件地址，器件读
  timeout = I2C_FLAG_TimeOut;    //设置超时变量
  while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) && (timeout--));
	
  I2C_AcknowledgeConfig(I2C1, DISABLE);	    //最后一位后要关闭应答的
  I2C_GenerateSTOP(I2C1, ENABLE);			//发送停止位

  timeout = I2C_FLAG_TimeOut;//设置超时变量
  while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)) && (timeout--)); /* EV7 */
  Reg_Value = I2C_ReceiveData(I2C1);
		
  I2C_AcknowledgeConfig(I2C1, ENABLE);//再次允许应答模式
  
  return Reg_Value;
}

/*************************************************
**函数名:I2C_Write_Byte
**功能:
**I2C标准驱动
*************************************************/
void I2C_Write_Byte(unsigned char Device_Addr,unsigned char Reg_Addr,unsigned char Reg_Value)
{
  unsigned int timeout;
  
  I2C_GenerateSTART(I2C1, ENABLE);//起始位
  timeout = I2C_FLAG_TimeOut;//设置超时变量
  while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) && (timeout--));	

  I2C_Send7bitAddress(I2C1, Device_Addr, I2C_Direction_Transmitter);//器件地址(写)
  timeout = I2C_FLAG_TimeOut;//设置超时变量
  while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) && (timeout--));

  I2C_SendData(I2C1, Reg_Addr); //发送高8位地址
  timeout = I2C_FLAG_TimeOut;//设置超时变量  
  while ((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) && (timeout--));//数据已发送
  
  I2C_SendData(I2C1, Reg_Value); //发送需要写入的数据
  timeout = I2C_FLAG_TimeOut;//设置超时变量
  while ((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) && (timeout--));

  I2C_GenerateSTOP(I2C1, ENABLE);//停止位
}


/***************************************************
**函数名:I2C1_ReadS
**功能:读取EEPROM多个字节，最大8192
**
***************************************************/
void I2C1_ReadS(unsigned short addr ,unsigned char * pBuffer,unsigned short Length)
{
  unsigned int timeout;
  if(Length==0)return;  //长度为0直接返回
  timeout = I2C_FLAG_TimeOut;//设置超时变量
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) && (timeout--));

  /*允许1字节1应答模式*/
  I2C_AcknowledgeConfig(I2C1, ENABLE);

  /* 发送起始位 */
  I2C_GenerateSTART(I2C1, ENABLE);
  timeout = I2C_FLAG_TimeOut;//设置超时变量  
  while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) && (timeout--));/*EV5,主模式*/

  /*发送器件地址(写)*/
  I2C_Send7bitAddress(I2C1,  ACC_ADDR, I2C_Direction_Transmitter);
  timeout = I2C_FLAG_TimeOut;//设置超时变量  
  while ((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) && (timeout--));

  /*发送地址,,地址分两部分，高地址是字节地址，低地址是页地址*/
  I2C_SendData(I2C1, (addr & 0xff00) >> 8); //发送高8位地址
  timeout = I2C_FLAG_TimeOut;//设置超时变量
  while ((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) && (timeout--));/*数据已发送*/
  I2C_SendData(I2C1, addr & 0x00ff); //发送低8位地址
  timeout = I2C_FLAG_TimeOut;//设置超时变量
  while ((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) && (timeout--));/*数据已发送*/

  /*起始位*/
  I2C_GenerateSTART(I2C1, ENABLE);
  timeout = I2C_FLAG_TimeOut;//设置超时变量
  while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) && (timeout--));

  /*器件读*/
  I2C_Send7bitAddress(I2C1, ACC_ADDR, I2C_Direction_Receiver);
  timeout = I2C_FLAG_TimeOut;//设置超时变量
  while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) && (timeout--));
  while (Length)
  {
    if(Length==1)
		{
			I2C_AcknowledgeConfig(I2C1, DISABLE);	//最后一位后要关闭应答的
			I2C_GenerateSTOP(I2C1, ENABLE);			//发送停止位
		}

		timeout = I2C_FLAG_TimeOut;//设置超时变量
		while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)) && (timeout--)); /* EV7 */
		*pBuffer = I2C_ReceiveData(I2C1);
		pBuffer++;
		/* Decrement the read bytes counter */
		Length--;
  }
  //再次允许应答模式
  I2C_AcknowledgeConfig(I2C1, ENABLE);
}
/*************************************************
**函数名:I2C_WriteS
**功能:
*************************************************/
unsigned char I2C_WriteS(unsigned short addr,unsigned char* pBuffer, unsigned char Length)
{
  unsigned int timeout;
  /*起始位*/
  I2C_GenerateSTART(I2C1, ENABLE);
  timeout = I2C_FLAG_TimeOut;//设置超时变量
  while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) && (timeout--));
	if(!timeout)return ERROR;	

  /*器件地址(写)*/
  I2C_Send7bitAddress(I2C1, ACC_ADDR, I2C_Direction_Transmitter);
  timeout = I2C_FLAG_TimeOut;//设置超时变量
  while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) && (timeout--));
	if(!timeout)return ERROR;

  /*写地址值*/
  I2C_SendData(I2C1, (addr & 0xff00) >> 8); //发送高8位地址
  timeout = I2C_FLAG_TimeOut;//设置超时变量  
  while ((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) && (timeout--));/*数据已发送*/
	if(!timeout)return ERROR;
  I2C_SendData(I2C1, addr & 0x00ff); //发送低8位地址
  timeout = I2C_FLAG_TimeOut;//设置超时变量  
  while ((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) && (timeout--));/*数据已发送*/
	if(!timeout)return ERROR;

  while(Length--)  
  {
    I2C_SendData(I2C1, *pBuffer++); 
    timeout = I2C_FLAG_TimeOut;//设置超时变量
    while ((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) && (timeout--));
		if(!timeout)return ERROR;
  }
  /*停止位*/
  I2C_GenerateSTOP(I2C1, ENABLE);
	return SUCCESS;
}





