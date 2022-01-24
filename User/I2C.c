
#include "main.h"

#define I2C1_SLAVE_ADDRESS7   0x30     //�����ַ
#define ClockSpeed            400000   //����400K

#define I2C_FLAG_TimeOut  		0xffff     //��ʱ���� 0x5000
#define I2C_LONG_TimeOut  		(10 * I2C_FLAG_TimeOut)

/*******************************************************************************
* Function Name  : I2C1_Configuration
* 
*******************************************************************************/
void I2C1_Configuration(void)
{
	//����GPIO�ṹ�壬����I2C�ṹ��
  GPIO_InitTypeDef GPIO_InitStructure;
  I2C_InitTypeDef  I2C_InitStructure;
  
  /* Enable I2C1 ��GPIO clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	/* Reset I2C1 peripheral */
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1,  ENABLE);
	/* Release reset signal of I2C1 IP */
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);
	
  /* Configure I2C1 pins: SCL and SDA --GPIO6 and GPIO7 */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;//���踴��ģʽ
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  /* I2C1 configuration ------------------------------------------------------*/
  I2C_DeInit(I2C1);//I2C��ʼ��
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;//����ΪI2Cģʽ
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;//������ֻ����I2C�����ڿ���ģʽ�£�ʱ�ӹ���Ƶ�ʸ���100KHz���²�������
  I2C_InitStructure.I2C_OwnAddress1 = I2C1_SLAVE_ADDRESS7;//���õ�һ���豸�������ַ
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;//ʹ��Ӧ��
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;//AT24C64��ַΪ7λ����������7λ
  I2C_InitStructure.I2C_ClockSpeed = ClockSpeed;//����ʱ���ٶȣ�
  I2C_AcknowledgeConfig(I2C1, ENABLE);  //����1�ֽ�1Ӧ��ģʽ
 
  I2C_Cmd(I2C1, ENABLE);
  I2C_Init(I2C1, &I2C_InitStructure);
  I2C_AcknowledgeConfig(I2C1,ENABLE);  
}

/****************************************************
**������:I2C_Read_Byte
**����:
**I2C��׼����
****************************************************/
unsigned char I2C1_Read_Byte(unsigned char Device_Addr,unsigned char Reg_Addr)
{
  unsigned int timeout;
  unsigned char Reg_Value;
  timeout = I2C_FLAG_TimeOut;                //���ó�ʱ����
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) && (timeout--));
  I2C_AcknowledgeConfig(I2C1, ENABLE);       //����1�ֽ�1Ӧ��ģʽ

  I2C_GenerateSTART(I2C1, ENABLE);//������ʼλ 
  timeout = I2C_FLAG_TimeOut;     //���ó�ʱ����  
  while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) && (timeout--));//EV5,��ģʽ

  I2C_Send7bitAddress(I2C1,  Device_Addr, I2C_Direction_Transmitter);//����������ַ(д)
  timeout = I2C_FLAG_TimeOut;//���ó�ʱ����  
  while ((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) && (timeout--));

  I2C_SendData(I2C1, Reg_Addr); //���ͼĴ�����ַ,
  timeout = I2C_FLAG_TimeOut;   //���ó�ʱ����
  while ((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) && (timeout--));//�����ѷ���

  I2C_GenerateSTART(I2C1, ENABLE);//������ʼλ
  timeout = I2C_FLAG_TimeOut;     //���ó�ʱ����
  while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) && (timeout--));

  I2C_Send7bitAddress(I2C1, Device_Addr, I2C_Direction_Receiver);//����������ַ��������
  timeout = I2C_FLAG_TimeOut;    //���ó�ʱ����
  while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) && (timeout--));
	
  I2C_AcknowledgeConfig(I2C1, DISABLE);	    //���һλ��Ҫ�ر�Ӧ���
  I2C_GenerateSTOP(I2C1, ENABLE);			//����ֹͣλ

  timeout = I2C_FLAG_TimeOut;//���ó�ʱ����
  while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)) && (timeout--)); /* EV7 */
  Reg_Value = I2C_ReceiveData(I2C1);
		
  I2C_AcknowledgeConfig(I2C1, ENABLE);//�ٴ�����Ӧ��ģʽ
  
  return Reg_Value;
}

/*************************************************
**������:I2C_Write_Byte
**����:
**I2C��׼����
*************************************************/
void I2C_Write_Byte(unsigned char Device_Addr,unsigned char Reg_Addr,unsigned char Reg_Value)
{
  unsigned int timeout;
  
  I2C_GenerateSTART(I2C1, ENABLE);//��ʼλ
  timeout = I2C_FLAG_TimeOut;//���ó�ʱ����
  while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) && (timeout--));	

  I2C_Send7bitAddress(I2C1, Device_Addr, I2C_Direction_Transmitter);//������ַ(д)
  timeout = I2C_FLAG_TimeOut;//���ó�ʱ����
  while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) && (timeout--));

  I2C_SendData(I2C1, Reg_Addr); //���͸�8λ��ַ
  timeout = I2C_FLAG_TimeOut;//���ó�ʱ����  
  while ((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) && (timeout--));//�����ѷ���
  
  I2C_SendData(I2C1, Reg_Value); //������Ҫд�������
  timeout = I2C_FLAG_TimeOut;//���ó�ʱ����
  while ((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) && (timeout--));

  I2C_GenerateSTOP(I2C1, ENABLE);//ֹͣλ
}


/***************************************************
**������:I2C1_ReadS
**����:��ȡEEPROM����ֽڣ����8192
**
***************************************************/
void I2C1_ReadS(unsigned short addr ,unsigned char * pBuffer,unsigned short Length)
{
  unsigned int timeout;
  if(Length==0)return;  //����Ϊ0ֱ�ӷ���
  timeout = I2C_FLAG_TimeOut;//���ó�ʱ����
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY) && (timeout--));

  /*����1�ֽ�1Ӧ��ģʽ*/
  I2C_AcknowledgeConfig(I2C1, ENABLE);

  /* ������ʼλ */
  I2C_GenerateSTART(I2C1, ENABLE);
  timeout = I2C_FLAG_TimeOut;//���ó�ʱ����  
  while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) && (timeout--));/*EV5,��ģʽ*/

  /*����������ַ(д)*/
  I2C_Send7bitAddress(I2C1,  ACC_ADDR, I2C_Direction_Transmitter);
  timeout = I2C_FLAG_TimeOut;//���ó�ʱ����  
  while ((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) && (timeout--));

  /*���͵�ַ,,��ַ�������֣��ߵ�ַ���ֽڵ�ַ���͵�ַ��ҳ��ַ*/
  I2C_SendData(I2C1, (addr & 0xff00) >> 8); //���͸�8λ��ַ
  timeout = I2C_FLAG_TimeOut;//���ó�ʱ����
  while ((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) && (timeout--));/*�����ѷ���*/
  I2C_SendData(I2C1, addr & 0x00ff); //���͵�8λ��ַ
  timeout = I2C_FLAG_TimeOut;//���ó�ʱ����
  while ((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) && (timeout--));/*�����ѷ���*/

  /*��ʼλ*/
  I2C_GenerateSTART(I2C1, ENABLE);
  timeout = I2C_FLAG_TimeOut;//���ó�ʱ����
  while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) && (timeout--));

  /*������*/
  I2C_Send7bitAddress(I2C1, ACC_ADDR, I2C_Direction_Receiver);
  timeout = I2C_FLAG_TimeOut;//���ó�ʱ����
  while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) && (timeout--));
  while (Length)
  {
    if(Length==1)
		{
			I2C_AcknowledgeConfig(I2C1, DISABLE);	//���һλ��Ҫ�ر�Ӧ���
			I2C_GenerateSTOP(I2C1, ENABLE);			//����ֹͣλ
		}

		timeout = I2C_FLAG_TimeOut;//���ó�ʱ����
		while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)) && (timeout--)); /* EV7 */
		*pBuffer = I2C_ReceiveData(I2C1);
		pBuffer++;
		/* Decrement the read bytes counter */
		Length--;
  }
  //�ٴ�����Ӧ��ģʽ
  I2C_AcknowledgeConfig(I2C1, ENABLE);
}
/*************************************************
**������:I2C_WriteS
**����:
*************************************************/
unsigned char I2C_WriteS(unsigned short addr,unsigned char* pBuffer, unsigned char Length)
{
  unsigned int timeout;
  /*��ʼλ*/
  I2C_GenerateSTART(I2C1, ENABLE);
  timeout = I2C_FLAG_TimeOut;//���ó�ʱ����
  while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) && (timeout--));
	if(!timeout)return ERROR;	

  /*������ַ(д)*/
  I2C_Send7bitAddress(I2C1, ACC_ADDR, I2C_Direction_Transmitter);
  timeout = I2C_FLAG_TimeOut;//���ó�ʱ����
  while((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) && (timeout--));
	if(!timeout)return ERROR;

  /*д��ֵַ*/
  I2C_SendData(I2C1, (addr & 0xff00) >> 8); //���͸�8λ��ַ
  timeout = I2C_FLAG_TimeOut;//���ó�ʱ����  
  while ((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) && (timeout--));/*�����ѷ���*/
	if(!timeout)return ERROR;
  I2C_SendData(I2C1, addr & 0x00ff); //���͵�8λ��ַ
  timeout = I2C_FLAG_TimeOut;//���ó�ʱ����  
  while ((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) && (timeout--));/*�����ѷ���*/
	if(!timeout)return ERROR;

  while(Length--)  
  {
    I2C_SendData(I2C1, *pBuffer++); 
    timeout = I2C_FLAG_TimeOut;//���ó�ʱ����
    while ((!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) && (timeout--));
		if(!timeout)return ERROR;
  }
  /*ֹͣλ*/
  I2C_GenerateSTOP(I2C1, ENABLE);
	return SUCCESS;
}





