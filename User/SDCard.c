#include "main.h"

/******************** (C) COPYRIGHT 2009 STMicroelectronics ********************
* File Name          : sdcard.c
* Author             : MCD Application Team
* Version            : V3.0.1
* Date               : 04/27/2009
* Description        : This file provides all the SD Card driver firmware
*                      functions.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint32_t CardType = SDIO_STD_CAPACITY_SD_CARD_V2_0;
static uint32_t CSD_Tab[4], CID_Tab[4], RCA = 0;
static uint32_t DeviceMode = SD_POLLING_MODE;
static uint32_t TotalNumberOfBytes = 0, StopCondition = 0;
uint32_t *SrcBuffer, *DestBuffer;
__IO SD_Error TransferError = SD_OK;
__IO uint32_t TransferEnd = 0;
__IO uint32_t NumberOfBytes = 0;
SDIO_InitTypeDef SDIO_InitStructure;
SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
SDIO_DataInitTypeDef SDIO_DataInitStructure;

SDCardInfo_Type  SDCardInfo;


//SD_ReadDisk/SD_WriteDisk函数专用buf,当这两个函数的数据缓存区地址不是4字节对齐的时候,
//需要用到该数组,确保数据缓存区地址是4字节对齐的.
__align(4) unsigned char SDIO_DATA_BUFFER[512];	

/* Private function prototypes -----------------------------------------------*/
static SD_Error CmdError(void);
static SD_Error CmdResp1Error(uint8_t cmd);
static SD_Error CmdResp7Error(void);
static SD_Error CmdResp3Error(void);
static SD_Error CmdResp2Error(void);
static SD_Error CmdResp6Error(uint8_t cmd, uint16_t *prca);
static SD_Error SDEnWideBus(FunctionalState NewState);
static SD_Error IsCardProgramming(uint8_t *pstatus);
static SD_Error FindSCR(uint16_t rca, uint32_t *pscr);
static uint8_t convert_from_bytes_to_power_of_two(uint16_t NumberOfBytes);
static void SDCard_IO_Configuration(void);
static void DMA_TxConfiguration(uint32_t *BufferSRC, uint32_t BufferSize);
static void DMA_RxConfiguration(uint32_t *BufferDST, uint32_t BufferSize);


//THUMB指令不支持汇编内联
//采用如下方法实现执行汇编指令WFI  
void WFI_SET(void)
{
	__ASM volatile("wfi");		  
}
//关闭所有中断
void INTX_DISABLE(void)
{		  
	//__ASM volatile("cpsid i");
	//NVIC_SETPRIMASK();
	__set_PRIMASK(1); 
}
//开启所有中断
void INTX_ENABLE(void)
{
	//__ASM volatile("cpsie i");	
	//NVIC_RESETPRIMASK();
	__set_PRIMASK(0);
}
//设置栈顶地址
//addr:栈顶地址
__asm void MSR_MSP(u32 addr) 
{
    MSR MSP, r0 			//set Main Stack value
    BX r14
}

/* Private functions ---------------------------------------------------------*/


/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the SDIO Corresponding GPIO Ports
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void SDCard_IO_Configuration(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* GPIOC and GPIOD Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD , ENABLE);

  /* Configure PC.08, PC.09, PC.10, PC.11, PC.12 pin: D0, D1, D2, D3, CLK pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure PD.02 CMD line */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

//SDIO时钟初始化设置
//clkdiv:时钟分频系数
//CK时钟=SDIOCLK/[clkdiv+2];(SDIOCLK时钟固定为48Mhz)
void SDIO_Clock_Set(u8 clkdiv)
{
	u32 tmpreg=SDIO->CLKCR; 
  tmpreg&=0XFFFFFF00; 
 	tmpreg|=clkdiv;   
	SDIO->CLKCR=tmpreg;
}

/*******************************************************************************
* Function Name  : SD_Init
* Description    : Initializes the SD Card and put it into StandBy State (Ready 
*                  for data transfer).
* Input          : None
* Output         : None
* Return         : SD_Error: SD Card Error code.
*******************************************************************************/
SD_Error SD_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	unsigned char clkdiv=0;
  SD_Error errorstatus = SD_OK;

	NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;			//SDIO中断配置
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//抢占优先级0 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;					//子优先级0 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
	
  /* Configure SDIO interface GPIO */
  SDCard_IO_Configuration();//配置SDIO接口使用的GPIO管脚
	//log_info("SDCard_IO_Configuration OK!\r\n");
	
  /* Enable the SDIO and DMA2 Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_SDIO | RCC_AHBPeriph_DMA2, ENABLE); //使能SDIO接口的AHB时钟//使能DMA2时钟

  SDIO_DeInit(); //配置SDIO接口，恢复初始设置
	
	//SD_PowerON和SD_InitializeCards完成卡检测、识别流程
  errorstatus = SD_PowerON();
		if (errorstatus != SD_OK)return(errorstatus); /* CMD Response TimeOut (wait for CMDSENT flag) */
		
  errorstatus = SD_InitializeCards();/*卡识别成功，进行卡初始化    */
		if (errorstatus != SD_OK)return(errorstatus);/* CMD Response TimeOut (wait for CMDSENT flag) */
		
	errorstatus = SD_GetCardInfo(&SDCardInfo);// Read CSD/CID MSD registers //用来读取csd/cid寄存器
		if (errorstatus != SD_OK)return(errorstatus);
		
		/*!< Configure the SDIO peripheral */
  /*!< SDIO_CK = SDIOCLK / (SDIO_TRANSFER_CLK_DIV + 2) */
	//  /* Configure the SDIO peripheral */
	//	/*上电识别，卡初始化完成后，进入数据传输模式，提高读写速度,最高支持24MHz*/
	//  /* HCLK = 72 MHz, SDIOCLK = 72 MHz, SDIO_CK = HCLK/(2 + 4) = 12 MHz */ 
	if((errorstatus==SD_OK)||(SDIO_MULTIMEDIA_CARD==CardType))
	{  		    
		if(SDCardInfo.CardType==SDIO_STD_CAPACITY_SD_CARD_V1_1||SDCardInfo.CardType==SDIO_STD_CAPACITY_SD_CARD_V2_0)
		{
			clkdiv=SDIO_TRANSFER_CLK_DIV+6;	//V1.1/V2.0卡，设置最高72/12=6Mhz
		}
		else
		{
			clkdiv=SDIO_TRANSFER_CLK_DIV;	//SDHC等其他卡，设置最高72/6=12Mhz
		}
		log_info("clkdiv:%d\r\n",clkdiv);
	}
	
  SDIO_InitStructure.SDIO_ClockDiv = clkdiv;
  SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
  SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;
  SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
  SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_1b;
  SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
  SDIO_Init(&SDIO_InitStructure);
		
	errorstatus = SD_SelectDeselect( (uint32_t) (SDCardInfo.RCA << 16) );//-Select Card 通过cmd7  ,rca选择要操作的卡
		if (errorstatus != SD_OK)return(errorstatus);
		
	errorstatus = SD_EnableWideBusOperation(SDIO_BusWide_4b);   //开启4bits模式
		if (errorstatus != SD_OK)return(errorstatus);
		
	errorstatus = SD_SetDeviceMode(SD_POLLING_MODE);	//设置为查询模式
		if (errorstatus != SD_OK)return(errorstatus);
	
	log_info("SDIO_CK:%dMHz\r\n",72/(clkdiv+2));
	switch(SDCardInfo.CardType)
	{
		case SDIO_STD_CAPACITY_SD_CARD_V1_1:log_info("Card Type:SDSC V1.1\r\n");break;
		case SDIO_STD_CAPACITY_SD_CARD_V2_0:log_info("Card Type:SDSC V2.0\r\n");break;
		case SDIO_HIGH_CAPACITY_SD_CARD:log_info("Card Type:SDHC V2.0\r\n");break;
		case SDIO_MULTIMEDIA_CARD:log_info("Card Type:MMC Card\r\n");break;
	}	
  log_info("Card ManufacturerID:%d\r\n",SDCardInfo.SD_cid.ManufacturerID);	//制造商ID
 	log_info("Card RCA:%d\r\n",SDCardInfo.RCA);								//卡相对地址
	log_info("Card Capacity:%d MB\r\n",(SDCardInfo.CardCapacity>>10));	//显示容量
	log_info("Card Capacity:%.3f GB\r\n",((1.0*SDCardInfo.CardCapacity)/(1024*1024)));	//显示容量
 	log_info("Card BlockSize:%d\r\n",SDCardInfo.CardBlockSize);			    //显示块大小
	
	log_info("SD_Card Init OK!\r\n\r\n");			    //显示块大小
  return(errorstatus);
}

/*******************************************************************************
* Function Name  : SD_PowerON
* Description    : Enquires cards about their operating voltage and configures 
*                  clock controls.
* Input          : None
* Output         : None
* Return         : SD_Error: SD Card Error code.
*******************************************************************************/
SD_Error SD_PowerON(void)
{
	//u8 i=0;
  SD_Error errorstatus = SD_OK;
  uint32_t response = 0, count = 0,validvoltage = 0;
  uint32_t SDType = SD_STD_CAPACITY;

  /* Power ON Sequence *//* Configure the SDIO peripheral *///初始化的时钟不能大于400KHz
  SDIO_InitStructure.SDIO_ClockDiv = SDIO_INIT_CLK_DIV; /* HCLK = 72MHz, SDIOCLK = 72MHz, SDIO_CK = HCLK/(178 + 2) = 400 KHz */
  SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
  SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;//不使用bypass模式，直接用HCLK进行分频得到SDIO_CK
  SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;// 空闲时不关闭时钟电源
  SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_1b;        //1位数据线
  SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;//硬件流
  SDIO_Init(&SDIO_InitStructure);

  SDIO_SetPowerState(SDIO_PowerState_ON);//上电状态，开启卡时钟/* Set Power State to ON */
  SDIO_ClockCmd(ENABLE);/* Enable SDIO Clock */

  /* CMD0: GO_IDLE_STATE -------------------------------------------------------*/
  /* No CMD response required */

	SDIO_CmdInitStructure.SDIO_Argument = 0x0;
	SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_GO_IDLE_STATE;  //cmd0
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_No; //无响应
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable; //则CPSM在开始发送命令之前等待数据传输结束
	SDIO_SendCommand(&SDIO_CmdInitStructure);    //写命令进命令寄存器

	errorstatus = CmdError();   //检测是否正确接收到cmd0
	log_info("errorstatus1:%d\r\n",errorstatus);	
  if (errorstatus != SD_OK) return(errorstatus);/* CMD Response TimeOut (wait for CMDSENT flag) *///命令发送出错，返回
  

  /* CMD8: SEND_IF_COND --------------------------------------------------------*/
  /* Send CMD8 to verify SD card interface operating condition */
  /* Argument: - [31:12]: Reserved (shall be set to '0')
               - [11:8]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
               - [7:0]: Check Pattern (recommended 0xAA) */
  /* CMD Response: R7 */
  SDIO_CmdInitStructure.SDIO_Argument = SD_CHECK_PATTERN;    //接收到命令sd会返回这个参数
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SEND_IF_COND;   //cmd8
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;   //r7
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;  //关闭等待中断
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable; 
  SDIO_SendCommand(&SDIO_CmdInitStructure);
	
  errorstatus = CmdResp7Error();/*检查是否接收到命令*/
  log_info("errorstatus2:%d\r\n",errorstatus);
  if (errorstatus == SD_OK) //有响应则card遵循sd协议2.0版本
  {
    CardType = SDIO_STD_CAPACITY_SD_CARD_V2_0; /* SD Card 2.0 *//*!< SD Card 2.0 ，先把它定义会sdsc类型的卡*/
    SDType = SD_HIGH_CAPACITY; //这个变量用作acmd41的参数，用来询问是sdsc卡还是sdhc卡
  }
	else
  {
    /*!< CMD55 */
    SDIO_CmdInitStructure.SDIO_Argument = 0x00;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_APP_CMD;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);
    errorstatus = CmdResp1Error(SDIO_APP_CMD);
  }
  /* CMD55 *//*!< CMD55 */     //为什么在else里和else外面都要发送CMD55?
	//发送cmd55，用于检测是sd卡还是mmc卡，或是不支持的卡
  SDIO_CmdInitStructure.SDIO_Argument = 0x00;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_APP_CMD;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short; //r1
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);
	
  errorstatus = CmdResp1Error(SDIO_APP_CMD); //是否响应，没响应的是mmc或不支持的卡
 log_info("errorstatus3:%d\r\n",errorstatus);
  /* If errorstatus is Command TimeOut, it is a MMC card */
  /* If errorstatus is SD_OK it is a SD card: SD card 2.0 (voltage range mismatch)
     or SD card 1.x */
  if (errorstatus == SD_OK)
  {
		/*下面开始循环地发送sdio支持的电压范围，循环一定次数*/
    /* SD CARD */
    /* Send ACMD41 SD_APP_OP_COND with Argument 0x80100000 */
    while ((!validvoltage) && (count < SD_MAX_VOLT_TRIAL))
    {
			//因为下面要用到ACMD41，是ACMD命令，在发送ACMD命令前都要先向卡发送CMD55
      /* SEND CMD55 APP_CMD with RCA as 0 */
      SDIO_CmdInitStructure.SDIO_Argument = 0x00;
      SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_APP_CMD; //CMD55
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(SDIO_APP_CMD); //检测响应

      if (errorstatus != SD_OK)return(errorstatus);  //没响应CMD55，返回
      
			//acmd41，命令参数由支持的电压范围及HCS位组成，HCS位置一来区分卡是SDSc还是sdhc
      SDIO_CmdInitStructure.SDIO_Argument = SD_VOLTAGE_WINDOW_SD | SDType;//参数为主机可供电压范围及hcs位
      SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SD_APP_OP_COND;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp3Error();//等待R3响应，检测是否正确接收到数据
      if (errorstatus != SD_OK)return(errorstatus);//没正确接收到acmd41，出错，返回
      
			/*若卡需求电压在SDIO的供电电压范围内，会自动上电并标志pwr_up位*/
      response = SDIO_GetResponse(SDIO_RESP1);//读取卡寄存器，卡状态
      validvoltage = (((response >> 31) == 1) ? 1 : 0); //读取卡的ocr寄存器的pwr_up位，看是否已工作在正常电压
      count++;//计算循环次数
    }
    if (count >= SD_MAX_VOLT_TRIAL)  //循环检测超过一定次数还没上电
    {
      errorstatus = SD_INVALID_VOLTRANGE;   //SDIO不支持card的供电电压
      return(errorstatus);
    }
		/*检查卡返回信息中的HCS位*/
    if (response &= SD_HIGH_CAPACITY) //判断ocr中的ccs位 ，如果是sdsc卡则不执行下面的语句
    {
      CardType = SDIO_HIGH_CAPACITY_SD_CARD; //把卡类型从初始化的sdsc型改为sdhc型
    }
  }
	/* else MMC Card */
	else
	{
		//MMC卡,发送CMD1 SDIO_SEND_OP_COND,参数为:0x80FF8000 
		while((!validvoltage)&&(count<SD_MAX_VOLT_TRIAL))
		{	   										   				   
			SDIO_CmdInitStructure.SDIO_Argument = SD_VOLTAGE_WINDOW_MMC;//发送CMD1,短响应	   
      SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SEND_OP_COND;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;  //r3
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);
			
			errorstatus=CmdResp3Error(); 					//等待R3响应   
 			if(errorstatus!=SD_OK)return errorstatus;   	//响应错误  
			response=SDIO->RESP1;;			   				//得到响应
			validvoltage=(((response>>31)==1)?1:0);
			count++;
		}
		if(count>=SD_MAX_VOLT_TRIAL)
		{
			errorstatus=SD_INVALID_VOLTRANGE;
			return errorstatus;
		}	 			    
		CardType=SDIO_MULTIMEDIA_CARD;	  
  	}  
  return(errorstatus);
}

/*******************************************************************************
* Function Name  : SD_PowerOFF
* Description    : Turns the SDIO output signals off.
* Input          : None
* Output         : None
* Return         : SD_Error: SD Card Error code.
*******************************************************************************/
SD_Error SD_PowerOFF(void)
{
  SD_Error errorstatus = SD_OK;

  /* Set Power State to OFF */
  SDIO_SetPowerState(SDIO_PowerState_OFF);

  return(errorstatus);
}

/*******************************************************************************
* Function Name  : SD_InitializeCards
* Description    : Intialises all cards or single card as the case may be. 
*                  Card(s) come into standby state.
* Input          : None
* Output         : None
* Return         : SD_Error: SD Card Error code.
*******************************************************************************/
SD_Error SD_InitializeCards(void)
{
  SD_Error errorstatus = SD_OK;
  uint16_t rca = 0x01;

  if (SDIO_GetPowerState() == SDIO_PowerState_OFF)
  {
    errorstatus = SD_REQUEST_NOT_APPLICABLE;//检查电源状态,确保为上电状态
    return(errorstatus);
  }

  if (SDIO_SECURE_DIGITAL_IO_CARD != CardType)//判断卡的类型
  {
    /* Send CMD2 ALL_SEND_CID */
    SDIO_CmdInitStructure.SDIO_Argument = 0x0;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_ALL_SEND_CID;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Long;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp2Error();

    if (SD_OK != errorstatus)
    {
      return(errorstatus);
    }

    CID_Tab[0] = SDIO_GetResponse(SDIO_RESP1);
    CID_Tab[1] = SDIO_GetResponse(SDIO_RESP2);
    CID_Tab[2] = SDIO_GetResponse(SDIO_RESP3);
    CID_Tab[3] = SDIO_GetResponse(SDIO_RESP4);
  }
	/*下面开始SD卡初始化流程*/
  if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) ||  (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) ||  (SDIO_SECURE_DIGITAL_IO_COMBO_CARD == CardType)
      ||  (SDIO_HIGH_CAPACITY_SD_CARD == CardType)) //使用的是2.0的卡
  {
    /* Send CMD3 SET_REL_ADDR with argument 0 */
    /* SD Card publishes its RCA. */
    SDIO_CmdInitStructure.SDIO_Argument = 0x00;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SET_REL_ADDR;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp6Error(SDIO_SET_REL_ADDR, &rca);//把接收到的卡相对地址存起来

    if (SD_OK != errorstatus)
    {
      return(errorstatus);
    }
  }
	if (SDIO_MULTIMEDIA_CARD==CardType)
  {
		SDIO_CmdInitStructure.SDIO_Argument = (u32)(rca<<16);//发送CMD3,短响应 
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SET_REL_ADDR;	//cmd3
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short; //r6
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);	//发送CMD3,短响应 	
			
		errorstatus=CmdResp2Error(); 					//等待R2响应   
		if(errorstatus!=SD_OK)return errorstatus;   	//响应错误	 
  }

  if (SDIO_SECURE_DIGITAL_IO_CARD != CardType)
  {
    RCA = rca;

    /* Send CMD9 SEND_CSD with argument as card's RCA */
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)(rca << 16);
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SEND_CSD;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Long;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp2Error();

    if (SD_OK != errorstatus)
    {
      return(errorstatus);
    }

    CSD_Tab[0] = SDIO_GetResponse(SDIO_RESP1);
    CSD_Tab[1] = SDIO_GetResponse(SDIO_RESP2);
    CSD_Tab[2] = SDIO_GetResponse(SDIO_RESP3);
    CSD_Tab[3] = SDIO_GetResponse(SDIO_RESP4);
  }

  errorstatus = SD_OK; /* All cards get intialized */

  return(errorstatus);
}

/*******************************************************************************
* Function Name  : SD_GetCardInfo
* Description    : Returns information about specific card.
* Input          : cardinfo : pointer to a SD_CardInfo structure 
*                  that contains all SD card information.
* Output         : None
* Return         : SD_Error: SD Card Error code.
*******************************************************************************/
SD_Error SD_GetCardInfo(SDCardInfo_Type *cardinfo)
{
  SD_Error errorstatus = SD_OK;
  uint8_t tmp = 0;

  cardinfo->CardType = (uint8_t)CardType;
  cardinfo->RCA = (uint16_t)RCA;

  /* Byte 0 */
  tmp = (uint8_t)((CSD_Tab[0] & 0xFF000000) >> 24);
  cardinfo->SD_csd.CSDStruct = (tmp & 0xC0) >> 6;
  cardinfo->SD_csd.SysSpecVersion = (tmp & 0x3C) >> 2;
  cardinfo->SD_csd.Reserved1 = tmp & 0x03;

  /* Byte 1 */
  tmp = (uint8_t)((CSD_Tab[0] & 0x00FF0000) >> 16);
  cardinfo->SD_csd.TAAC = tmp;

  /* Byte 2 */
  tmp = (uint8_t)((CSD_Tab[0] & 0x0000FF00) >> 8);
  cardinfo->SD_csd.NSAC = tmp;

  /* Byte 3 */
  tmp = (uint8_t)(CSD_Tab[0] & 0x000000FF);
  cardinfo->SD_csd.MaxBusClkFrec = tmp;

  /* Byte 4 */
  tmp = (uint8_t)((CSD_Tab[1] & 0xFF000000) >> 24);
  cardinfo->SD_csd.CardComdClasses = tmp << 4;

  /* Byte 5 */
  tmp = (uint8_t)((CSD_Tab[1] & 0x00FF0000) >> 16);
  cardinfo->SD_csd.CardComdClasses |= (tmp & 0xF0) >> 4;
  cardinfo->SD_csd.RdBlockLen = tmp & 0x0F;

  /* Byte 6 */
  tmp = (uint8_t)((CSD_Tab[1] & 0x0000FF00) >> 8);
  cardinfo->SD_csd.PartBlockRead = (tmp & 0x80) >> 7;
  cardinfo->SD_csd.WrBlockMisalign = (tmp & 0x40) >> 6;
  cardinfo->SD_csd.RdBlockMisalign = (tmp & 0x20) >> 5;
  cardinfo->SD_csd.DSRImpl = (tmp & 0x10) >> 4;
  cardinfo->SD_csd.Reserved2 = 0; /* Reserved */

  if ((CardType == SDIO_STD_CAPACITY_SD_CARD_V1_1) || (CardType == SDIO_STD_CAPACITY_SD_CARD_V2_0))
  {
    cardinfo->SD_csd.DeviceSize = (tmp & 0x03) << 10;

    /* Byte 7 */
    tmp = (uint8_t)(CSD_Tab[1] & 0x000000FF);
    cardinfo->SD_csd.DeviceSize |= (tmp) << 2;

    /* Byte 8 */
    tmp = (uint8_t)((CSD_Tab[2] & 0xFF000000) >> 24);
    cardinfo->SD_csd.DeviceSize |= (tmp & 0xC0) >> 6;

    cardinfo->SD_csd.MaxRdCurrentVDDMin = (tmp & 0x38) >> 3;
    cardinfo->SD_csd.MaxRdCurrentVDDMax = (tmp & 0x07);

    /* Byte 9 */
    tmp = (uint8_t)((CSD_Tab[2] & 0x00FF0000) >> 16);
    cardinfo->SD_csd.MaxWrCurrentVDDMin = (tmp & 0xE0) >> 5;
    cardinfo->SD_csd.MaxWrCurrentVDDMax = (tmp & 0x1C) >> 2;
    cardinfo->SD_csd.DeviceSizeMul = (tmp & 0x03) << 1;
    /* Byte 10 */
    tmp = (uint8_t)((CSD_Tab[2] & 0x0000FF00) >> 8);
    cardinfo->SD_csd.DeviceSizeMul |= (tmp & 0x80) >> 7;
    
    cardinfo->CardCapacity = (cardinfo->SD_csd.DeviceSize + 1) ;
    cardinfo->CardCapacity *= (1 << (cardinfo->SD_csd.DeviceSizeMul + 2));
    cardinfo->CardBlockSize = 1 << (cardinfo->SD_csd.RdBlockLen);
    cardinfo->CardCapacity *= cardinfo->CardBlockSize;
  }
  else if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
  {
    /* Byte 7 */
    tmp = (uint8_t)(CSD_Tab[1] & 0x000000FF);
    cardinfo->SD_csd.DeviceSize = (tmp & 0x3F) << 16;

    /* Byte 8 */
    tmp = (uint8_t)((CSD_Tab[2] & 0xFF000000) >> 24);

    cardinfo->SD_csd.DeviceSize |= (tmp << 8);

    /* Byte 9 */
    tmp = (uint8_t)((CSD_Tab[2] & 0x00FF0000) >> 16);

    cardinfo->SD_csd.DeviceSize |= (tmp);
		//printf("cardinfo->SD_csd.DeviceSize:%d\r\n",cardinfo->SD_csd.DeviceSize);

    /* Byte 10 */
    tmp = (uint8_t)((CSD_Tab[2] & 0x0000FF00) >> 8);
    
    cardinfo->CardCapacity = (uint64_t)(cardinfo->SD_csd.DeviceSize + 1) * 512    ;
		//printf("cardinfo->CardCapacity:%d\r\n",cardinfo->CardCapacity);
    cardinfo->CardBlockSize = 512;    
  }


  cardinfo->SD_csd.EraseGrSize = (tmp & 0x40) >> 6;
  cardinfo->SD_csd.EraseGrMul = (tmp & 0x3F) << 1;

  /* Byte 11 */
  tmp = (uint8_t)(CSD_Tab[2] & 0x000000FF);
  cardinfo->SD_csd.EraseGrMul |= (tmp & 0x80) >> 7;
  cardinfo->SD_csd.WrProtectGrSize = (tmp & 0x7F);

  /* Byte 12 */
  tmp = (uint8_t)((CSD_Tab[3] & 0xFF000000) >> 24);
  cardinfo->SD_csd.WrProtectGrEnable = (tmp & 0x80) >> 7;
  cardinfo->SD_csd.ManDeflECC = (tmp & 0x60) >> 5;
  cardinfo->SD_csd.WrSpeedFact = (tmp & 0x1C) >> 2;
  cardinfo->SD_csd.MaxWrBlockLen = (tmp & 0x03) << 2;

  /* Byte 13 */
  tmp = (uint8_t)((CSD_Tab[3] & 0x00FF0000) >> 16);
  cardinfo->SD_csd.MaxWrBlockLen |= (tmp & 0xC0) >> 6;
  cardinfo->SD_csd.WriteBlockPaPartial = (tmp & 0x20) >> 5;
  cardinfo->SD_csd.Reserved3 = 0;
  cardinfo->SD_csd.ContentProtectAppli = (tmp & 0x01);

  /* Byte 14 */
  tmp = (uint8_t)((CSD_Tab[3] & 0x0000FF00) >> 8);
  cardinfo->SD_csd.FileFormatGrouop = (tmp & 0x80) >> 7;
  cardinfo->SD_csd.CopyFlag = (tmp & 0x40) >> 6;
  cardinfo->SD_csd.PermWrProtect = (tmp & 0x20) >> 5;
  cardinfo->SD_csd.TempWrProtect = (tmp & 0x10) >> 4;
  cardinfo->SD_csd.FileFormat = (tmp & 0x0C) >> 2;
  cardinfo->SD_csd.ECC = (tmp & 0x03);

  /* Byte 15 */
  tmp = (uint8_t)(CSD_Tab[3] & 0x000000FF);
  cardinfo->SD_csd.CSD_CRC = (tmp & 0xFE) >> 1;
  cardinfo->SD_csd.Reserved4 = 1;

///以下为CID寄存器信息

  /* Byte 0 */
  tmp = (uint8_t)((CID_Tab[0] & 0xFF000000) >> 24);
	cardinfo->SD_cid.ManufacturerID = tmp;
  //cardinfo->SD_cid.ManufacturerID = CID_Tab[0];
	//printf("cardinfo->SD_cid.ManufacturerID:0x%x\r\n",cardinfo->SD_cid.ManufacturerID);

  /* Byte 1 */
  tmp = (uint8_t)((CID_Tab[0] & 0x00FF0000) >> 16);
  cardinfo->SD_cid.OEM_AppliID = tmp << 8;

  /* Byte 2 */
  tmp = (uint8_t)((CID_Tab[0] & 0x000000FF00) >> 8);
  cardinfo->SD_cid.OEM_AppliID |= tmp;

  /* Byte 3 */
  tmp = (uint8_t)(CID_Tab[0] & 0x000000FF);
  cardinfo->SD_cid.ProdName1 = tmp << 24;

  /* Byte 4 */
  tmp = (uint8_t)((CID_Tab[1] & 0xFF000000) >> 24);
  cardinfo->SD_cid.ProdName1 |= tmp << 16;

  /* Byte 5 */
  tmp = (uint8_t)((CID_Tab[1] & 0x00FF0000) >> 16);
  cardinfo->SD_cid.ProdName1 |= tmp << 8;

  /* Byte 6 */
  tmp = (uint8_t)((CID_Tab[1] & 0x0000FF00) >> 8);
  cardinfo->SD_cid.ProdName1 |= tmp;

  /* Byte 7 */
  tmp = (uint8_t)(CID_Tab[1] & 0x000000FF);
  cardinfo->SD_cid.ProdName2 = tmp;

  /* Byte 8 */
  tmp = (uint8_t)((CID_Tab[2] & 0xFF000000) >> 24);
  cardinfo->SD_cid.ProdRev = tmp;

  /* Byte 9 */
  tmp = (uint8_t)((CID_Tab[2] & 0x00FF0000) >> 16);
  cardinfo->SD_cid.ProdSN = tmp << 24;

  /* Byte 10 */
  tmp = (uint8_t)((CID_Tab[2] & 0x0000FF00) >> 8);
  cardinfo->SD_cid.ProdSN |= tmp << 16;

  /* Byte 11 */
  tmp = (uint8_t)(CID_Tab[2] & 0x000000FF);
  cardinfo->SD_cid.ProdSN |= tmp << 8;

  /* Byte 12 */
  tmp = (uint8_t)((CID_Tab[3] & 0xFF000000) >> 24);
  cardinfo->SD_cid.ProdSN |= tmp;

  /* Byte 13 */
  tmp = (uint8_t)((CID_Tab[3] & 0x00FF0000) >> 16);
  cardinfo->SD_cid.Reserved1 |= (tmp & 0xF0) >> 4;
  cardinfo->SD_cid.ManufactDate = (tmp & 0x0F) << 8;

  /* Byte 14 */
  tmp = (uint8_t)((CID_Tab[3] & 0x0000FF00) >> 8);
  cardinfo->SD_cid.ManufactDate |= tmp;

  /* Byte 15 */
  tmp = (uint8_t)(CID_Tab[3] & 0x000000FF);
  cardinfo->SD_cid.CID_CRC = (tmp & 0xFE) >> 1;
  cardinfo->SD_cid.Reserved2 = 1;
	
  return(errorstatus);
}

/*******************************************************************************
* Function Name  : SD_EnableWideBusOperation
* Description    : Enables wide bus opeartion for the requeseted card if 
*                  supported by card.
* Input          : WideMode: Specifies the SD card wide bus mode. 
*                     This parameter can be one of the following values:
*                       - SDIO_BusWide_8b: 8-bit data transfer (Only for MMC)
*                       - SDIO_BusWide_4b: 4-bit data transfer
*                       - SDIO_BusWide_1b: 1-bit data transfer
* Output         : None
* Return         : SD_Error: SD Card Error code.
*******************************************************************************/
SD_Error SD_EnableWideBusOperation(uint32_t WideMode)
{
  SD_Error errorstatus = SD_OK;

  /* MMC Card doesn't support this feature */
  if (SDIO_MULTIMEDIA_CARD == CardType)
  {
    errorstatus = SD_UNSUPPORTED_FEATURE;
    return(errorstatus);
  }
  else if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) || (SDIO_HIGH_CAPACITY_SD_CARD == CardType))
  {
    if (SDIO_BusWide_8b == WideMode)
    {
      errorstatus = SD_UNSUPPORTED_FEATURE;
      return(errorstatus);
    }
    else if (SDIO_BusWide_4b == WideMode)
    {
      errorstatus = SDEnWideBus(ENABLE);

      if (SD_OK == errorstatus)
      {
        /* Configure the SDIO peripheral */
        SDIO_InitStructure.SDIO_ClockDiv = SDIO_TRANSFER_CLK_DIV; 
        SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
        SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;
        SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
        SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_4b;
        SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
        SDIO_Init(&SDIO_InitStructure);
      }
    }
    else
    {
      errorstatus = SDEnWideBus(DISABLE);

      if (SD_OK == errorstatus)
      {
        /* Configure the SDIO peripheral */
        SDIO_InitStructure.SDIO_ClockDiv = SDIO_TRANSFER_CLK_DIV; 
        SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
        SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;
        SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
        SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_1b;
        SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
        SDIO_Init(&SDIO_InitStructure);
      }
    }
  }

  return(errorstatus);
}

/*******************************************************************************
* Function Name  : SD_SetDeviceMode
* Description    : Sets device mode whether to operate in Polling, Interrupt or
*                  DMA mode.
* Input          : Mode: Specifies the Data Transfer mode.
*                     This parameter can be one of the following values:
*                       - SD_DMA_MODE: Data transfer using DMA.
*                       - SD_INTERRUPT_MODE: Data transfer using interrupts.
*                       - SD_POLLING_MODE: Data transfer using flags.
* Output         : None
* Return         : SD_Error: SD Card Error code.
*******************************************************************************/
SD_Error SD_SetDeviceMode(uint32_t Mode)
{
  SD_Error errorstatus = SD_OK;

  if ((Mode == SD_DMA_MODE) || (Mode == SD_INTERRUPT_MODE) || (Mode == SD_POLLING_MODE))
  {
    DeviceMode = Mode;
  }
  else
  {
    errorstatus = SD_INVALID_PARAMETER;
  }
  return(errorstatus);

}

/*******************************************************************************
* Function Name  : SD_SelectDeselect
* Description    : Selects od Deselects the corresponding card.
* Input          : addr: Address of the Card to be selected.
* Output         : None
* Return         : SD_Error: SD Card Error code.
*******************************************************************************/
SD_Error SD_SelectDeselect(uint32_t addr)
{
  SD_Error errorstatus = SD_OK;

  /* Send CMD7 SDIO_SEL_DESEL_CARD */
  SDIO_CmdInitStructure.SDIO_Argument =  addr;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SEL_DESEL_CARD;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SDIO_SEL_DESEL_CARD);

  return(errorstatus);
}

/*******************************************************************************
* Function Name  : SD_ReadBlock   块读
* Description    : Allows to read one block from a specified address in a card.
* Input          : - addr: Address from where data are to be read.
*                  - readbuff: pointer to the buffer that will contain the 
*                    received data
*                  - blocksize: the SD card Data block size.
* Output         : None
* Return         : SD_Error: SD Card Error code.
CMD16和CMD17指令
*******************************************************************************/
SD_Error SD_ReadBlock(u8 *buf,long long addr,uint16_t blksize)
{
	//u32 i=0;
	SD_Error errorstatus = SD_OK;
	uint32_t count = 0, *tempbuff = (u32*)buf;
	uint8_t power = 0;
	u32 timeout=SDIO_DATATIMEOUT; 

	if (NULL == buf)
	{
		errorstatus = SD_INVALID_PARAMETER;
		return(errorstatus);
	}
	
	//TransferError = SD_OK;
	//TransferEnd = 0;
	//StopCondition = 0;
	SDIO->DCTRL=0x0;	//数据控制寄存器清零(关DMA) 
	
	if(CardType==SDIO_HIGH_CAPACITY_SD_CARD)//大容量卡
	{
		blksize=512;
		addr /= 512;
	}	
	
	SDIO_DataInitStructure.SDIO_DataBlockSize= SDIO_DataBlockSize_1b ;//清除DPSM状态机配置
	SDIO_DataInitStructure.SDIO_DataLength= 0 ;
	SDIO_DataInitStructure.SDIO_DataTimeOut=SD_DATATIMEOUT ;
	SDIO_DataInitStructure.SDIO_DPSM=SDIO_DPSM_Enable;
	SDIO_DataInitStructure.SDIO_TransferDir=SDIO_TransferDir_ToCard;
	SDIO_DataInitStructure.SDIO_TransferMode=SDIO_TransferMode_Block;
	SDIO_DataConfig(&SDIO_DataInitStructure);
	
	if (SDIO_GetResponse(SDIO_RESP1) & SD_CARD_LOCKED)//卡锁了
	{
		errorstatus = SD_LOCK_UNLOCK_FAILED;
		return(errorstatus);
	}
	
	if ((blksize > 0) && (blksize <= 2048) && ((blksize & (blksize - 1)) == 0))
	{
		power = convert_from_bytes_to_power_of_two(blksize);

		/* Set Block Size for Card */
		SDIO_CmdInitStructure.SDIO_Argument = blksize;
		SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SET_BLOCKLEN;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);//发送CMD16+设置数据长度为blksize,短响应
		
		errorstatus = CmdResp1Error(SDIO_SET_BLOCKLEN);//等待R1响应  
		if (SD_OK != errorstatus)return(errorstatus);
	}
	else
	{
		errorstatus = SD_INVALID_PARAMETER;
		return(errorstatus);
	}
	
	SDIO_DataInitStructure.SDIO_DataBlockSize= power<<4 ;//清除DPSM状态机配置
	SDIO_DataInitStructure.SDIO_DataLength= blksize ;
	SDIO_DataInitStructure.SDIO_DataTimeOut=SD_DATATIMEOUT ;
	SDIO_DataInitStructure.SDIO_DPSM=SDIO_DPSM_Enable;
	SDIO_DataInitStructure.SDIO_TransferDir=SDIO_TransferDir_ToSDIO;
	SDIO_DataInitStructure.SDIO_TransferMode=SDIO_TransferMode_Block;
	SDIO_DataConfig(&SDIO_DataInitStructure);
	
	SDIO_CmdInitStructure.SDIO_Argument =  addr;
	SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_READ_SINGLE_BLOCK;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);//发送CMD17+从addr地址出读取数据,短响应
	
	errorstatus = CmdResp1Error(SDIO_READ_SINGLE_BLOCK);//等待R1响应  

	if (errorstatus != SD_OK)return(errorstatus);
	
	if(DeviceMode==SD_POLLING_MODE)						//查询模式,轮询数据	 
	{
 		INTX_DISABLE();//关闭总中断(POLLING模式,严禁中断打断SDIO读写操作!!!)
		while(!(SDIO->STA&((1<<5)|(1<<1)|(1<<3)|(1<<10)|(1<<9))))//无上溢/CRC/超时/完成(标志)/起始位错误
		{
			if(SDIO_GetFlagStatus(SDIO_FLAG_RXFIFOHF) != RESET)							//接收区半满,表示至少存了8个字
			{
				for(count=0;count<8;count++)			//循环读取数据
				{
					*(tempbuff+count)=SDIO->FIFO;
				}
				tempbuff+=8;	 
				timeout=0X7FFFFF; 	//读数据溢出时间
			}
			else 	//处理超时
			{
				//log_info("errorstatus1:0x%x\r\n",errorstatus);
				if(timeout==0)return SD_DATA_TIMEOUT;
				timeout--;
			}
		} 
		if(SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)		//数据超时错误
		{										   
	 		SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT); 	//清错误标志
			log_info("SDIO_Flag_DTIMEOUT.Errorstatus:0x%x\r\n",errorstatus);
			INTX_ENABLE();
			return SD_DATA_TIMEOUT;
	 	}
		else if(SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)	//数据块CRC错误
		{
	 		SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);  		//清错误标志
			log_info("SDIO_FLAG_DCRCFAIL.Errorstatus:0x%x\r\n",errorstatus);
			INTX_ENABLE();
			return SD_DATA_CRC_FAIL;		   
		}
		else if(SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET) 	//接收fifo上溢错误
		{
	 		SDIO_ClearFlag(SDIO_FLAG_RXOVERR);		//清错误标志
			log_info("SDIO_FLAG_RXOVERR.Errorstatus:0x%x\r\n",errorstatus);
			INTX_ENABLE();
			return SD_RX_OVERRUN;		 
		}
		else if(SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET) 	//接收起始位错误
		{
	 		SDIO_ClearFlag(SDIO_FLAG_STBITERR);//清错误标志
			log_info("SDIO_FLAG_STBITERR.Errorstatus:0x%x\r\n",errorstatus);
			INTX_ENABLE();
			return SD_START_BIT_ERR;		 
		}
		while(SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET)	//FIFO里面,还存在可用数据
		{
			*tempbuff=SDIO_ReadData();	//循环读取数据
			tempbuff++;
		}
		INTX_ENABLE();//开启总中断
		SDIO_ClearFlag(SDIO_STATIC_FLAGS);//清除所有标记
	}
	else if(DeviceMode==SD_DMA_MODE)
	{
		SDIO_DMA_Configuration((u32*)buf,blksize,DMA_DIR_PeripheralSRC); 
		TransferError=SD_OK;
		StopCondition=0;			//单块读,不需要发送停止传输指令
		TransferEnd=0;				//传输结束标置位，在中断服务置1
		SDIO->MASK|=(1<<1)|(1<<3)|(1<<8)|(1<<5)|(1<<9);	//配置需要的中断 
		SDIO_DMACmd(ENABLE);
 		while(((DMA2->ISR&0X2000)==RESET)&&(TransferEnd==0)&&(TransferError==SD_OK)&&timeout)timeout--;//等待传输完成 
		if(timeout==0)return SD_DATA_TIMEOUT;//超时
		if(TransferError!=SD_OK)errorstatus=TransferError;  
    }   
	return errorstatus; 
}

/*******************************************************************************
* Function Name  : SD_ReadMultiBlocks   多块读
* Description    : Allows to read blocks from a specified address  in a card.
* Input          : - addr: Address from where data are to be read.
*                  - readbuff: pointer to the buffer that will contain the 
*                    received data.
*                  - BlockSize: the SD card Data block size.
*                  - NumberOfBlocks: number of blocks to be read.
* Output         : None
* Return         : SD_Error: SD Card Error code.
CMD16和CMD18指令，CMD12结束读取
*******************************************************************************/
SD_Error SD_ReadMultiBlocks( uint32_t *readbuff,uint64_t addr, uint16_t BlockSize, uint32_t NumberOfBlocks)
{
  SD_Error errorstatus = SD_OK;
  uint32_t count = 0, *tempbuff = readbuff;
  uint8_t power = 0;

  if (NULL == readbuff)
  {
    errorstatus = SD_INVALID_PARAMETER;
    return(errorstatus);
  }

  TransferError = SD_OK;
  TransferEnd = 0;
  TotalNumberOfBytes = 0;

  /* Clear all DPSM configuration */
  SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
  SDIO_DataInitStructure.SDIO_DataLength = 0;
  SDIO_DataInitStructure.SDIO_DataBlockSize = SDIO_DataBlockSize_1b;
  SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
  SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
  SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Disable;
  SDIO_DataConfig(&SDIO_DataInitStructure);
  SDIO_DMACmd(DISABLE);

  if (SDIO_GetResponse(SDIO_RESP1) & SD_CARD_LOCKED)
  {
    errorstatus = SD_LOCK_UNLOCK_FAILED;
    return(errorstatus);
  }

  if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
  {
    BlockSize = 512;
    addr /= 512;
  }
  
  if ((BlockSize > 0) && (BlockSize <= 2048) && (0 == (BlockSize & (BlockSize - 1))))
  {
    power = convert_from_bytes_to_power_of_two(BlockSize);

    /* Set Block Size for Card */
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) BlockSize;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SET_BLOCKLEN;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SDIO_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
    {
      return(errorstatus);
    }
  }
  else
  {
    errorstatus = SD_INVALID_PARAMETER;
    return(errorstatus);
  }

  if (NumberOfBlocks > 1)
  {
    /* Common to all modes */
    if (NumberOfBlocks * BlockSize > SD_MAX_DATA_LENGTH)
    {
      errorstatus = SD_INVALID_PARAMETER;
      return(errorstatus);
    }

    TotalNumberOfBytes = NumberOfBlocks * BlockSize;
    StopCondition = 1;
    DestBuffer = readbuff;

    SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
    SDIO_DataInitStructure.SDIO_DataLength = NumberOfBlocks * BlockSize;
    SDIO_DataInitStructure.SDIO_DataBlockSize = (uint32_t) power << 4;
    SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;
    SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
    SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
    SDIO_DataConfig(&SDIO_DataInitStructure);

    /* Send CMD18 READ_MULT_BLOCK with argument data address */
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)addr;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_READ_MULT_BLOCK;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SDIO_READ_MULT_BLOCK);

    if (errorstatus != SD_OK)
    {
      return(errorstatus);
    }

    if (DeviceMode == SD_POLLING_MODE)
    {
			INTX_DISABLE();
			//关总中断
      /* Polling mode */
      while (!(SDIO->STA &(SDIO_FLAG_RXOVERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DATAEND | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_STBITERR)))
      {
        if (SDIO_GetFlagStatus(SDIO_FLAG_RXFIFOHF) != RESET)
        {
          for (count = 0; count < SD_HALFFIFO; count++)
          {
            *(tempbuff + count) = SDIO_ReadData();
          }
          tempbuff += SD_HALFFIFO;
        }
      }

      if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)
      {
        SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
				INTX_ENABLE();
        return(errorstatus);
      }
      else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)
      {
        SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
        errorstatus = SD_DATA_CRC_FAIL;
				INTX_ENABLE();
        return(errorstatus);
      }
      else if (SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET)
      {
        SDIO_ClearFlag(SDIO_FLAG_RXOVERR);
        errorstatus = SD_RX_OVERRUN;
				INTX_ENABLE();
        return(errorstatus);
      }
      else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET)
      {
        SDIO_ClearFlag(SDIO_FLAG_STBITERR);
        errorstatus = SD_START_BIT_ERR;
				INTX_ENABLE();
        return(errorstatus);
      }
      while (SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET)
      {
        *tempbuff = SDIO_ReadData();
        tempbuff++;
      }

      if (SDIO_GetFlagStatus(SDIO_FLAG_DATAEND) != RESET)
      {
        /* In Case Of SD-CARD Send Command STOP_TRANSMISSION */
        if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) || (SDIO_HIGH_CAPACITY_SD_CARD == CardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType))
        {
          /* Send CMD12 STOP_TRANSMISSION */
          SDIO_CmdInitStructure.SDIO_Argument = 0x0;
          SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_STOP_TRANSMISSION;
          SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
          SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
          SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
          SDIO_SendCommand(&SDIO_CmdInitStructure);

          errorstatus = CmdResp1Error(SDIO_STOP_TRANSMISSION);

          if (errorstatus != SD_OK)
          {
						INTX_ENABLE();
            return(errorstatus);
          }
        }
      }
			INTX_ENABLE();
			//开总中断
      /* Clear all the static flags */
      SDIO_ClearFlag(SDIO_STATIC_FLAGS);
    }
    else if (DeviceMode == SD_INTERRUPT_MODE)
    {
      SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND | SDIO_IT_RXOVERR | SDIO_IT_RXFIFOHF | SDIO_IT_STBITERR, ENABLE);
      while ((TransferEnd == 0) && (TransferError == SD_OK))
      {}
      if (TransferError != SD_OK)
      {
        return(TransferError);
      }
    }
    else if (DeviceMode == SD_DMA_MODE)
    {
      SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND | SDIO_IT_RXOVERR | SDIO_IT_STBITERR, ENABLE);
      SDIO_DMACmd(ENABLE);
      DMA_RxConfiguration(readbuff, (NumberOfBlocks * BlockSize));
      while (DMA_GetFlagStatus(DMA2_FLAG_TC4) == RESET)
      {}
      while ((TransferEnd == 0) && (TransferError == SD_OK))
      {}
      if (TransferError != SD_OK)
      {
        return(TransferError);
      }
    }
  }
  return(errorstatus);
}

/*******************************************************************************
* Function Name  : SD_WriteBlock   写一个块
* Description    : Allows to write one block starting from a specified address 
*                  in a card.
* Input          : - addr: Address from where data are to be read.
*                  - buf: pointer to the buffer that contain the data to be
*                    transferred.
*                  - BlockSize: the SD card Data block size.
* Output         : None
* Return         : SD_Error: SD Card Error code.
CMD16-CMD13（查询卡状态）-CMD24-写入数据块
*******************************************************************************/
SD_Error SD_WriteBlock(u8 *buf,long long addr,  u16 blksize)
{
	SD_Error errorstatus = SD_OK;
	
	u8  power=0,cardstate=0;
	
	u32 timeout=0,bytestransferred=0;
	
	u32 cardstatus=0,count=0,restwords=0;
	
	u32	tlen=blksize;						//总长度(字节)
	
	u32*tempbuff=(u32*)buf;								 
 	
	if(buf==NULL)return SD_INVALID_PARAMETER;//参数错误   
  
	SDIO->DCTRL=0x0;							//数据控制寄存器清零(关DMA)   
  
	SDIO_DataInitStructure.SDIO_DataBlockSize= 0; ;//清除DPSM状态机配置
	SDIO_DataInitStructure.SDIO_DataLength= 0 ;
	SDIO_DataInitStructure.SDIO_DataTimeOut=SD_DATATIMEOUT ;
	SDIO_DataInitStructure.SDIO_DPSM=SDIO_DPSM_Enable;
	SDIO_DataInitStructure.SDIO_TransferDir=SDIO_TransferDir_ToCard;
	SDIO_DataInitStructure.SDIO_TransferMode=SDIO_TransferMode_Block;
  SDIO_DataConfig(&SDIO_DataInitStructure);
	
	
	if(SDIO->RESP1&SD_CARD_LOCKED)return SD_LOCK_UNLOCK_FAILED;//卡锁了
 	if(CardType==SDIO_HIGH_CAPACITY_SD_CARD)	//大容量卡
	{
		blksize=512;
		addr>>=9;
	}    
	if((blksize>0)&&(blksize<=2048)&&((blksize&(blksize-1))==0))
	{
		power=convert_from_bytes_to_power_of_two(blksize);	

		SDIO_CmdInitStructure.SDIO_Argument = blksize;//发送CMD16+设置数据长度为blksize,短响应 	
		SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SET_BLOCKLEN;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);	
		
		errorstatus=CmdResp1Error(SDIO_SET_BLOCKLEN);	//等待R1响应 
		
		if(errorstatus!=SD_OK)return errorstatus;   	//响应错误	 
		
	}else return SD_INVALID_PARAMETER;	 
 
	SDIO_CmdInitStructure.SDIO_Argument = (u32)RCA<<16;//发送CMD13,查询卡的状态,短响应 	
	SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SEND_STATUS;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);	

	errorstatus=CmdResp1Error(SDIO_SEND_STATUS);		//等待R1响应 
	
	if(errorstatus!=SD_OK)return errorstatus;
	cardstatus=SDIO->RESP1;													  
	timeout=SD_DATATIMEOUT;
   	while(((cardstatus&0x00000100)==0)&&(timeout>0)) 	//检查READY_FOR_DATA位是否置位
	{
		timeout--;

		SDIO_CmdInitStructure.SDIO_Argument = (u32)RCA<<16;//发送CMD13,查询卡的状态,短响应
		SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SEND_STATUS;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);	
		
		errorstatus=CmdResp1Error(SDIO_SEND_STATUS);	//等待R1响应   		   
		if(errorstatus!=SD_OK)return errorstatus;				    
		cardstatus=SDIO->RESP1;													  
	}
	if(timeout==0)return SD_ERROR;
	
		SDIO_CmdInitStructure.SDIO_Argument = addr;//发送CMD24,写单块指令,短响应 	
		SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_WRITE_SINGLE_BLOCK;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);	
	
	errorstatus=CmdResp1Error(SDIO_WRITE_SINGLE_BLOCK);//等待R1响应   		   
	if(errorstatus!=SD_OK)return errorstatus;   	  
	StopCondition=0;									//单块写,不需要发送停止传输指令 

	SDIO_DataInitStructure.SDIO_DataBlockSize= power<<4; ;	//blksize, 控制器到卡	
	SDIO_DataInitStructure.SDIO_DataLength= blksize ;
	SDIO_DataInitStructure.SDIO_DataTimeOut=SD_DATATIMEOUT ;
	SDIO_DataInitStructure.SDIO_DPSM=SDIO_DPSM_Enable;
	SDIO_DataInitStructure.SDIO_TransferDir=SDIO_TransferDir_ToCard;
	SDIO_DataInitStructure.SDIO_TransferMode=SDIO_TransferMode_Block;
	SDIO_DataConfig(&SDIO_DataInitStructure);
	  
	timeout=SDIO_DATATIMEOUT;
	if (DeviceMode == SD_POLLING_MODE)
	{
		INTX_DISABLE();//关闭总中断(POLLING模式,严禁中断打断SDIO读写操作!!!)
		while(!(SDIO->STA&((1<<10)|(1<<4)|(1<<1)|(1<<3)|(1<<9))))//数据块发送成功/下溢/CRC/超时/起始位错误
		{
			if(SDIO_GetFlagStatus(SDIO_FLAG_TXFIFOHE) != RESET)							//发送区半空,表示至少存了8个字
			{
				if((tlen-bytestransferred)<SD_HALFFIFOBYTES)//不够32字节了
				{
					restwords=((tlen-bytestransferred)%4==0)?((tlen-bytestransferred)/4):((tlen-bytestransferred)/4+1);
					
					for(count=0;count<restwords;count++,tempbuff++,bytestransferred+=4)
					{
						SDIO_WriteData(*tempbuff);
					}
				}else
				{
					for(count=0;count<8;count++)
					{
						SDIO_WriteData(*(tempbuff+count));
					}
					tempbuff+=8;
					bytestransferred+=32;
				}
				timeout=0X3FFFFFFF;	//写数据溢出时间
			}else
			{
				if(timeout==0)return SD_DATA_TIMEOUT;
				timeout--;
			}
		} 
		if(SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)		//数据超时错误
		{										   
	 		SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT); 	//清错误标志
			INTX_ENABLE();//开启总中断
			return SD_DATA_TIMEOUT;
	 	}else if(SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)	//数据块CRC错误
		{
	 		SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);  		//清错误标志
			INTX_ENABLE();//开启总中断
			return SD_DATA_CRC_FAIL;		   
		}else if(SDIO_GetFlagStatus(SDIO_FLAG_TXUNDERR) != RESET) 	//接收fifo下溢错误
		{
	 		SDIO_ClearFlag(SDIO_FLAG_TXUNDERR);		//清错误标志
			INTX_ENABLE();//开启总中断
			return SD_TX_UNDERRUN;		 
		}else if(SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET) 	//接收起始位错误
		{
	 		SDIO_ClearFlag(SDIO_FLAG_STBITERR);//清错误标志
			INTX_ENABLE();//开启总中断
			return SD_START_BIT_ERR;		 
		}   
	      
		INTX_ENABLE();//开启总中断
		SDIO->ICR=0X5FF;	 		//清除所有标记	  
	}else if(DeviceMode==SD_DMA_MODE)
	{
		SDIO_DMA_Configuration((u32*)buf,blksize,DMA_DIR_PeripheralDST);//SDIO DMA配置
   	TransferError=SD_OK;
		StopCondition=0;			//单块写,不需要发送停止传输指令 
		TransferEnd=0;				//传输结束标置位，在中断服务置1
		SDIO->MASK|=(1<<1)|(1<<3)|(1<<8)|(1<<4)|(1<<9);	//配置产生数据接收完成中断
 	 	SDIO->DCTRL|=1<<3;								//SDIO DMA使能.  
 		while(((DMA2->ISR&0X2000)==RESET)&&timeout)timeout--;//等待传输完成 
		if(timeout==0)
		{
  			SD_Init();	 					//重新初始化SD卡,可以解决写入死机的问题
			return SD_DATA_TIMEOUT;			//超时	 
 		}
		timeout=SDIO_DATATIMEOUT;
		while((TransferEnd==0)&&(TransferError==SD_OK)&&timeout)timeout--;
 		if(timeout==0)return SD_DATA_TIMEOUT;			//超时	 
  		if(TransferError!=SD_OK)return TransferError;
 	}  
 	SDIO_ClearFlag(SDIO_STATIC_FLAGS);//清除所有标记
 	errorstatus=IsCardProgramming(&cardstate);
 	while((errorstatus==SD_OK)&&((cardstate==SD_CARD_PROGRAMMING)||(cardstate==SD_CARD_RECEIVING)))
	{
		errorstatus=IsCardProgramming(&cardstate);
	}   
	return errorstatus;
}

/*******************************************************************************
* Function Name  : SD_WriteMultiBlocks    写多个块
* Description    : Allows to write blocks starting from a specified address in 
*                  a card.
* Input          : - addr: Address from where data are to be read.
*                  - writebuff: pointer to the buffer that contain the data to be
*                    transferred.
*                  - BlockSize: the SD card Data block size.
*                  - NumberOfBlocks: number of blocks to be written.
* Output         : None
* Return         : SD_Error: SD Card Error code.
*描述  ：从输入的起始地址开始，向卡写入多个数据块，
只能在DMA模式下使用这个函数
注意：调用这个函数后一定要调用
SD_WaitWriteOperation（）来等待DMA传输结束
和   SD_GetStatus() 检测卡与SDIO的FIFO间是否已经完成传输
CMD16-CMD55-ACMD23-CMD25-写入多个数据块-CMD12-CMD13-等待写入完成
*******************************************************************************/
SD_Error SD_WriteMultiBlocks(uint32_t *writebuff, uint64_t addr, uint16_t BlockSize, uint32_t NumberOfBlocks)
{
  SD_Error errorstatus = SD_OK;
  uint8_t  power = 0, cardstate = 0;
  uint32_t bytestransferred = 0;
  uint32_t count = 0, restwords = 0;
  uint32_t *tempbuff = writebuff;

  if (writebuff == NULL)
  {
    errorstatus = SD_INVALID_PARAMETER;
    return(errorstatus);
  }

  TransferError = SD_OK;
  TransferEnd = 0;
  TotalNumberOfBytes = 0;

  SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
  SDIO_DataInitStructure.SDIO_DataLength = 0;
  SDIO_DataInitStructure.SDIO_DataBlockSize = SDIO_DataBlockSize_1b;
  SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
  SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
  SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Disable;
  SDIO_DataConfig(&SDIO_DataInitStructure);
  SDIO_DMACmd(DISABLE);

  if (SDIO_GetResponse(SDIO_RESP1) & SD_CARD_LOCKED)
  {
    errorstatus = SD_LOCK_UNLOCK_FAILED;
    return(errorstatus);
  }

  if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
  {
    BlockSize = 512;
    addr /= 512;
  }
  
  /* Set the block size, both on controller and card */
  if ((BlockSize > 0) && (BlockSize <= 2048) && ((BlockSize & (BlockSize - 1)) == 0))
  {
    power = convert_from_bytes_to_power_of_two(BlockSize);

    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) BlockSize;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SET_BLOCKLEN;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SDIO_SET_BLOCKLEN);

    if (errorstatus != SD_OK)
    {
      return(errorstatus);
    }
  }
  else
  {
    errorstatus = SD_INVALID_PARAMETER;
    return(errorstatus);
  }

  /* Wait till card is ready for data Added */
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) (RCA << 16);
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SEND_STATUS;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SDIO_SEND_STATUS);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  if (NumberOfBlocks > 1)
  {
    /* Common to all modes */
    if (NumberOfBlocks * BlockSize > SD_MAX_DATA_LENGTH)
    {
      errorstatus = SD_INVALID_PARAMETER;
      return(errorstatus);
    }

    if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) || (SDIO_HIGH_CAPACITY_SD_CARD == CardType))
    {
      /* To improve performance */
      SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) (RCA << 16);
      SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_APP_CMD;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);


      errorstatus = CmdResp1Error(SDIO_APP_CMD);

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }
      /* To improve performance */
      SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)NumberOfBlocks;
      SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SET_BLOCK_COUNT;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(SDIO_SET_BLOCK_COUNT);

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }
    }

    /* Send CMD25 WRITE_MULT_BLOCK with argument data address */
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)addr;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_WRITE_MULT_BLOCK;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SDIO_WRITE_MULT_BLOCK);

    if (SD_OK != errorstatus)
    {
      return(errorstatus);
    }

    TotalNumberOfBytes = NumberOfBlocks * BlockSize;
    StopCondition = 1;
    SrcBuffer = writebuff;

    SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
    SDIO_DataInitStructure.SDIO_DataLength = NumberOfBlocks * BlockSize;
    SDIO_DataInitStructure.SDIO_DataBlockSize = (uint32_t) power << 4;
    SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
    SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
    SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
    SDIO_DataConfig(&SDIO_DataInitStructure);

    if (DeviceMode == SD_POLLING_MODE)
    {
			//INTX_DISABLE();
      while (!(SDIO->STA & (SDIO_FLAG_TXUNDERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DATAEND | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_STBITERR)))
      {
        if (SDIO_GetFlagStatus(SDIO_FLAG_TXFIFOHE) != RESET)
        {
          if (!((TotalNumberOfBytes - bytestransferred) < SD_HALFFIFOBYTES))
          {
            for (count = 0; count < SD_HALFFIFO; count++)
            {
              SDIO_WriteData(*(tempbuff + count));
            }
            tempbuff += SD_HALFFIFO;
            bytestransferred += SD_HALFFIFOBYTES;
          }
          else
          {
            restwords = ((TotalNumberOfBytes - bytestransferred) % 4 == 0) ? ((TotalNumberOfBytes - bytestransferred) / 4) :
                        ((TotalNumberOfBytes - bytestransferred) / 4 + 1);

            for (count = 0; count < restwords; count++, tempbuff++, bytestransferred += 4)
            {
              SDIO_WriteData(*tempbuff);
            }
          }
        }
      }

      if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)
      {
        SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
        errorstatus = SD_DATA_TIMEOUT;
        return(errorstatus);
      }
      else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)
      {
        SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
        errorstatus = SD_DATA_CRC_FAIL;
        return(errorstatus);
      }
      else if (SDIO_GetFlagStatus(SDIO_FLAG_TXUNDERR) != RESET)
      {
        SDIO_ClearFlag(SDIO_FLAG_TXUNDERR);
        errorstatus = SD_TX_UNDERRUN;
        return(errorstatus);
      }
      else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET)
      {
        SDIO_ClearFlag(SDIO_FLAG_STBITERR);
        errorstatus = SD_START_BIT_ERR;
        return(errorstatus);
      }

      if (SDIO_GetFlagStatus(SDIO_FLAG_DATAEND) != RESET)
      {
       if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) || (SDIO_HIGH_CAPACITY_SD_CARD == CardType))
        {
          /* Send CMD12 STOP_TRANSMISSION */
          SDIO_CmdInitStructure.SDIO_Argument = 0x0;
          SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_STOP_TRANSMISSION;
          SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
          SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
          SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
          SDIO_SendCommand(&SDIO_CmdInitStructure);


          errorstatus = CmdResp1Error(SDIO_STOP_TRANSMISSION);

          if (errorstatus != SD_OK)
          {
            return(errorstatus);
          }
        }
      }
    }
    else if (DeviceMode == SD_INTERRUPT_MODE)
    {
      SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND | SDIO_IT_TXFIFOHE | SDIO_IT_TXUNDERR | SDIO_IT_STBITERR, ENABLE);
      while ((TransferEnd == 0) && (TransferError == SD_OK))
      {}
      if (TransferError != SD_OK)
      {
        return(TransferError);
      }
    }
    else if (DeviceMode == SD_DMA_MODE)
    {
      SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND | SDIO_IT_TXUNDERR | SDIO_IT_STBITERR, ENABLE);
      SDIO_DMACmd(ENABLE);
      DMA_TxConfiguration(writebuff, (NumberOfBlocks * BlockSize));
      while (DMA_GetFlagStatus(DMA2_FLAG_TC4) == RESET)
      {}
      while ((TransferEnd == 0) && (TransferError == SD_OK))
      {}
      if (TransferError != SD_OK)
      {
        return(TransferError);
      }
    }
  }
	//INTX_ENABLE();
  /* Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  /* Wait till the card is in programming state */
  errorstatus = IsCardProgramming(&cardstate);

  while ((errorstatus == SD_OK) && ((cardstate == SD_CARD_PROGRAMMING) || (cardstate == SD_CARD_RECEIVING)))
  {
    errorstatus = IsCardProgramming(&cardstate);
  }

  return(errorstatus);
}

/*******************************************************************************
* Function Name  : SD_GetTransferState
* Description    : Gets the cuurent data transfer state.
* Input          : None
* Output         : None
* Return         : SDTransferState: Data Transfer state.
*                  This value can be: 
*                   - SD_NO_TRANSFER: No data transfer is acting
*                   - SD_TRANSFER_IN_PROGRESS: Data transfer is acting
*******************************************************************************/
SDTransferState SD_GetTransferState(void)
{
  if (SDIO->STA & (SDIO_FLAG_TXACT | SDIO_FLAG_RXACT))
  {
    return(SD_TRANSFER_IN_PROGRESS);
  }
  else
  {
    return(SD_NO_TRANSFER);
  }
}

/*******************************************************************************
* Function Name  : SD_StopTransfer
* Description    : Aborts an ongoing data transfer.
* Input          : None
* Output         : None
* Return         : SD_Error: SD Card Error code.
*******************************************************************************/
SD_Error SD_StopTransfer(void)
{
  SD_Error errorstatus = SD_OK;

  /* Send CMD12 STOP_TRANSMISSION  */
  SDIO_CmdInitStructure.SDIO_Argument = 0x0;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_STOP_TRANSMISSION;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SDIO_STOP_TRANSMISSION);

  return(errorstatus);
}

/*******************************************************************************
* Function Name  : SD_Erase
* Description    : Allows to erase memory area specified for the given card.
* Input          : - startaddr: the start address.
*                  - endaddr: the end address.
* Output         : None
* Return         : SD_Error: SD Card Error code.
*******************************************************************************/
SD_Error SD_Erase(uint32_t startaddr, uint32_t endaddr)
{
  SD_Error errorstatus = SD_OK;
  uint32_t delay = 0;
  __IO uint32_t maxdelay = 0;
  uint8_t cardstate = 0;

  /* Check if the card coomnd class supports erase command */
  if (((CSD_Tab[1] >> 20) & SD_CCCC_ERASE) == 0)
  {
    errorstatus = SD_REQUEST_NOT_APPLICABLE;
    return(errorstatus);
  }

  maxdelay = 72000 / ((SDIO->CLKCR & 0xFF) + 2);

  if (SDIO_GetResponse(SDIO_RESP1) & SD_CARD_LOCKED)
  {
    errorstatus = SD_LOCK_UNLOCK_FAILED;
    return(errorstatus);
  }

  if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
  {
    startaddr /= 512;
    endaddr /= 512;
  }
  
  /* According to sd-card spec 1.0 ERASE_GROUP_START (CMD32) and erase_group_end(CMD33) */
  if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) || (SDIO_HIGH_CAPACITY_SD_CARD == CardType))
  {
    /* Send CMD32 SD_ERASE_GRP_START with argument as addr  */
    SDIO_CmdInitStructure.SDIO_Argument = startaddr;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SD_ERASE_GRP_START;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SDIO_SD_ERASE_GRP_START);
    if (errorstatus != SD_OK)
    {
      return(errorstatus);
    }

    /* Send CMD33 SD_ERASE_GRP_END with argument as addr  */
    SDIO_CmdInitStructure.SDIO_Argument = endaddr;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SD_ERASE_GRP_END;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SDIO_SD_ERASE_GRP_END);
    if (errorstatus != SD_OK)
    {
      return(errorstatus);
    }
  }

  /* Send CMD38 ERASE */
  SDIO_CmdInitStructure.SDIO_Argument = 0;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_ERASE;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SDIO_ERASE);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  for (delay = 0; delay < maxdelay; delay++)
  {}

  /* Wait till the card is in programming state */
  errorstatus = IsCardProgramming(&cardstate);

  while ((errorstatus == SD_OK) && ((SD_CARD_PROGRAMMING == cardstate) || (SD_CARD_RECEIVING == cardstate)))
  {
    errorstatus = IsCardProgramming(&cardstate);
  }

  return(errorstatus);
}

/*******************************************************************************
* Function Name  : SD_SendStatus
* Description    : Returns the current card's status.
* Input          : pcardstatus: pointer to the buffer that will contain the SD 
*                  card status (Card Status register).
* Output         : None
* Return         : SD_Error: SD Card Error code.
*******************************************************************************/
SD_Error SD_SendStatus(uint32_t *pcardstatus)
{
  SD_Error errorstatus = SD_OK;

  if (pcardstatus == NULL)
  {
    errorstatus = SD_INVALID_PARAMETER;
    return(errorstatus);
  }

  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) RCA << 16;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SEND_STATUS;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);


  errorstatus = CmdResp1Error(SDIO_SEND_STATUS);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  *pcardstatus = SDIO_GetResponse(SDIO_RESP1);

  return(errorstatus);
}

/*******************************************************************************
* Function Name  : SD_SendSDStatus
* Description    : Returns the current SD card's status.
* Input          : psdstatus: pointer to the buffer that will contain the SD 
*                  card status (SD Status register).
* Output         : None
* Return         : SD_Error: SD Card Error code.
*******************************************************************************/
SD_Error SD_SendSDStatus(uint32_t *psdstatus)
{
  SD_Error errorstatus = SD_OK;
  uint32_t count = 0;

  if (SDIO_GetResponse(SDIO_RESP1) & SD_CARD_LOCKED)
  {
    errorstatus = SD_LOCK_UNLOCK_FAILED;
    return(errorstatus);
  }

  /* Set block size for card if it is not equal to current block size for card. */
  SDIO_CmdInitStructure.SDIO_Argument = 64;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SET_BLOCKLEN;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SDIO_SET_BLOCKLEN);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  /* CMD55 */
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) RCA << 16;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_APP_CMD;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);
  errorstatus = CmdResp1Error(SDIO_APP_CMD);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
  SDIO_DataInitStructure.SDIO_DataLength = 64;
  SDIO_DataInitStructure.SDIO_DataBlockSize = SDIO_DataBlockSize_64b;
  SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;
  SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
  SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
  SDIO_DataConfig(&SDIO_DataInitStructure);


  /* Send ACMD13 SD_APP_STAUS  with argument as card's RCA.*/
  SDIO_CmdInitStructure.SDIO_Argument = 0;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SD_APP_STAUS;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);
  errorstatus = CmdResp1Error(SDIO_SD_APP_STAUS);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  while (!(SDIO->STA &(SDIO_FLAG_RXOVERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_DBCKEND | SDIO_FLAG_STBITERR)))
  {
    if (SDIO_GetFlagStatus(SDIO_FLAG_RXFIFOHF) != RESET)
    {
      for (count = 0; count < 8; count++)
      {
        *(psdstatus + count) = SDIO_ReadData();
      }
      psdstatus += 8;
    }
  }

  if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
    errorstatus = SD_DATA_TIMEOUT;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
    errorstatus = SD_DATA_CRC_FAIL;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_RXOVERR);
    errorstatus = SD_RX_OVERRUN;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_STBITERR);
    errorstatus = SD_START_BIT_ERR;
    return(errorstatus);
  }

  while (SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET)
  {
    *psdstatus = SDIO_ReadData();
    psdstatus++;
  }

  /* Clear all the static status flags*/
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);
  psdstatus -= 16;
  for (count = 0; count < 16; count++)
  {
    psdstatus[count] = ((psdstatus[count] & SD_0TO7BITS) << 24) |((psdstatus[count] & SD_8TO15BITS) << 8) |
                       ((psdstatus[count] & SD_16TO23BITS) >> 8) |((psdstatus[count] & SD_24TO31BITS) >> 24);
  }
  return(errorstatus);
}

/*******************************************************************************
* Function Name  : SD_ProcessIRQSrc
* Description    : Allows to process all the interrupts that are high.
* Input          : None
* Output         : None
* Return         : SD_Error: SD Card Error code.
*******************************************************************************/
SD_Error SD_ProcessIRQSrc(void)
{
  uint32_t count = 0, restwords = 0;

  if (DeviceMode == SD_INTERRUPT_MODE)
  {
    if (SDIO_GetITStatus(SDIO_IT_RXFIFOHF) != RESET)
    {
      for (count = 0; count < SD_HALFFIFO; count++)
      {
        *(DestBuffer + count) = SDIO_ReadData();
      }
      DestBuffer += SD_HALFFIFO;
      NumberOfBytes += SD_HALFFIFOBYTES;
    }
    else if (SDIO_GetITStatus(SDIO_IT_TXFIFOHE) != RESET)
    {
      if ((TotalNumberOfBytes - NumberOfBytes) < SD_HALFFIFOBYTES)
      {
        restwords = ((TotalNumberOfBytes - NumberOfBytes) %  4 == 0) ?
                    ((TotalNumberOfBytes - NumberOfBytes) / 4) :
                    ((TotalNumberOfBytes - NumberOfBytes) / 4 + 1);

        for (count = 0; count < restwords;  count++, SrcBuffer++, NumberOfBytes += 4)
        {
          SDIO_WriteData(*SrcBuffer);
        }
      }
      else
      {
        for (count = 0; count < SD_HALFFIFO; count++)
        {
          SDIO_WriteData(*(SrcBuffer + count));
        }

        SrcBuffer += SD_HALFFIFO;
        NumberOfBytes += SD_HALFFIFOBYTES;
      }
    }
  }

  if (SDIO_GetITStatus(SDIO_IT_DATAEND) != RESET)
  {
    if (DeviceMode != SD_DMA_MODE)
    {
      while ((SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET)  &&  (NumberOfBytes < TotalNumberOfBytes))
      {
        *DestBuffer = SDIO_ReadData();
        DestBuffer++;
        NumberOfBytes += 4;
      }
    }

    if (StopCondition == 1)
    {
      TransferError = SD_StopTransfer();
    }
    else
    {
      TransferError = SD_OK;
    }
    SDIO_ClearITPendingBit(SDIO_IT_DATAEND);
    SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND |
                  SDIO_IT_TXFIFOHE | SDIO_IT_RXFIFOHF | SDIO_IT_TXUNDERR |
                  SDIO_IT_RXOVERR | SDIO_IT_STBITERR, DISABLE);
    TransferEnd = 1;
    NumberOfBytes = 0;
    return(TransferError);
  }

  if (SDIO_GetITStatus(SDIO_IT_DCRCFAIL) != RESET)
  {
    SDIO_ClearITPendingBit(SDIO_IT_DCRCFAIL);
    SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND |
                  SDIO_IT_TXFIFOHE | SDIO_IT_RXFIFOHF | SDIO_IT_TXUNDERR |
                  SDIO_IT_RXOVERR | SDIO_IT_STBITERR, DISABLE);
    NumberOfBytes = 0;
    TransferError = SD_DATA_CRC_FAIL;
    return(SD_DATA_CRC_FAIL);
  }

  if (SDIO_GetITStatus(SDIO_IT_DTIMEOUT) != RESET)
  {
    SDIO_ClearITPendingBit(SDIO_IT_DTIMEOUT);
    SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND |
                  SDIO_IT_TXFIFOHE | SDIO_IT_RXFIFOHF | SDIO_IT_TXUNDERR |
                  SDIO_IT_RXOVERR | SDIO_IT_STBITERR, DISABLE);
    NumberOfBytes = 0;
    TransferError = SD_DATA_TIMEOUT;
    return(SD_DATA_TIMEOUT);
  }

  if (SDIO_GetITStatus(SDIO_IT_RXOVERR) != RESET)
  {
    SDIO_ClearITPendingBit(SDIO_IT_RXOVERR);
    SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND |
                  SDIO_IT_TXFIFOHE | SDIO_IT_RXFIFOHF | SDIO_IT_TXUNDERR |
                  SDIO_IT_RXOVERR | SDIO_IT_STBITERR, DISABLE);
    NumberOfBytes = 0;
    TransferError = SD_RX_OVERRUN;
    return(SD_RX_OVERRUN);
  }

  if (SDIO_GetITStatus(SDIO_IT_TXUNDERR) != RESET)
  {
    SDIO_ClearITPendingBit(SDIO_IT_TXUNDERR);
    SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND |
                  SDIO_IT_TXFIFOHE | SDIO_IT_RXFIFOHF | SDIO_IT_TXUNDERR |
                  SDIO_IT_RXOVERR | SDIO_IT_STBITERR, DISABLE);
    NumberOfBytes = 0;
    TransferError = SD_TX_UNDERRUN;
    return(SD_TX_UNDERRUN);
  }

  if (SDIO_GetITStatus(SDIO_IT_STBITERR) != RESET)
  {
    SDIO_ClearITPendingBit(SDIO_IT_STBITERR);
    SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND |
                  SDIO_IT_TXFIFOHE | SDIO_IT_RXFIFOHF | SDIO_IT_TXUNDERR |
                  SDIO_IT_RXOVERR | SDIO_IT_STBITERR, DISABLE);
    NumberOfBytes = 0;
    TransferError = SD_START_BIT_ERR;
    return(SD_START_BIT_ERR);
  }

  return(SD_OK);
}

/*******************************************************************************
* Function Name  : CmdError
* Description    : Checks for error conditions for CMD0.
* Input          : None
* Output         : None
* Return         : SD_Error: SD Card Error code.
*******************************************************************************/
static SD_Error CmdError(void)
{
  SD_Error errorstatus = SD_OK;
  uint32_t timeout;

  timeout = SDIO_CMD0TIMEOUT; /* 10000 */

  while ((timeout > 0) && (SDIO_GetFlagStatus(SDIO_FLAG_CMDSENT) == RESET))
  {
    timeout--;
  }

  if (timeout == 0)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    return(errorstatus);
  }

  /* Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  return(errorstatus);
}

/*******************************************************************************
* Function Name  : CmdResp7Error
* Description    : Checks for error conditions for R7.
*                  response.
* Input          : None
* Output         : None
* Return         : SD_Error: SD Card Error code.
*******************************************************************************/
static SD_Error CmdResp7Error(void)
{
  SD_Error errorstatus = SD_OK;
  uint32_t status;
  uint32_t timeout = SDIO_CMD0TIMEOUT;

  status = SDIO->STA;

  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)) && (timeout > 0))
  {
    timeout--;
    status = SDIO->STA;
  }

  if ((timeout == 0) || (status & SDIO_FLAG_CTIMEOUT))
  {
    /* Card is not V2.0 complient or card does not support the set voltage range */
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }

  if (status & SDIO_FLAG_CMDREND)
  {
    /* Card is SD V2.0 compliant */
    errorstatus = SD_OK;
    SDIO_ClearFlag(SDIO_FLAG_CMDREND);
    return(errorstatus);
  }
  return(errorstatus);
}

/*******************************************************************************
* Function Name  : CmdResp1Error
* Description    : Checks for error conditions for R1.
*                  response
* Input          : cmd: The sent command index.
* Output         : None
* Return         : SD_Error: SD Card Error code.
*******************************************************************************/
static SD_Error CmdResp1Error(uint8_t cmd)
{
  SD_Error errorstatus = SD_OK;
  uint32_t status;
  uint32_t response_r1;

  status = SDIO->STA;

  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)))
  {
    status = SDIO->STA;
  }

  if (status & SDIO_FLAG_CTIMEOUT)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }
  else if (status & SDIO_FLAG_CCRCFAIL)
  {
    errorstatus = SD_CMD_CRC_FAIL;
    SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
    return(errorstatus);
  }

  /* Check response received is of desired command */
  if (SDIO_GetCommandResponse() != cmd)
  {
    errorstatus = SD_ILLEGAL_CMD;
    return(errorstatus);
  }

  /* Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  /* We have received response, retrieve it for analysis  */
  response_r1 = SDIO_GetResponse(SDIO_RESP1);

  if ((response_r1 & SD_OCR_ERRORBITS) == SD_ALLZERO)
  {
    return(errorstatus);
  }

  if (response_r1 & SD_OCR_ADDR_OUT_OF_RANGE)
  {
    return(SD_ADDR_OUT_OF_RANGE);
  }

  if (response_r1 & SD_OCR_ADDR_MISALIGNED)
  {
    return(SD_ADDR_MISALIGNED);
  }

  if (response_r1 & SD_OCR_BLOCK_LEN_ERR)
  {
    return(SD_BLOCK_LEN_ERR);
  }

  if (response_r1 & SD_OCR_ERASE_SEQ_ERR)
  {
    return(SD_ERASE_SEQ_ERR);
  }

  if (response_r1 & SD_OCR_BAD_ERASE_PARAM)
  {
    return(SD_BAD_ERASE_PARAM);
  }

  if (response_r1 & SD_OCR_WRITE_PROT_VIOLATION)
  {
    return(SD_WRITE_PROT_VIOLATION);
  }

  if (response_r1 & SD_OCR_LOCK_UNLOCK_FAILED)
  {
    return(SD_LOCK_UNLOCK_FAILED);
  }

  if (response_r1 & SD_OCR_COM_CRC_FAILED)
  {
    return(SD_COM_CRC_FAILED);
  }

  if (response_r1 & SD_OCR_ILLEGAL_CMD)
  {
    return(SD_ILLEGAL_CMD);
  }

  if (response_r1 & SD_OCR_CARD_ECC_FAILED)
  {
    return(SD_CARD_ECC_FAILED);
  }

  if (response_r1 & SD_OCR_CC_ERROR)
  {
    return(SD_CC_ERROR);
  }

  if (response_r1 & SD_OCR_GENERAL_UNKNOWN_ERROR)
  {
    return(SD_GENERAL_UNKNOWN_ERROR);
  }

  if (response_r1 & SD_OCR_STREAM_READ_UNDERRUN)
  {
    return(SD_STREAM_READ_UNDERRUN);
  }

  if (response_r1 & SD_OCR_STREAM_WRITE_OVERRUN)
  {
    return(SD_STREAM_WRITE_OVERRUN);
  }

  if (response_r1 & SD_OCR_CID_CSD_OVERWRIETE)
  {
    return(SD_CID_CSD_OVERWRITE);
  }

  if (response_r1 & SD_OCR_WP_ERASE_SKIP)
  {
    return(SD_WP_ERASE_SKIP);
  }

  if (response_r1 & SD_OCR_CARD_ECC_DISABLED)
  {
    return(SD_CARD_ECC_DISABLED);
  }

  if (response_r1 & SD_OCR_ERASE_RESET)
  {
    return(SD_ERASE_RESET);
  }

  if (response_r1 & SD_OCR_AKE_SEQ_ERROR)
  {
    return(SD_AKE_SEQ_ERROR);
  }
  return(errorstatus);
}

/*******************************************************************************
* Function Name  : CmdResp3Error
* Description    : Checks for error conditions for R3 (OCR).
*                  response.
* Input          : None
* Output         : None
* Return         : SD_Error: SD Card Error code.
*******************************************************************************/
static SD_Error CmdResp3Error(void)
{
  SD_Error errorstatus = SD_OK;
  uint32_t status;

  status = SDIO->STA;

  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)))
  {
    status = SDIO->STA;
  }

  if (status & SDIO_FLAG_CTIMEOUT)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }
  /* Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);
  return(errorstatus);
}

/*******************************************************************************
* Function Name  : CmdResp2Error
* Description    : Checks for error conditions for R2 (CID or CSD).
*                  response.
* Input          : None
* Output         : None
* Return         : SD_Error: SD Card Error code.
*******************************************************************************/
static SD_Error CmdResp2Error(void)
{
  SD_Error errorstatus = SD_OK;
  uint32_t status;

  status = SDIO->STA;

  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CMDREND)))
  {
    status = SDIO->STA;
  }

  if (status & SDIO_FLAG_CTIMEOUT)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }
  else if (status & SDIO_FLAG_CCRCFAIL)
  {
    errorstatus = SD_CMD_CRC_FAIL;
    SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
    return(errorstatus);
  }

  /* Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  return(errorstatus);
}

/*******************************************************************************
* Function Name  : CmdResp6Error
* Description    : Checks for error conditions for R6 (RCA).
*                  response.
* Input          : - cmd: The sent command index.
*                  - prca: pointer to the variable that will contain the SD
*                    card relative address RCA. 
* Output         : None
* Return         : SD_Error: SD Card Error code.
*******************************************************************************/
static SD_Error CmdResp6Error(uint8_t cmd, uint16_t *prca)
{
  SD_Error errorstatus = SD_OK;
  uint32_t status;
  uint32_t response_r1;

  status = SDIO->STA;

  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CMDREND)))
  {
    status = SDIO->STA;
  }

  if (status & SDIO_FLAG_CTIMEOUT)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }
  else if (status & SDIO_FLAG_CCRCFAIL)
  {
    errorstatus = SD_CMD_CRC_FAIL;
    SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
    return(errorstatus);
  }

  /* Check response received is of desired command */
  if (SDIO_GetCommandResponse() != cmd)
  {
    errorstatus = SD_ILLEGAL_CMD;
    return(errorstatus);
  }

  /* Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  /* We have received response, retrieve it.  */
  response_r1 = SDIO_GetResponse(SDIO_RESP1);

  if (SD_ALLZERO == (response_r1 & (SD_R6_GENERAL_UNKNOWN_ERROR | SD_R6_ILLEGAL_CMD | SD_R6_COM_CRC_FAILED)))
  {
    *prca = (uint16_t) (response_r1 >> 16);
    return(errorstatus);
  }

  if (response_r1 & SD_R6_GENERAL_UNKNOWN_ERROR)
  {
    return(SD_GENERAL_UNKNOWN_ERROR);
  }

  if (response_r1 & SD_R6_ILLEGAL_CMD)
  {
    return(SD_ILLEGAL_CMD);
  }

  if (response_r1 & SD_R6_COM_CRC_FAILED)
  {
    return(SD_COM_CRC_FAILED);
  }

  return(errorstatus);
}

/*******************************************************************************
* Function Name  : SDEnWideBus
* Description    : Enables or disables the SDIO wide bus mode.
* Input          : NewState: new state of the SDIO wide bus mode.
*                  This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : SD_Error: SD Card Error code.
*******************************************************************************/
static SD_Error SDEnWideBus(FunctionalState NewState)
{
  SD_Error errorstatus = SD_OK;

  uint32_t scr[2] = {0, 0};

  if (SDIO_GetResponse(SDIO_RESP1) & SD_CARD_LOCKED)
  {
    errorstatus = SD_LOCK_UNLOCK_FAILED;
    return(errorstatus);
  }

  /* Get SCR Register */
  errorstatus = FindSCR(RCA, scr);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  /* If wide bus operation to be enabled */
  if (NewState == ENABLE)
  {
    /* If requested card supports wide bus operation */
    if ((scr[1] & SD_WIDE_BUS_SUPPORT) != SD_ALLZERO)
    {
      /* Send CMD55 APP_CMD with argument as card's RCA.*/
      SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) RCA << 16;
      SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_APP_CMD;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(SDIO_APP_CMD);

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }

      /* Send ACMD6 APP_CMD with argument as 2 for wide bus mode */
      SDIO_CmdInitStructure.SDIO_Argument = 0x2;
      SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_APP_SD_SET_BUSWIDTH;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(SDIO_APP_SD_SET_BUSWIDTH);

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }
      return(errorstatus);
    }
    else
    {
      errorstatus = SD_REQUEST_NOT_APPLICABLE;
      return(errorstatus);
    }
  }   /* If wide bus operation to be disabled */
  else
  {
    /* If requested card supports 1 bit mode operation */
    if ((scr[1] & SD_SINGLE_BUS_SUPPORT) != SD_ALLZERO)
    {
      /* Send CMD55 APP_CMD with argument as card's RCA.*/
      SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) RCA << 16;
      SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_APP_CMD;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);


      errorstatus = CmdResp1Error(SDIO_APP_CMD);

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }

      /* Send ACMD6 APP_CMD with argument as 2 for wide bus mode */
      SDIO_CmdInitStructure.SDIO_Argument = 0x00;
      SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_APP_SD_SET_BUSWIDTH;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(SDIO_APP_SD_SET_BUSWIDTH);

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }

      return(errorstatus);
    }
    else
    {
      errorstatus = SD_REQUEST_NOT_APPLICABLE;
      return(errorstatus);
    }
  }
}

/*******************************************************************************
* Function Name  : IsCardProgramming
* Description    : Checks if the SD card is in programming state.
* Input          : pstatus: pointer to the variable that will contain the SD
*                  card state.
* Output         : None
* Return         : SD_Error: SD Card Error code.
*******************************************************************************/
static SD_Error IsCardProgramming(uint8_t *pstatus)
{
  SD_Error errorstatus = SD_OK;
  __IO uint32_t respR1 = 0, status = 0;

  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) RCA << 16;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SEND_STATUS;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  status = SDIO->STA;
  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)))
  {
    status = SDIO->STA;
  }

  if (status & SDIO_FLAG_CTIMEOUT)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }
  else if (status & SDIO_FLAG_CCRCFAIL)
  {
    errorstatus = SD_CMD_CRC_FAIL;
    SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
    return(errorstatus);
  }

  status = (uint32_t)SDIO_GetCommandResponse();

  /* Check response received is of desired command */
  if (status != SDIO_SEND_STATUS)
  {
    errorstatus = SD_ILLEGAL_CMD;
    return(errorstatus);
  }

  /* Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);


  /* We have received response, retrieve it for analysis  */
  respR1 = SDIO_GetResponse(SDIO_RESP1);

  /* Find out card status */
  *pstatus = (uint8_t) ((respR1 >> 9) & 0x0000000F);

  if ((respR1 & SD_OCR_ERRORBITS) == SD_ALLZERO)
  {
    return(errorstatus);
  }

  if (respR1 & SD_OCR_ADDR_OUT_OF_RANGE)
  {
    return(SD_ADDR_OUT_OF_RANGE);
  }

  if (respR1 & SD_OCR_ADDR_MISALIGNED)
  {
    return(SD_ADDR_MISALIGNED);
  }

  if (respR1 & SD_OCR_BLOCK_LEN_ERR)
  {
    return(SD_BLOCK_LEN_ERR);
  }

  if (respR1 & SD_OCR_ERASE_SEQ_ERR)
  {
    return(SD_ERASE_SEQ_ERR);
  }

  if (respR1 & SD_OCR_BAD_ERASE_PARAM)
  {
    return(SD_BAD_ERASE_PARAM);
  }

  if (respR1 & SD_OCR_WRITE_PROT_VIOLATION)
  {
    return(SD_WRITE_PROT_VIOLATION);
  }

  if (respR1 & SD_OCR_LOCK_UNLOCK_FAILED)
  {
    return(SD_LOCK_UNLOCK_FAILED);
  }

  if (respR1 & SD_OCR_COM_CRC_FAILED)
  {
    return(SD_COM_CRC_FAILED);
  }

  if (respR1 & SD_OCR_ILLEGAL_CMD)
  {
    return(SD_ILLEGAL_CMD);
  }

  if (respR1 & SD_OCR_CARD_ECC_FAILED)
  {
    return(SD_CARD_ECC_FAILED);
  }

  if (respR1 & SD_OCR_CC_ERROR)
  {
    return(SD_CC_ERROR);
  }

  if (respR1 & SD_OCR_GENERAL_UNKNOWN_ERROR)
  {
    return(SD_GENERAL_UNKNOWN_ERROR);
  }

  if (respR1 & SD_OCR_STREAM_READ_UNDERRUN)
  {
    return(SD_STREAM_READ_UNDERRUN);
  }

  if (respR1 & SD_OCR_STREAM_WRITE_OVERRUN)
  {
    return(SD_STREAM_WRITE_OVERRUN);
  }

  if (respR1 & SD_OCR_CID_CSD_OVERWRIETE)
  {
    return(SD_CID_CSD_OVERWRITE);
  }

  if (respR1 & SD_OCR_WP_ERASE_SKIP)
  {
    return(SD_WP_ERASE_SKIP);
  }

  if (respR1 & SD_OCR_CARD_ECC_DISABLED)
  {
    return(SD_CARD_ECC_DISABLED);
  }

  if (respR1 & SD_OCR_ERASE_RESET)
  {
    return(SD_ERASE_RESET);
  }

  if (respR1 & SD_OCR_AKE_SEQ_ERROR)
  {
    return(SD_AKE_SEQ_ERROR);
  }

  return(errorstatus);
}

/*******************************************************************************
* Function Name  : FindSCR
* Description    : Find the SD card SCR register value.
* Input          : - rca: selected card address.
*                  - pscr: pointer to the buffer that will contain the SCR value.
* Output         : None
* Return         : SD_Error: SD Card Error code.
*******************************************************************************/
static SD_Error FindSCR(uint16_t rca, uint32_t *pscr)
{
  uint32_t index = 0;
  SD_Error errorstatus = SD_OK;
  uint32_t tempscr[2] = {0, 0};

  /* Set Block Size To 8 Bytes */
  /* Send CMD55 APP_CMD with argument as card's RCA */
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)8;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SET_BLOCKLEN;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SDIO_SET_BLOCKLEN);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  /* Send CMD55 APP_CMD with argument as card's RCA */
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) RCA << 16;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_APP_CMD;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SDIO_APP_CMD);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }
  SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
  SDIO_DataInitStructure.SDIO_DataLength = 8;
  SDIO_DataInitStructure.SDIO_DataBlockSize = SDIO_DataBlockSize_8b;
  SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;
  SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
  SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
  SDIO_DataConfig(&SDIO_DataInitStructure);


  /* Send ACMD51 SD_APP_SEND_SCR with argument as 0 */
  SDIO_CmdInitStructure.SDIO_Argument = 0x0;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SD_APP_SEND_SCR;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SDIO_SD_APP_SEND_SCR);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  while (!(SDIO->STA & (SDIO_FLAG_RXOVERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_DBCKEND | SDIO_FLAG_STBITERR)))
  {
    if (SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET)
    {
      *(tempscr + index) = SDIO_ReadData();
      index++;
    }
  }

  if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
    errorstatus = SD_DATA_TIMEOUT;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
    errorstatus = SD_DATA_CRC_FAIL;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_RXOVERR);
    errorstatus = SD_RX_OVERRUN;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_STBITERR);
    errorstatus = SD_START_BIT_ERR;
    return(errorstatus);
  }

  /* Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  *(pscr + 1) = ((tempscr[0] & SD_0TO7BITS) << 24) | ((tempscr[0] & SD_8TO15BITS) << 8) | ((tempscr[0] & SD_16TO23BITS) >> 8) | ((tempscr[0] & SD_24TO31BITS) >> 24);

  *(pscr) = ((tempscr[1] & SD_0TO7BITS) << 24) | ((tempscr[1] & SD_8TO15BITS) << 8) | ((tempscr[1] & SD_16TO23BITS) >> 8) | ((tempscr[1] & SD_24TO31BITS) >> 24);

  return(errorstatus);
}

/*******************************************************************************
* Function Name  : convert_from_bytes_to_power_of_two
* Description    : Converts the number of bytes in power of two and returns the
*                  power.
* Input          : NumberOfBytes: number of bytes.
* Output         : None
* Return         : None
*******************************************************************************/
static uint8_t convert_from_bytes_to_power_of_two(uint16_t NumberOfBytes)
{
  uint8_t count = 0;

  while (NumberOfBytes != 1)
  {
    NumberOfBytes >>= 1;
    count++;
  }
  return(count);
}

//配置SDIO DMA  
//mbuf:存储器地址
//bufsize:传输数据量
//DMA_DIR:方向; @ref DMA_data_transfer_direction      DMA_DIR_PeripheralDST ,存储器-->SDIO(写数据);DMA_DIR_PeripheralSRC,SDIO-->存储器(读数据);
void SDIO_DMA_Configuration(u32*mbuf,u32 bufsize,u32 DMA_DIR)
{		 
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);	//使能DMA2时钟

	DMA_DeInit(DMA2_Channel4);   //将DMA2的通道4寄存器重设为缺省值
	DMA_Cmd(DMA2_Channel4, DISABLE ); //关闭DMA2 通道4

	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SDIO->FIFO;  //DMA外设基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)mbuf;  //DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR;  //数据传输方向，从内存读取发送到外设
	DMA_InitStructure.DMA_BufferSize = bufsize/4;  //DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;  //数据宽度为32位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word; //数据宽度为32位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //工作在正常缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; //DMA通道 x拥有高优先级 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA2_Channel4, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道USART1_Tx_DMA_Channel所标识的寄存器

	DMA_Cmd(DMA2_Channel4, DISABLE ); //开启DMA2 通道4
} 

/*******************************************************************************
* Function Name  : DMA_TxConfiguration
* Description    : Configures the DMA2 Channel4 for SDIO Tx request.
* Input          : - BufferSRC: pointer to the source buffer
*                  - BufferSize: buffer size
* Output         : None
* Return         : None
*******************************************************************************/
static void DMA_TxConfiguration(uint32_t *BufferSRC, uint32_t BufferSize)
{
  DMA_InitTypeDef DMA_InitStructure;

  DMA_ClearFlag(DMA2_FLAG_TC4 | DMA2_FLAG_TE4 | DMA2_FLAG_HT4 | DMA2_FLAG_GL4);

  /* DMA2 Channel4 disable */
  DMA_Cmd(DMA2_Channel4, DISABLE);

  /* DMA2 Channel4 Config */
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SDIO_FIFO_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)BufferSRC;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = BufferSize / 4;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA2_Channel4, &DMA_InitStructure);

  /* DMA2 Channel4 enable */
  DMA_Cmd(DMA2_Channel4, ENABLE);
}

/*******************************************************************************
* Function Name  : DMA_RxConfiguration
* Description    : Configures the DMA2 Channel4 for SDIO Rx request.
* Input          : - BufferDST: pointer to the destination buffer
*                  - BufferSize: buffer size
* Output         : None
* Return         : None
*******************************************************************************/
static void DMA_RxConfiguration(uint32_t *BufferDST, uint32_t BufferSize)
{
  DMA_InitTypeDef DMA_InitStructure;

  DMA_ClearFlag(DMA2_FLAG_TC4 | DMA2_FLAG_TE4 | DMA2_FLAG_HT4 | DMA2_FLAG_GL4);

  /* DMA2 Channel4 disable */
  DMA_Cmd(DMA2_Channel4, DISABLE);

  /* DMA2 Channel4 Config */
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SDIO_FIFO_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)BufferDST;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = BufferSize / 4;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA2_Channel4, &DMA_InitStructure);

  /* DMA2 Channel4 enable */
  DMA_Cmd(DMA2_Channel4, ENABLE);
}



//读SD卡
//buf:读数据缓存区
//sector:扇区地址
//cnt:扇区个数	
//返回值:错误状态;0,正常;其他,错误代码;				  				 
unsigned char SD_ReadDisk(u8*buf,u32 sector,u8 cnt)
{
	u8 sta=SD_OK;
	long long lsector=sector;
	u8 n;
	lsector<<=9;
	if((u32)buf%4!=0)
	{
	 	for(n=0;n<cnt;n++)
		{
		 	sta=SD_ReadBlock(SDIO_DATA_BUFFER,lsector+512*n,512);//单个sector的读操作
			memcpy(buf,SDIO_DATA_BUFFER,512);
			buf+=512;
		} 
	}
	else
	{
		//log_info("cnt0:%d\r\n",cnt);
		if(cnt==1){sta=SD_ReadBlock(buf,lsector,512);}   	//单个sector的读操作
		else {sta=SD_ReadMultiBlocks((uint32_t *)buf,lsector,512,cnt);}//多个sector
	}
	//log_info("sta:0x%x\r\n",sta);
	return sta;
}

//写SD卡
//buf:写数据缓存区
//sector:扇区地址
//cnt:扇区个数	
//返回值:错误状态;0,正常;其他,错误代码;	
unsigned char SD_WriteDisk(u8*buf,u32 sector,u8 cnt)
{
	u8 sta=SD_OK;
	u8 n;
	long long lsector=sector;
	lsector<<=9;
	if((u32)buf%4!=0)
	{
	 	for(n=0;n<cnt;n++)
		{
			memcpy(SDIO_DATA_BUFFER,buf,512);
		 	sta=SD_WriteBlock(SDIO_DATA_BUFFER,lsector+512*n,512);//单个sector的写操作
			buf+=512;
		} 
	}else
	{
		if(cnt==1)sta=SD_WriteBlock(buf,lsector,512);    	//单个sector的写操作
		else sta=SD_WriteMultiBlocks((uint32_t *)buf,lsector,512,cnt);	//多个sector  
	}
	return sta;
}




/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/



