#include "comunication.h"
#include "main.h"



bool isACKProcessed    = false;
bool ackReceivedByUser = false;
DJI::OSDK::ACK::ErrorCode waitForACK();
static void DMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar,u16 ndtr);

static void Usart2_Gpio_Config(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); // tx
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); // rx
}


static void Usart3_Gpio_Config(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3); // tx
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3); // rx
}


//串口二用于单片机与N3飞控通讯
static void Usart2_Config(void)
{
		Usart2_Gpio_Config();

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		USART_InitTypeDef USART_InitStructure;

		USART_InitStructure.USART_BaudRate   = 230400;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits   = USART_StopBits_1;
		USART_InitStructure.USART_Parity     = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl =
			USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

		USART_Init(USART2, &USART_InitStructure);
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

		USART_Cmd(USART2, ENABLE);

		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) != SET)
			;
}


//串口三用于与电脑通讯，方便电脑查看SDK状态，出错会发具体信息到电脑端，方便找出问题
static void Usart3_Config(void)
{
		Usart3_Gpio_Config();

		USART_InitTypeDef USART_InitStructure;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

		USART_InitStructure.USART_BaudRate   = 115200;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits   = USART_StopBits_1;
		USART_InitStructure.USART_Parity     = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl =
			USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

		USART_Init(USART3, &USART_InitStructure);
//		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

		USART_Cmd(USART3, ENABLE);
		while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) != SET)//发送寄存器空则跳出循环
			;//如果对DR寄存器写操作，则TXE标志位被清零
}

void UsartConfig()
{
		Usart2_Config();
		Usart3_Config();
//	Ano_Usart_Config();
}


#ifdef __cplusplus
extern "C" 
{
#endif //__cplusplus
    

void USART2_IRQHandler(void)
{
  if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == SET)//RDR寄存器非空，收到了数据
  {
		USART_ClearFlag(USART2, USART_FLAG_RXNE);
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    isACKProcessed = false;
    isFrame = v->protocolLayer->byteHandler(USART_ReceiveData(USART2));
    if (isFrame == true)
    {
			rFrame = v->protocolLayer->getReceivedFrame();
			
      //! Trigger default or user defined callback
      v->processReceivedData(rFrame);

      //! Reset
      isFrame        = false;
      isACKProcessed = true;
    }
  }
	if (USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET)//数据过载错误
	{
		  USART_ReceiveData(USART2);
		  USART_ClearFlag(USART2, USART_FLAG_ORE);
	}	
}


#ifdef __cplusplus
}
#endif //__cplusplus

static void DMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar,u16 ndtr)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);

  DMA_DeInit(DMA_Streamx);
	
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	

  DMA_InitStructure.DMA_Channel            = chx;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = par;
  DMA_InitStructure.DMA_Memory0BaseAddr    = mar;
  DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize         = ndtr;
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal; 
  DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;
  DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
  DMA_Init(DMA_Streamx, &DMA_InitStructure);
}


void DMA1_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{
 
	DMA_Cmd(DMA_Streamx, DISABLE);                     
	
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	
		
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);         
 
	DMA_Cmd(DMA_Streamx, ENABLE);                     
}	

