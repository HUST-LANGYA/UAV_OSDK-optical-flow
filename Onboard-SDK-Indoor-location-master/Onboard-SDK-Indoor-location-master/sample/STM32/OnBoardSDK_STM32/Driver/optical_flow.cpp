#include "optical_flow.h"
#include "main.h"

_Flow  Flow;
uint8_t flow_time;
static uint8_t rx_data[RX_BUF_NUM];
int16_t dis_x;
int16_t dis_y;

void Optical_Usart_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
	  DMA_InitTypeDef   DMA_InitStructure;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12 , GPIO_AF_UART5); // tx
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5); // rx
		
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12 ;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 ;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

		USART_InitStructure.USART_BaudRate            = 500000;
		USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits            = USART_StopBits_1;
		USART_InitStructure.USART_Parity              = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;

		USART_Init(UART5, &USART_InitStructure);
		
		USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);
		
		USART_DMACmd(UART5 , USART_DMAReq_Rx , ENABLE);
 
    DMA_DeInit(DMA1_Stream0);
    DMA_InitStructure.DMA_Channel            = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (UART5->DR);
		DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)rx_data;
		DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
		DMA_InitStructure.DMA_BufferSize         = RX_BUF_NUM;
		DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
   	DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
		DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
		DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;
		DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
		DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream0 , &DMA_InitStructure);
 
    DMA_ClearFlag(DMA1_Stream0 , DMA_FLAG_TCIF0);
		DMA_Cmd(DMA1_Stream0 , ENABLE);
		USART_Cmd(UART5, ENABLE);
}


//光流数据接收后分析  具体看光流的数据手册
static void optical_data_analysis(uint8_t *data_buf , uint8_t num)
{
    int8_t	Length = num;
	  uint8_t sum = 0;
		uint8_t add = 0;
	  static uint8_t receiver_flow_flag = 0;
	  static uint8_t _data_len = 0;
	  static uint64_t last_time = 0;
	  
	  while(Length > 0)
		{
			  switch(receiver_flow_flag)
				{
				case 0:	
					  if (data_buf[num - Length] == 0xAA && receiver_flow_flag == 0)
						{
						    receiver_flow_flag = 1;
                Length -= 1;							
						}
						else
						{
							  receiver_flow_flag = 0;
						    Length = 0;	
						}
				    break;
				case 1:		
					  if (data_buf[num - Length] == 0xFF && receiver_flow_flag == 1)
						{
							  receiver_flow_flag = 2;
							  Length -= 1;
						}
						else
						{
							  receiver_flow_flag = 0;
						    Length = 0;	
						}
						break;			
        case 2:
					  if (data_buf[num - Length] == 0X51 && receiver_flow_flag == 2)//光流信息融合后的数据
						{
							  receiver_flow_flag = 4;  
						}
						else if (data_buf[num - Length] == 0X34 && receiver_flow_flag == 2)//高度信息
						{
						    receiver_flow_flag = 5;	
						}
						Length -= 1;
            break;
				case 4://光流信息处理
				   sum=0;
					 add=0;
					 int16_t x_change=0;
					 int16_t y_change=0;
				   _data_len=data_buf[3];
						for(int t=0;t<data_buf[3]+4;t++)
            {
						   sum+=data_buf[t];
							 add+=sum;
						}						
							if(sum!=data_buf[data_buf[3]+4]||add!=data_buf[data_buf[3]+5])	
							{	
							return;
              }							
						Flow.OF_DX2 = (int16_t)((data_buf[num - Length + 3] << 8)| data_buf[num - Length + 4]);	
						Flow.OF_DY2 = (int16_t)((data_buf[num - Length + 5] << 8)| data_buf[num - Length + 6]);
           if(10000>abs(Flow.OF_DX2FIX-(int16_t)((data_buf[num - Length + 7] << 8)| data_buf[num - Length + 8])))						
						{
						  Flow.OF_DX2FIX = (int16_t)((data_buf[num - Length + 7] << 8)| data_buf[num - Length + 8]);	
						}
					if(10000>abs(Flow.OF_DY2FIX-(int16_t)((data_buf[num - Length + 9] << 8)| data_buf[num - Length + 10])))
						{
						 Flow.OF_DY2FIX = (int16_t)((data_buf[num - Length + 9] << 8)| data_buf[num - Length + 10]);
           	}					
						Flow.OF_QUALITY = data_buf[num - Length + 15];
//						flow_time = (v->protocolLayer->getDriver()->getTimeStamp() - last_time) * 0.001f;//大疆提供的API数据
//						last_time = v->protocolLayer->getDriver()->getTimeStamp();
//						Flow.DISTANCE_X += flow_time * Flow.OF_DX2FIX;
//						Flow.DISTANCE_Y += flow_time * Flow.OF_DY2FIX; 
            dis_x=(int16_t)((data_buf[num-Length+11]<<8)|data_buf[num-Length+12]);
            dis_y=(int16_t)((data_buf[num-Length+13]<<8)|data_buf[num-Length+14]);
						
						Length -= _data_len + 1;
						receiver_flow_flag = 0;
            break;						
				case 5:	//高度信息处理	
					 sum=0;
					 add=0;
				   _data_len=data_buf[3];
						for(int t=0;t<data_buf[3]+4;t++)
            {
						   sum+=data_buf[t];
							 add+=sum;
						}						
							if(sum!=data_buf[data_buf[3]+4]||add!=data_buf[data_buf[3]+5])	
							{	
							return;
              }					
	          {
			          static uint8_t length = 0;
							  //存历史激光高度数据，求变化率
								if(0xFFFFFFFF != (data_buf[num - Length + 4]<<24|data_buf[num - Length + 5]<<16|data_buf[num-Length+6]<<8|data_buf[num-Length+7]))
								{
								Flow.OF_ALT = ((int32_t)(data_buf[num - Length + 4]<<24|data_buf[num - Length + 5]<<16|data_buf[num-Length+6]<<8|data_buf[num-Length+7]))/10000000.0;		
								if(Flow.OF_ALT	> Flow.HISTORY_OF_ALT[length])
								  {
										if(Flow.OF_ALT	- Flow.HISTORY_OF_ALT[length]<30)
										{		
								    Flow.ALT_RATE_OF_CHANGE = Flow.OF_ALT	- Flow.HISTORY_OF_ALT[length];
										Flow.OF_ALT2=Flow.OF_ALT;
										Flow.HISTORY_OF_ALT[length] = Flow.OF_ALT;
										}
                  }
								else
								  {
										if(Flow.HISTORY_OF_ALT[length]-Flow.OF_ALT <30)
										{
								    Flow.ALT_RATE_OF_CHANGE =  Flow.HISTORY_OF_ALT[length]-Flow.OF_ALT ;
										Flow.OF_ALT2=Flow.OF_ALT;
										Flow.HISTORY_OF_ALT[length] = Flow.OF_ALT;
										}
								  }
									
                }								
						}
						Length -= _data_len + 1;
						receiver_flow_flag = 0;								
            break;
        default:
					  receiver_flow_flag = 0;
				    Length = 0;
            break;					
				} 				
	  }
}

#ifdef __cplusplus
extern "C" 
{
void UART5_IRQHandler(void)
{
	  static uint8_t buff_length = 0;
	  if (USART_GetFlagStatus(UART5, USART_FLAG_ORE) != RESET)//当DR寄存器尚未读出已满时，移位寄存器中有数据需要移至DR，此时若再来数据，DR寄存器数据不会被覆盖，但是移位寄存器中数据会被覆盖造成丢失
	  {
		    USART_ReceiveData(UART5);//将此时DR寄存器中的数据读出，缓解过载.但此时DR寄存器中的值丢失
		    USART_ClearFlag(UART5, USART_FLAG_ORE);
	  }
	  if (USART_GetITStatus(UART5 , USART_IT_IDLE) != RESET)
		{
			  DMA_Cmd(DMA1_Stream0 , DISABLE);
			  DMA_ClearFlag(DMA1_Stream0 , DMA_FLAG_TCIF0); 
        buff_length = RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream0);
			  optical_data_analysis(rx_data , buff_length-1);
			  DMA_SetCurrDataCounter(DMA1_Stream0 , 50);
			  DMA_Cmd(DMA1_Stream0 , ENABLE);
        USART_ReceiveData(UART5);	//先RXNE标志，避免过载
				
		}	
}

}
#endif //__cplusplus