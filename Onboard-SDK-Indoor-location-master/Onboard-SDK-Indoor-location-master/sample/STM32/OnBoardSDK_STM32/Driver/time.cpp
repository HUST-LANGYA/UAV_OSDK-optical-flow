#include "time.h"
#include "main.h"


uint32_t tick = 0; 
extern TerminalCommand myTerminal;

static void Timer_Init(unsigned int Handler_Frequency)
{

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIM_DeInit(TIM3);
	
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 1000*1000/Handler_Frequency;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 84-1;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
	TIM_ClearFlag(TIM3,TIM_FLAG_Update);
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	
	TIM_Cmd(TIM3,ENABLE);

}

static void Timer1Config()
{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
		NVIC_InitTypeDef NVIC_InitStructure;

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
		TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;
		TIM_TimeBaseInitStructure.TIM_Period =
			(200 - 1); // t is the time between each Timer irq.
		TIM_TimeBaseInitStructure.TIM_Prescaler =
			(8400 - 1); // t = (1+TIM_Prescaler/SystemCoreClock)*(1+TIM_Period)
		TIM_TimeBaseInitStructure.TIM_RepetitionCounter =
			0x00; // here configure TIM1 in 50Hz
		TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

		NVIC_InitStructure.NVIC_IRQChannel                   = TIM1_UP_TIM10_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 3;
		NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		TIM_ClearFlag(TIM1, TIM_FLAG_Update);
		TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
		TIM_Cmd(TIM1, DISABLE);
}

static void Timer2Config()
{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		NVIC_InitTypeDef NVIC_InitStructure;

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;
		TIM_TimeBaseInitStructure.TIM_Period =
			(200 - 1); // t is the time between each Timer irq.
		TIM_TimeBaseInitStructure.TIM_Period =
			(40 - 1); // t is the time between each Timer irq.
		TIM_TimeBaseInitStructure.TIM_Prescaler =
			(42000 - 1); // t = (1+TIM_Prescaler/SystemCoreClock)*(1+TIM_Period)
		TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

		NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 3;
		NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		TIM_ClearFlag(TIM2, TIM_FLAG_Update);
		TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
		TIM_Cmd(TIM2, DISABLE);
}

void SystickConfig()
{
		if (SysTick_Config(SystemCoreClock / 1000)) // 1000 ticks per second.
		{
				while (1)
					; // run here when error.
		}
}

void Timer_Condfig()
{
	  SystickConfig();
//		Timer_Init(1000);	
		Timer1Config();
		Timer2Config();
}

void delay_nms(uint16_t time)
{
		u32 i = 0;
		while (time--)
		{
				i = 30000;
				while (i--)
					;
		}
}

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

void SysTick_Handler(void)
{
		if (tick > 4233600000ll) // 49 days non-reset would cost a tick reset.
		{
				tick = 0;
		}
		tick++;
}
void TIM1_UP_TIM10_IRQHandler(void)
{
		if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
		{
			//    virtualrc.sendData(myVRCdata);
		}
		TIM_ClearFlag(TIM1, TIM_FLAG_Update);
}

void TIM2_IRQHandler()
{
		if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
		{
				if ((myTerminal.cmdIn[2] == 0x04) && (myTerminal.cmdIn[3] == 0x01))
				{
					//      flight.setFlight(&flightData);
				}
				else
				{
						TIM_Cmd(TIM2, DISABLE);
				}
		}
		TIM_ClearFlag(TIM2, TIM_FLAG_Update);
}
#ifdef __cplusplus

}
#endif //__cplusplus



