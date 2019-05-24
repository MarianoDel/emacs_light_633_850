//---------------------------------------------
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ##
// #### TIM.C ################################
//---------------------------------------------

/* Includes ------------------------------------------------------------------*/
#include "tim.h"
#include "hard.h"

//--- VARIABLES EXTERNAS ---//
extern volatile unsigned char timer_1seg;
extern volatile unsigned short timer_led_comm;
extern volatile unsigned short wait_ms_var;

//--- VARIABLES GLOBALES ---//

//--- FUNCIONES DEL MODULO ---//
void Update_TIM3_CH1 (unsigned short a)
{
    TIM3->CCR1 = a;
}

void Update_TIM3_CH2 (unsigned short a)
{
    TIM3->CCR2 = a;
}

void Update_TIM3_CH3 (unsigned short a)
{
    TIM3->CCR3 = a;
}

void Update_TIM3_CH4 (unsigned short a)
{
    TIM3->CCR4 = a;
}

void Update_TIM1_CH1 (unsigned short a)
{
    TIM1->CCR1 = a;
}

void Update_TIM1_CH4 (unsigned short a)
{
    TIM1->CCR4 = a;
}

void Wait_ms (unsigned short wait)
{
    wait_ms_var = wait;

    while (wait_ms_var);
}

//-------------------------------------------//
// @brief  TIM configure.
// @param  None
// @retval None
//------------------------------------------//

void TIM_1_Init (void)
{
    unsigned long temp;

    if (!RCC_TIM1_CLK)
        RCC_TIM1_CLK_ON;

    //Configuracion del timer.
    TIM1->CR1 = 0x00;		//clk int / 1; upcounting
    TIM1->CR2 |= TIM_CR2_MMS_1;		//UEV -> TRG0

    TIM1->SMCR = 0x0000;
    TIM1->CCMR1 = 0x0060;    //CH1 output PWM mode 1 (channel active TIM1->CNT < TIM1->CCR1)
    TIM1->CCMR2 = 0x6000;    //CH4 output PWM mode 1 (channel active TIM1->CNT < TIM1->CCR1)
    
    TIM1->CCER |= TIM_CCER_CC4E | TIM_CCER_CC1E;	//CH4 y CH1 enable on pin direct polarity

    TIM1->BDTR |= TIM_BDTR_MOE;
    
    TIM1->ARR = DUTY_100_PERCENT;
    TIM1->CNT = 0;

#if defined USE_FREQ_48KHZ
    TIM1->PSC = 0;
#elif defined USE_FREQ_24KHZ
    TIM1->PSC = 1;
#elif defined USE_FREQ_16KHZ
    TIM1->PSC = 2;
#else
#error "No FREQ selected for TIM1 on hard.h"
#endif

    //Alternate Fuction Pin Configurations
    temp = GPIOA->AFR[1];
    temp &= 0xFFFF0FF0;    
    temp |= 0x00002002;    //PA11 -> AF2; PA8 -> AF2
    GPIOA->AFR[1] = temp;

    // Enable timer interrupt ver UDIS
    // TIM1->DIER |= TIM_DIER_UIE;
    // NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
    // NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 1);
    
    TIM1->CR1 |= TIM_CR1_CEN;
}


void TIM_3_Init (void)
{
    unsigned long temp;
    
    if (!RCC_TIM3_CLK)
        RCC_TIM3_CLK_ON;

    //Configuracion del timer.
    TIM3->CR1 = 0x00;		//clk int / 1; upcounting
    TIM3->CR2 = 0x00;		//igual al reset

    TIM3->SMCR |= TIM_SMCR_SMS_2;			//trigger: reset mode; link timer 1
    // TIM3->SMCR |= TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1;	//trigger: trigger mode; link timer 1    
    TIM3->CCMR1 = 0x6060;      //CH1, CH2 output PWM mode 1 (channel active TIM3->CNT < TIM3->CCR1)
    TIM3->CCMR2 = 0x6060;      //CH3, CH4 output PWM mode 1 (channel active TIM3->CNT < TIM3->CCR1)
    
    TIM3->CCER |= TIM_CCER_CC4E | TIM_CCER_CC3E |
        TIM_CCER_CC2E | TIM_CCER_CC1E;	//CH4 CH3 CH2 y CH1 enable on pin direct polarity

    TIM3->ARR = DUTY_100_PERCENT;        //tick cada 20.83us --> 48KHz
    TIM3->CNT = 0;

#if defined USE_FREQ_48KHZ
    TIM3->PSC = 0;
#elif defined USE_FREQ_24KHZ
    TIM3->PSC = 1;
#elif defined USE_FREQ_16KHZ
    TIM3->PSC = 2;
#else
#error "No FREQ selected for TIM3 on hard.h"
#endif
	
    //Alternate Fuction Pin Configurations
    temp = GPIOA->AFR[0];
    temp &= 0x00FFFFFF;	
    temp |= 0x11000000;    //PA7 -> AF1; PA6 -> AF1
    GPIOA->AFR[0] = temp;

    temp = GPIOB->AFR[0];
    temp &= 0xFFFFFF00;	
    temp |= 0x00000011;    //PB1 -> AF1; PB0 -> AF1
    GPIOB->AFR[0] = temp;
    
    // Enable timer ver UDIS
    //TIM3->DIER |= TIM_DIER_UIE;
    TIM3->CR1 |= TIM_CR1_CEN;    
}


void TIM3_IRQHandler (void)	//1 ms
{
	/*
	Usart_Time_1ms ();

	if (timer_1seg)
	{
		if (timer_1000)
			timer_1000--;
		else
		{
			timer_1seg--;
			timer_1000 = 1000;
		}
	}

	if (timer_led_comm)
		timer_led_comm--;

	if (timer_standby)
		timer_standby--;
	*/
	//bajar flag
	if (TIM3->SR & 0x01)	//bajo el flag
		TIM3->SR = 0x00;
}


void TIM14_IRQHandler (void)	//20us
{
    if (TIM14->SR & 0x01)
        TIM14->SR = 0x00;    //bajar flag

}


void TIM_14_Init (void)
{
    if (!RCC_TIM14_CLK)
        RCC_TIM14_CLK_ON;

    //Configuracion del timer.
    TIM14->CR1 = 0x00;		//clk int / 1; upcounting; uev
    TIM14->PSC = 47;		//tick cada 1us
    // TIM14->ARR = 20;	//int cada 20us
    TIM14->ARR = 20;	//int cada 10us para pruebas
    TIM14->EGR |= 0x0001;

    // Enable timer interrupt ver UDIS
    TIM14->DIER |= TIM_DIER_UIE;
    TIM14->CR1 |= TIM_CR1_URS | TIM_CR1_CEN;	//solo int cuando hay overflow y one shot

    NVIC_EnableIRQ(TIM14_IRQn);
    NVIC_SetPriority(TIM14_IRQn, 8);
    
}

// void TIM16_IRQHandler (void)	//100uS
// {
//
// 	if (TIM16->SR & 0x01)
// 		//bajar flag
// 		TIM16->SR = 0x00;
// }
//
//
// void TIM_16_Init (void)
// {
//
// 	NVIC_InitTypeDef NVIC_InitStructure;
//
// 	if (!RCC_TIM16_CLK)
// 		RCC_TIM16_CLK_ON;
//
// 	//Configuracion del timer.
// 	TIM16->ARR = 2000; //10m
// 	TIM16->CNT = 0;
// 	TIM16->PSC = 479;
// 	TIM16->EGR = TIM_EGR_UG;
//
// 	// Enable timer ver UDIS
// 	TIM16->DIER |= TIM_DIER_UIE;
// 	TIM16->CR1 |= TIM_CR1_CEN;
//
// 	NVIC_InitStructure.NVIC_IRQChannel = TIM16_IRQn;
// 	NVIC_InitStructure.NVIC_IRQChannelPriority = 5;
// 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
// 	NVIC_Init(&NVIC_InitStructure);
// }
//
// void TIM17_IRQHandler (void)	//100uS
// {
//
// 	//if (GPIOA_PIN0_OUT)
// 	//	GPIOA_PIN0_OFF;
// 	//else
// 	//	GPIOA_PIN0_ON;
//
// 	if (TIM17->SR & 0x01)
// 		//bajar flag
// 		TIM17->SR = 0x00;
// }
//
//
// void TIM_17_Init (void)
// {
//
// 	NVIC_InitTypeDef NVIC_InitStructure;
//
// 	if (!RCC_TIM17_CLK)
// 		RCC_TIM17_CLK_ON;
//
// 	//Configuracion del timer.
// 	TIM17->ARR = 2000; //10m
// 	TIM17->CNT = 0;
// 	TIM17->PSC = 479;
// 	TIM17->EGR = TIM_EGR_UG;
//
// 	// Enable timer ver UDIS
// 	TIM17->DIER |= TIM_DIER_UIE;
// 	TIM17->CR1 |= TIM_CR1_CEN;
//
// 	NVIC_InitStructure.NVIC_IRQChannel = TIM17_IRQn;
// 	NVIC_InitStructure.NVIC_IRQChannelPriority = 5;
// 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
// 	NVIC_Init(&NVIC_InitStructure);
// }

//--- end of file ---//
