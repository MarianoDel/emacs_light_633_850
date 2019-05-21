//---------------------------------------------------
// #### PROYnECTO LIGHT TREATMENT - Custom Board ####
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ##
// #### MAIN.C ######################################
//---------------------------------------------------

/* Includes ------------------------------------------------------------------*/
#include "hard.h"
#include "stm32f0xx.h"
#include "gpio.h"
#include "uart.h"

#include "core_cm0.h"
#include "adc.h"
#include "tim.h"

#include "comm.h"
#include "signals.h"
#include "dma.h"

//#include <stdio.h>
//#include <string.h>




//--- VARIABLES EXTERNAS ---//
// ------- Externals del ADC -------
volatile unsigned short adc_ch [ADC_CHANNEL_QUANTITY];

// ------- Externals de los timers -------
volatile unsigned char timer_1seg = 0;
volatile unsigned short timer_signals = 0;
volatile unsigned short timer_signals_gen = 0;
volatile unsigned short timer_led = 0;
volatile unsigned short timer_buzzer = 0;

// ------- Externals del USART -------
volatile unsigned char usart1_have_data;


//--- VARIABLES GLOBALES ---//


// ------- de los timers -------
volatile unsigned short timer_standby;
volatile unsigned short wait_ms_var = 0;

//--- FUNCIONES DEL MODULO ---//
void TimingDelay_Decrement(void);


// ------- del DMX -------
// extern void EXTI4_15_IRQHandler(void);


//-------------------------------------------//
// @brief  Main program.
// @param  None
// @retval None
//------------------------------------------//
int main(void)
{
    unsigned short i = 0;
    // char s_to_senda [100];    
    // unsigned char bytes_readed = 0;

    //GPIO Configuration.
    GPIO_Config();

    //TIM Configuration.
    // TIM_3_Init();
    // TIM_14_Init();

    //ACTIVAR SYSTICK TIMER
    if (SysTick_Config(48000))
    {
        while (1)	/* Capture error */
        {
            if (LED)
                LED_OFF;
            else
                LED_ON;

            for (i = 0; i < 255; i++)
            {
                asm (	"nop \n\t"
                        "nop \n\t"
                        "nop \n\t" );
            }
        }
    }


    //prueba modulo signals.c comm.c tim.c adc.c
    TIM_3_Init();
    CTRL_CH1(0);
    CTRL_CH2(0);
    CTRL_CH3(0);
    CTRL_CH4(0);

    //Activo el ADC con DMA
    AdcConfig();

    //-- DMA configuration.
    DMAConfig();
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    ADC1->CR |= ADC_CR_ADSTART;


    USART1Config();

    //--- Mensaje Bienvenida ---//
    //---- Defines from hard.h -----//
    Wait_ms(1000);
    Usart1Send("\nLight Treatment -- powered by: Kirno Technology\n");
    Wait_ms(100);
    Usart1Send("First Power Board\n\n");
    
#ifdef HARD
    Usart1Send(HARD);
    Wait_ms(100);    
#else
#error	"No Hardware defined in hard.h file"
#endif

#ifdef SOFT
    Usart1Send(SOFT);
    Wait_ms(100);    
#else
#error	"No Soft Version defined in hard.h file"
#endif

    WelcomeCodeFeatures();
    
//---- End of Defines from hard.h -----//
    
    while (1)
    {        
        // TreatmentManager();
        UpdateCommunications();
        UpdateLed();
        UpdateBuzzer();
    }

    return 0;
}
//--- End of Main ---//

void TimingDelay_Decrement(void)
{
    if (wait_ms_var)
        wait_ms_var--;

    if (timer_standby)
        timer_standby--;

    if (timer_signals)
        timer_signals--;

    if (timer_signals_gen)
        timer_signals_gen--;

    if (timer_led)
        timer_led--;

    if (timer_buzzer)
        timer_buzzer--;

}

//--- end of file ---//

