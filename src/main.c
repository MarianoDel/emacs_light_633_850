//---------------------------------------------------
// #### PROYECTO LIGHT TREATMENT - Custom Board #####
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ##
// #### MAIN.C ######################################
//---------------------------------------------------

// Includes ------------------------------------------
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

#include <stdio.h>
// #include <string.h>


// External Variables --------------------------------
//-- ADC externals -----------------------------------
volatile unsigned short adc_ch [ADC_CHANNEL_QUANTITY];

//-- Timers externals --------------------------------
volatile unsigned char timer_1seg = 0;
volatile unsigned short timer_signals = 0;
volatile unsigned short timer_signals_gen = 0;
volatile unsigned short timer_led = 0;
volatile unsigned short timer_buzzer = 0;

//-- Usart externals ---------------------------------
volatile unsigned char usart1_have_data;


// Global Variables ----------------------------------
//-- Timers globals ----------------------------------
volatile unsigned short timer_standby;
volatile unsigned short wait_ms_var = 0;


// Module Functions Declaration ----------------------
void TimingDelay_Decrement(void);


// Module Functions Definition -----------------------
int main(void)
{
    unsigned short i = 0;

    //GPIO Configuration.
    GPIO_Config();

    //Systick Timer
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

    //Hard Test LED
    // while (1)
    // {
    //     if (LED)
    //         LED_OFF;
    //     else
    //         LED_ON;

    //     Wait_ms(1000);
    // }

    //Hard Test Buzzer
    // while (1)
    // {
    //     if (!timer_standby)
    //     {
    //         timer_standby = 5000;
    //         BuzzerCommands(BUZZER_SHORT_CMD, 2);
    //     }
        
    //     UpdateBuzzer();
    // }

    //Hard Test CTRL FAN
    CTRL_FAN1_OFF;
    CTRL_FAN2_OFF;
    CTRL_FAN3_OFF;
    CTRL_FAN4_OFF;
    while (1)
    {
        switch (i)
        {
        case 0:
            if (!timer_standby)
            {
                timer_standby = 5000;
                i++;
                CTRL_FAN1_ON;
                CTRL_FAN4_OFF;
            }
            break;
        case 1:
            if (!timer_standby)
            {
                timer_standby = 5000;
                i++;
                CTRL_FAN2_ON;
                CTRL_FAN1_OFF;
            }
            break;
        case 2:
            if (!timer_standby)
            {
                timer_standby = 5000;
                i++;
                CTRL_FAN3_ON;
                CTRL_FAN2_OFF;
            }
            break;
        case 3:
            if (!timer_standby)
            {
                timer_standby = 5000;
                i = 0;
                CTRL_FAN4_ON;
                CTRL_FAN3_OFF;
            }
            break;

        default:
            i = 0;
            break;
        }
    }


    //Hard Test Usart Tx & Rx
    USART1Config();
    char str [SIZEOF_RXDATA];
    while (1)
    {
        if (!timer_standby)
        {
            timer_standby = 2000;
            Usart1Send("Test Usart1\n");
        }

        if (usart1_have_data)
        {            
            usart1_have_data = 0;
            timer_standby = 2000;
            ReadUsart1Buffer((unsigned char *)str, SIZEOF_RXDATA);
            Usart1Send(str);
        }            
    }

    //Hard Test Channels
    TIM_3_Init();
    TIM_1_Init();
    CTRL_CH1(0);
    CTRL_CH2(0);
    CTRL_CH3(0);
    CTRL_CH4(0);
    CTRL_CH5(0);
    CTRL_CH6(0);

    while (1)
    {
        if (!timer_standby)
        {
            timer_standby = 2;
            if (i < DUTY_ALWAYS)
                i++;
            else
                i = 0;

            CTRL_CH1(i);
            CTRL_CH2(i);
            CTRL_CH3(i);
            CTRL_CH4(i);
            CTRL_CH5(i);
            CTRL_CH6(i);
        }
    }


    //Hard Test ADC
    // char str [SIZEOF_RXDATA];
    //Activo el ADC con DMA
    AdcConfig();

    //-- DMA configuration.
    DMAConfig();
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    ADC1->CR |= ADC_CR_ADSTART;

    i = 0;
    while (1)
    {
        if (sequence_ready)
        {
            sequence_ready_reset;
            i++;
        }

        if (i > 1000)
        {
            i = 0;
            sprintf(str, "Temp: %d, V12: %d, V24: %d\n",
                    LM335_VO,
                    V_Sense_12V,
                    V_Sense_24V);

            Usart1Send(str);
        }
    }




    //---------- END OF HARD TEST ----------//
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

