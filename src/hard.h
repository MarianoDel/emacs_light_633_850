//--------------------------------------------------
// #### PROYECTO LIGHT TREATMENT - Custom Board ####
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ##
// #### HARD.C #####################################
//--------------------------------------------------
#ifndef _HARD_H_
#define _HARD_H_

#include "stm32f0xx.h"


//-- Defines For Configuration -------------------
//---- Configuration for Hardware Versions -------
// #define HARDWARE_VERSION_1_1
#define HARDWARE_VERSION_1_0


// #define SOFTWARE_VERSION_1_1
#define SOFTWARE_VERSION_1_0


//---- Features Configuration ----------------
// #define USE_FREQ_48KHZ
// #define USE_FREQ_24KHZ
#define USE_FREQ_16KHZ

//------ Configuration for Firmware-Channels -----

//---- End of Features Configuration ----------


//--- Hardware Welcome Code ------------------//
#ifdef HARDWARE_VERSION_1_1
#define HARD "Hardware Version: 1.1\n"
#endif
#ifdef HARDWARE_VERSION_1_0
#define HARD "Hardware Version: 1.0\n"
#endif
#ifdef HARDWARE_VERSION_2_0
#define HARD "Hardware Version: 2.0\n"
#endif

//--- Software Welcome Code ------------------//
#ifdef SOFTWARE_VERSION_1_2
#define SOFT "Software Version: 1.2\n"
#endif
#ifdef SOFTWARE_VERSION_1_1
#define SOFT "Software Version: 1.1\n"
#endif
#ifdef SOFTWARE_VERSION_1_0
#define SOFT "Software Version: 1.0\n"
#endif

//-------- Configuration for Outputs-Channels -----


//---- Configuration for Firmware-Programs --------


//-------- Configuration for Outputs-Firmware ------


//-- End Of Defines For Configuration ---------------

//GPIOA pin0    V_Sense_24V analog input
//GPIOA pin1    V_Sense_12V analog input

//GPIOA pin2    NC
//GPIOA pin3    NC

//GPIOA pin4
#define EN_AUDIO ((GPIOA->ODR & 0x0010) != 0)
#define EN_AUDIO_ON GPIOA->BSRR = 0x00000010
#define EN_AUDIO_OFF GPIOA->BSRR = 0x00100000

//GPIOA pin5    LM335_VO analog input

//GPIOA pin6
//GPIOA pin7
//GPIOB pin0
//GPIOB pin1    TIM3 CH1 - CH4

//GPIOA pin8    TIM1 CH1

//GPIOA pin9    
//GPIOA pin10    USART1

//GPIOA pin11    TIM1 CH4

//GPIOA pin12
#define LED ((GPIOA->ODR & 0x1000) != 0)
#define LED_ON GPIOA->BSRR = 0x00001000
#define LED_OFF GPIOA->BSRR = 0x10000000

//GPIOA pin13
//GPIOA pin14    NC

//GPIOA pin15    
#define CTRL_FAN1 ((GPIOA->ODR & 0x8000) != 0)
#define CTRL_FAN1_ON GPIOA->BSRR = 0x00008000
#define CTRL_FAN1_OFF GPIOA->BSRR = 0x80000000

//GPIOB pin3     
#define CTRL_FAN2 ((GPIOB->ODR & 0x0008) != 0)
#define CTRL_FAN2_ON GPIOB->BSRR = 0x00000008
#define CTRL_FAN2_OFF GPIOB->BSRR = 0x00080000

//GPIOB pin4     
#define CTRL_FAN3 ((GPIOB->ODR & 0x0010) != 0)
#define CTRL_FAN3_ON GPIOB->BSRR = 0x00000010
#define CTRL_FAN3_OFF GPIOB->BSRR = 0x00100000

//GPIOB pin5     
#define CTRL_FAN4 ((GPIOB->ODR & 0x0020) != 0)
#define CTRL_FAN4_ON GPIOB->BSRR = 0x00000020
#define CTRL_FAN4_OFF GPIOB->BSRR = 0x00200000

//GPIOB pin6
#define BUZZER ((GPIOB->ODR & 0x0040) != 0)
#define BUZZER_ON GPIOB->BSRR = 0x00000040
#define BUZZER_OFF GPIOB->BSRR = 0x00400000

//GPIOB pin7    NC


//ESTADOS DEL LED
typedef enum
{    
    START_BLINKING = 0,
    WAIT_TO_OFF,
    WAIT_TO_ON,
    WAIT_NEW_CYCLE
} led_state_t;

//ESTADOS DEL BUZZER
typedef enum
{    
    BUZZER_INIT = 0,
    BUZZER_TO_STOP,

    BUZZER_MULTIPLE_LONG,
    BUZZER_MULTIPLE_LONGA,
    BUZZER_MULTIPLE_LONGB,

    BUZZER_MULTIPLE_HALF,
    BUZZER_MULTIPLE_HALFA,
    BUZZER_MULTIPLE_HALFB,

    BUZZER_MULTIPLE_SHORT,
    BUZZER_MULTIPLE_SHORTA,
    BUZZER_MULTIPLE_SHORTB
    
} buzzer_state_t;

//COMANDOS DEL BUZZER	(tienen que ser los del estado de arriba)
#define BUZZER_STOP_CMD		BUZZER_TO_STOP
#define BUZZER_LONG_CMD		BUZZER_MULTIPLE_LONG
#define BUZZER_HALF_CMD		BUZZER_MULTIPLE_HALF
#define BUZZER_SHORT_CMD	BUZZER_MULTIPLE_SHORT

#define TIM_BIP_SHORT		50
#define TIM_BIP_SHORT_WAIT	100
#define TIM_BIP_HALF		200
#define TIM_BIP_HALF_WAIT	500
#define TIM_BIP_LONG		1200
#define TIM_BIP_LONG_WAIT	1500


//Estados Externos de LED BLINKING
#define LED_NO_BLINKING    0
#define LED_TREATMENT_STANDBY    1
#define LED_TREATMENT_GENERATING    2
#define LED_TREATMENT_ERROR    6

//--- Stringtify Utils -----------------------
#define STRING_CONCAT(str1,str2) #str1 " " #str2
#define STRING_CONCAT_NEW_LINE(str1,str2) xstr(str1) #str2 "\n"
#define xstr_macro(s) str_macro(s)
#define str_macro(s) #s


/* Module Functions ------------------------------------------------------------*/
void ChangeLed (unsigned char);
void UpdateLed (void);
void UpdateBuzzer (void);
void BuzzerCommands(unsigned char, unsigned char);
void WelcomeCodeFeatures (void);

#endif /* _HARD_H_ */

//--- end of file ---//

