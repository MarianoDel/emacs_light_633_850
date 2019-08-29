//--------------------------------------------------
// #### PROYECTO LIGHT TREATMENT - Custom Board ####
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ##
// #### SIGNALS.C ##################################
//--------------------------------------------------

/* Includes ------------------------------------------------------------------*/
#include "signals.h"
#include "hard.h"
#include "stm32f0xx.h"
#include "tim.h"
#include "adc.h"
#include "dsp.h"
#include "uart.h"
#include "gpio.h"
#include "dma.h"

#include <stdio.h>


//--- VARIABLES EXTERNAS ---//
//del ADC
extern volatile unsigned short adc_ch[];

//del Main
extern volatile unsigned short timer_signals;
extern volatile unsigned short timer_signals_gen;

//--- VARIABLES GLOBALES ---//
treatment_t treatment_state = TREATMENT_INIT_FIRST_TIME;
signals_struct_t signal_to_gen;

//--- Para las Signals
cwave_state_t cwave_state = INIT_CWAVE;
pulsed_state_t pulsed_state = INIT_PULSED;
modulated_state_t modulated_state = INIT_MODULATED;
unsigned char modulated_index;

unsigned char global_error = 0;


//-- para determinacion de soft overcurrent ------------


//Signals Templates
#define I_MAX 151    //195 0.29A x 2.2ohms; lo entregado en el prototipo
                     //151 0.3A x 2.2ohms con leds Javi 19-07-2018
                 
#define I_MIN 15

const unsigned char v_triangular [] = {0,2,5,7,10,12,15,17,20,22,
                                     28,30,33,35,38,40,43,45,48,
                                     53,56,58,61,63,66,68,71,73,
                                     79,81,84,86,89,91,94,96,99,
                                     104,107,109,112,114,117,119,122,124,
                                     130,132,135,137,140,142,145,147,150,
                                     155,158,160,163,165,168,170,173,175,
                                     181,183,186,188,191,193,196,198,201,
                                     206,209,211,214,216,219,221,224,226,
                                     232,234,237,239,242,244,247,249,252,
                                     252,249,247,244,242,239,237,234,232,
                                     226,224,221,219,216,214,211,209,206,
                                     201,198,196,193,191,188,186,183,181,
                                     175,173,170,168,165,163,160,158,155,
                                     150,147,145,142,140,137,135,132,130,
                                     124,122,119,117,114,112,109,107,104,
                                     99,96,94,91,89,86,84,81,79,
                                     73,71,68,66,63,61,58,56,53,
                                     48,45,43,40,38,35,33,30,28,
                                     22,20,17,15,12,10,7,5,2};


//-- Module private Functions
unsigned short CalcPowerOffset(unsigned char);

//--- FUNCIONES DEL MODULO ---//
void TreatmentManager (void)
{
    switch (treatment_state)
    {
        case TREATMENT_INIT_FIRST_TIME:
            CTRL_CH1(DUTY_NONE);
            CTRL_CH2(DUTY_NONE);
            CTRL_CH3(DUTY_NONE);
            CTRL_CH4(DUTY_NONE);
            CTRL_CH5(DUTY_NONE);
            CTRL_CH6(DUTY_NONE);

            if (AssertTreatmentParams() == resp_ok)
            {
                treatment_state = TREATMENT_STANDBY;
                ChangeLed(LED_TREATMENT_STANDBY);
            }
            break;

        case TREATMENT_STANDBY:
            break;

        case TREATMENT_START_TO_GENERATE:    //reviso una vez mas los parametros y no tener ningun error
            if ((AssertTreatmentParams() == resp_ok) && (GetErrorStatus() == ERROR_OK))
            {
                if (signal_to_gen.signal == CWAVE_SIGNAL)
                {
                    cwave_state = INIT_CWAVE;
                    treatment_state = TREATMENT_GENERATING_CWAVE;
                }

                if (signal_to_gen.signal == PULSED_SIGNAL)
                {
                    pulsed_state = INIT_PULSED;
                    treatment_state = TREATMENT_GENERATING_PULSED;
                }

                if (signal_to_gen.signal == MODULATED_SIGNAL)
                {
                    modulated_state = INIT_MODULATED;
                    treatment_state = TREATMENT_GENERATING_MODULATED;
                }

                ChangeLed(LED_TREATMENT_GENERATING);
            }
            else
            {
                //error de parametros
                treatment_state = TREATMENT_INIT_FIRST_TIME;
                ChangeLed(LED_TREATMENT_ERROR);
            }
            break;

        case TREATMENT_GENERATING_CWAVE:
            //Cosas que dependen de las muestras
            //se la puede llamar las veces que sea necesario y entre funciones, para acelerar
            //la respuesta
            GenerateSignalCWave();
            break;

        case TREATMENT_GENERATING_PULSED:
            //Cosas que dependen de las muestras
            //se la puede llamar las veces que sea necesario y entre funciones, para acelerar
            //la respuesta
            GenerateSignalPulsed();
            break;

        case TREATMENT_GENERATING_MODULATED:
            //Cosas que dependen de las muestras
            //se la puede llamar las veces que sea necesario y entre funciones, para acelerar
            //la respuesta
            GenerateSignalModulated();
            break;
            
        case TREATMENT_STOPPING:
            CTRL_CH1(DUTY_NONE);
            CTRL_CH2(DUTY_NONE);
            CTRL_CH3(DUTY_NONE);
            CTRL_CH4(DUTY_NONE);
            CTRL_CH5(DUTY_NONE);
            CTRL_CH6(DUTY_NONE);

            timer_signals = 10;
            treatment_state = TREATMENT_STOPPING2;
            break;

        case TREATMENT_STOPPING2:
            if (!timer_signals)
                treatment_state = TREATMENT_INIT_FIRST_TIME;

            break;

        default:
            treatment_state = TREATMENT_INIT_FIRST_TIME;
            break;
    }
}

treatment_t GetTreatmentState (void)
{
    return treatment_state;
}

resp_t StartTreatment (void)
{
    if (treatment_state == TREATMENT_STANDBY)
    {
        if ((AssertTreatmentParams() == resp_ok) && (GetErrorStatus() == ERROR_OK))
        {
            treatment_state = TREATMENT_START_TO_GENERATE;
            return resp_ok;
        }
    }
    return resp_error;
}

void StopTreatment (void)
{
    if (treatment_state != TREATMENT_STANDBY)
        treatment_state = TREATMENT_STOPPING;
}

error_t GetErrorStatus (void)
{
	error_t error = ERROR_OK;

	if (global_error & ERROR_OVERTEMP_MASK)
		error = ERROR_OVERTEMP;
	else if (global_error & ERROR_OVERCURRENT_MASK)
		error = ERROR_OVERCURRENT;
	else if (global_error & ERROR_NO_CURRENT_MASK)
		error = ERROR_NO_CURRENT;
	else if (global_error & ERROR_SOFT_OVERCURRENT_MASK)
		error = ERROR_SOFT_OVERCURRENT;

	return error;
}

// void SetErrorStatus (error_t e)
// {
//     if (e == ERROR_FLUSH_MASK)
//         global_error = 0;
//     else
//     {
//         if (e == ERROR_OVERTEMP)
//             global_error |= ERROR_OVERTEMP_MASK;
//         if (e == ERROR_OVERCURRENT)
//             global_error |= ERROR_OVERCURRENT_MASK;
//         if (e == ERROR_SOFT_OVERCURRENT)
//             global_error |= ERROR_SOFT_OVERCURRENT_MASK;
//         if (e == ERROR_NO_CURRENT)
//             global_error |= ERROR_NO_CURRENT_MASK;
//     }
// }

// //TODO: PONER UNA TRABA DE SETEOS PARANO CAMBIAR NADA CORRIENDO

resp_t SetSignalType (signal_type_t a)
{
    //TODO: despues cargar directamente los k
    if ((treatment_state != TREATMENT_INIT_FIRST_TIME) && (treatment_state != TREATMENT_STANDBY))
        return resp_error;

    signal_to_gen.signal = a;

    return resp_ok;
}

resp_t SetFrequency (unsigned char a)
{
    if ((a >= 0) && (a < 10))
        signal_to_gen.frequency = a;

    return resp_ok;
}

resp_t SetPower (unsigned char ch, unsigned char a)
{
    if (ch == 0x0F)
    {              
        signal_to_gen.ch1_power = a;
        signal_to_gen.ch2_power = a;
        signal_to_gen.ch3_power = a;
        signal_to_gen.ch4_power = a;
        signal_to_gen.ch5_power = a;
        signal_to_gen.ch6_power = a;
    }
    else
    {
        if (ch == 1)
            signal_to_gen.ch1_power = a;

        if (ch == 2)
            signal_to_gen.ch2_power = a;

        if (ch == 3)
            signal_to_gen.ch3_power = a;

        if (ch == 4)
            signal_to_gen.ch4_power = a;

        if (ch == 5)
            signal_to_gen.ch5_power = a;

        if (ch == 6)
            signal_to_gen.ch6_power = a;

    }

    return resp_ok;
}

//verifica que se cumplan con todos los parametros para poder enviar una senial coherente
resp_t AssertTreatmentParams (void)
{
    resp_t resp = resp_error;

    if (signal_to_gen.frequency > 9)
        return resp;

    if ((signal_to_gen.signal != CWAVE_SIGNAL) &&
        (signal_to_gen.signal != PULSED_SIGNAL) &&
        (signal_to_gen.signal != MODULATED_SIGNAL))
        return resp;

    return resp_ok;
}

void SendAllConf (void)
{
    char b [64];
    sprintf(b, "signal: %d\n", signal_to_gen.signal);
    Usart1Send(b);
    sprintf(b, "freq: %d\n", signal_to_gen.frequency);
    Usart1Send(b);
    sprintf(b, "ch power led: %d, %d, %d, %d, %d, %d\n",
            signal_to_gen.ch1_power,
            signal_to_gen.ch2_power,
            signal_to_gen.ch3_power,
            signal_to_gen.ch4_power,
            signal_to_gen.ch5_power,
            signal_to_gen.ch6_power);
    
    Usart1Send(b);
    Usart1Send("\n");
}

//la llama el manager para generar las seniales CWAVE en los canales
void GenerateSignalCWave (void)
{
    switch (cwave_state)
    {
        case INIT_CWAVE:

            cwave_state = UPDATE_POWER_CWAVE;
            break;

        case UPDATE_POWER_CWAVE:
            //hago el update de la potencia cada 1 segundo
            CTRL_CH1(CalcPowerOffset(signal_to_gen.ch1_power));
            CTRL_CH2(CalcPowerOffset(signal_to_gen.ch2_power));
            CTRL_CH3(CalcPowerOffset(signal_to_gen.ch3_power));
            CTRL_CH4(CalcPowerOffset(signal_to_gen.ch4_power));
            CTRL_CH5(CalcPowerOffset(signal_to_gen.ch5_power));
            CTRL_CH6(CalcPowerOffset(signal_to_gen.ch6_power));

            cwave_state = GEN_CWAVE;
            timer_signals_gen = 1000;    //cada 1 seg reviso potencias de los lasers            
            break;
            
        case GEN_CWAVE:
            //secuencia de lasers
            if (!timer_signals_gen)
                cwave_state = UPDATE_POWER_CWAVE;

            break;

        default:
            //si me llaman y estoy en cualquiera igual genero
            cwave_state = INIT_CWAVE;
            break;
            
    }
}

//la llama el manager para generar las seniales PULSED en los canales
//dependen de la freq
void GenerateSignalPulsed (void)
{
    switch (pulsed_state)
    {
        case INIT_PULSED:

            CTRL_CH1(CalcPowerOffset(signal_to_gen.ch1_power));
            CTRL_CH2(CalcPowerOffset(signal_to_gen.ch2_power));
            CTRL_CH3(CalcPowerOffset(signal_to_gen.ch3_power));
            CTRL_CH4(CalcPowerOffset(signal_to_gen.ch4_power));
            CTRL_CH5(CalcPowerOffset(signal_to_gen.ch5_power));
            CTRL_CH6(CalcPowerOffset(signal_to_gen.ch6_power));
            
            if (signal_to_gen.frequency == 0)
                timer_signals_gen = 50;
            else
                timer_signals_gen = 1000 / (signal_to_gen.frequency * 2);

            pulsed_state = GEN_PULSED;            
            break;

        case GEN_PULSED:
            if (!timer_signals_gen)
            {
                CTRL_CH1(DUTY_NONE);
                CTRL_CH2(DUTY_NONE);
                CTRL_CH3(DUTY_NONE);
                CTRL_CH4(DUTY_NONE);
                CTRL_CH5(DUTY_NONE);
                CTRL_CH6(DUTY_NONE);
                
                if (signal_to_gen.frequency == 0)
                    timer_signals_gen = 50;
                else
                    timer_signals_gen = 1000 / (signal_to_gen.frequency * 2);

                pulsed_state = NO_GEN_PULSED;
            }
            break;

        case NO_GEN_PULSED:
            if (!timer_signals_gen)
                pulsed_state = INIT_PULSED;
            
            break;
        
        default:
            //si me llaman y estoy en cualquiera igual genero
            pulsed_state = INIT_PULSED;
            break;            
    }
}

//la llama el manager para generar las seniales MODULATED en los canales
//dependen de la freq
void GenerateSignalModulated (void)
{
    unsigned short dummy, dummy2;
    
    switch (modulated_state)
    {
        case INIT_MODULATED:
            CTRL_CH1(DUTY_NONE);
            CTRL_CH2(DUTY_NONE);
            CTRL_CH3(DUTY_NONE);
            CTRL_CH4(DUTY_NONE);
            CTRL_CH5(DUTY_NONE);
            CTRL_CH6(DUTY_NONE);

            modulated_index = 0;
            timer_signals_gen = 5;
            modulated_state = GEN_MODULATION;            
            break;
           
        case GEN_MODULATION:
            if (!timer_signals_gen)
            {
                if (signal_to_gen.frequency == 0)
                    modulated_index += 10;
                else
                    modulated_index += signal_to_gen.frequency;

                if (modulated_index < sizeof(v_triangular))
                {
                    dummy = v_triangular[modulated_index];

                    //Update Channels Power
                    dummy2 = signal_to_gen.ch1_power * dummy;
                    dummy2 >>= 8;
                    CTRL_CH1(CalcPowerOffset(dummy2));

                    dummy2 = signal_to_gen.ch2_power * dummy;
                    dummy2 >>= 8;
                    CTRL_CH2(dummy2);
                    
                    dummy2 = signal_to_gen.ch3_power * dummy;
                    dummy2 >>= 8;
                    CTRL_CH3(CalcPowerOffset(dummy2));

                    dummy2 = signal_to_gen.ch4_power * dummy;
                    dummy2 >>= 8;
                    CTRL_CH4(CalcPowerOffset(dummy2));

                    dummy2 = signal_to_gen.ch5_power * dummy;
                    dummy2 >>= 8;
                    CTRL_CH5(CalcPowerOffset(dummy2));

                    dummy2 = signal_to_gen.ch6_power * dummy;
                    dummy2 >>= 8;
                    CTRL_CH6(CalcPowerOffset(dummy2));

                    timer_signals_gen = 5;
                }
                else
                    modulated_state = INIT_MODULATED; 

            }

            break;

        default:
            //si me llaman y estoy en cualquiera igual genero
            modulated_state = INIT_MODULATED;
            break;            
    }
}

unsigned short CalcPowerOffset(unsigned char p)
{
    unsigned short dummy = 0;
    
    dummy = p * 10;
    if (dummy > DUTY_MAX)
        dummy = DUTY_MAX;

    return dummy;
}

//--- end of file ---//
