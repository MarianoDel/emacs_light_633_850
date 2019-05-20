//---------------------------------------------
// #### PROYECTO LIPO LASER - Custom Board ####
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ##
// #### SIGNALS.C #############################
//---------------------------------------------

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
//valores globales para los 4 PIDs
short d_ch1;
short d_ch2;
short d_ch3;
short d_ch4;

short e_z1_ch1;
short e_z1_ch2;
short e_z1_ch3;
short e_z1_ch4;

short e_z2_ch1;
short e_z2_ch2;
short e_z2_ch3;
short e_z2_ch4;

unsigned char undersampling;
unsigned short mod_SP_ch1;
unsigned short mod_SP_ch2;
unsigned short mod_SP_ch3;
unsigned short mod_SP_ch4;

//-- para determinacion de soft overcurrent ------------
unsigned short soft_overcurrent_max_current_in_cycles [SIZEOF_OVERCURRENT_BUFF];
unsigned short soft_overcurrent_treshold = 0;
unsigned short soft_overcurrent_index = 0;


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


//--- FUNCIONES DEL MODULO ---//
void TreatmentManager (void)
{
    switch (treatment_state)
    {
        case TREATMENT_INIT_FIRST_TIME:
            UpdateLaserCh1(0);
            UpdateLaserCh2(0);
            UpdateLaserCh3(0);
            UpdateLaserCh4(0);

            Update_TIM3_CH1(0);
            Update_TIM3_CH2(0);
            Update_TIM3_CH3(0);
            Update_TIM3_CH4(0);

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
#ifdef USE_SOFT_OVERCURRENT
                //cargo valor maximo de corriente para el soft_overcurrent
                soft_overcurrent_treshold = 1.2 * I_MAX;
                soft_overcurrent_index = 0;

                for (unsigned char i = 0; i < SIZEOF_OVERCURRENT_BUFF; i++)
                    soft_overcurrent_max_current_in_cycles[i] = 0;
#endif
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

#ifdef USE_SOFT_OVERCURRENT
            //TODO: poner algun synchro con muestras para que no ejecute el filtro todo el tiempo
            //soft current overload check
            // if (MAFilter8 (soft_overcurrent_max_current_in_cycles) > soft_overcurrent_treshold)
            // {
            //     treatment_state = TREATMENT_STOPPING;
            //     SetErrorStatus(ERROR_SOFT_OVERCURRENT);
            // }
#endif
            break;

        case TREATMENT_GENERATING_PULSED:
            //Cosas que dependen de las muestras
            //se la puede llamar las veces que sea necesario y entre funciones, para acelerar
            //la respuesta
            GenerateSignalPulsed();

#ifdef USE_SOFT_OVERCURRENT
            //TODO: poner algun synchro con muestras para que no ejecute el filtro todo el tiempo
            //soft current overload check
            // if (MAFilter8 (soft_overcurrent_max_current_in_cycles) > soft_overcurrent_treshold)
            // {
            //     treatment_state = TREATMENT_STOPPING;
            //     SetErrorStatus(ERROR_SOFT_OVERCURRENT);
            // }
#endif
            break;

        case TREATMENT_GENERATING_MODULATED:
            //Cosas que dependen de las muestras
            //se la puede llamar las veces que sea necesario y entre funciones, para acelerar
            //la respuesta
            GenerateSignalModulated();

#ifdef USE_SOFT_OVERCURRENT
            //TODO: poner algun synchro con muestras para que no ejecute el filtro todo el tiempo
            //soft current overload check
            // if (MAFilter8 (soft_overcurrent_max_current_in_cycles) > soft_overcurrent_treshold)
            // {
            //     treatment_state = TREATMENT_STOPPING;
            //     SetErrorStatus(ERROR_SOFT_OVERCURRENT);
            // }
#endif
            break;
            
        case TREATMENT_STOPPING:
            Update_TIM3_CH1(0);
            Update_TIM3_CH2(0);
            Update_TIM3_CH3(0);
            Update_TIM3_CH4(0);

            UpdateLaserCh1(0);
            UpdateLaserCh2(0);
            UpdateLaserCh3(0);
            UpdateLaserCh4(0);
            
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

resp_t SetPowerLed (unsigned char ch, unsigned char a)
{
    if (ch == 0x0F)
    {              
        signal_to_gen.ch1_power_led = a;
        signal_to_gen.ch2_power_led = a;
        signal_to_gen.ch3_power_led = a;
        signal_to_gen.ch4_power_led = a;
    }
    else
    {
        if (ch == 1)
            signal_to_gen.ch1_power_led = a;

        if (ch == 2)
            signal_to_gen.ch2_power_led = a;

        if (ch == 3)
            signal_to_gen.ch3_power_led = a;

        if (ch == 4)
            signal_to_gen.ch4_power_led = a;
    }
    
    return resp_ok;
}

resp_t SetPowerLaser (unsigned char ch, unsigned char a)
{
    if (ch == 0x0F)
    {              
        signal_to_gen.ch1_power_laser = a;
        signal_to_gen.ch2_power_laser = a;
        signal_to_gen.ch3_power_laser = a;
        signal_to_gen.ch4_power_laser = a;
    }
    else
    {
        if (ch == 1)
            signal_to_gen.ch1_power_laser = a;

        if (ch == 2)
            signal_to_gen.ch2_power_laser = a;

        if (ch == 3)
            signal_to_gen.ch3_power_laser = a;

        if (ch == 4)
            signal_to_gen.ch4_power_laser = a;
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
    sprintf(b, "ch power led: %d, %d, %d, %d\n",
            signal_to_gen.ch1_power_led,
            signal_to_gen.ch2_power_led,
            signal_to_gen.ch3_power_led,
            signal_to_gen.ch4_power_led);
    
    Usart1Send(b);
    sprintf(b, "ch power laser: %d, %d, %d, %d\n",
            signal_to_gen.ch1_power_laser,
            signal_to_gen.ch2_power_laser,
            signal_to_gen.ch3_power_laser,
            signal_to_gen.ch4_power_laser);

    Usart1Send(b);
    Usart1Send("\n");
}

//la llama el manager para generar las seniales CWAVE en los canales
void GenerateSignalCWave (void)
{
    unsigned short dummy;
    
    switch (cwave_state)
    {
        case INIT_CWAVE:

            undersampling = UNDERSAMPLING_TICKS;    //para que arranque pid
            d_ch1 = 0;
            e_z1_ch1 = 0;
            e_z2_ch1 = 0;

            d_ch2 = 0;
            e_z1_ch2 = 0;
            e_z2_ch2 = 0;

            d_ch3 = 0;
            e_z1_ch3 = 0;
            e_z2_ch3 = 0;

            d_ch4 = 0;
            e_z1_ch4 = 0;
            e_z2_ch4 = 0;

            cwave_state = UPDATE_POWER_CWAVE;
            break;

        case UPDATE_POWER_CWAVE:
            //la potencia de los leds entra sola en el loop
            //la de los lasers hago aca el update
            UpdateLaserCh1(signal_to_gen.ch1_power_laser);
            UpdateLaserCh2(signal_to_gen.ch2_power_laser);
            UpdateLaserCh3(signal_to_gen.ch3_power_laser);
            UpdateLaserCh4(signal_to_gen.ch4_power_laser);

            cwave_state = GEN_CWAVE;
            timer_signals_gen = 1000;    //cada 1 seg reviso potencias de los lasers            
            break;
            
        case GEN_CWAVE:
            //secuencia de lasers
            if (!timer_signals_gen)
                cwave_state = UPDATE_POWER_CWAVE;

            //secuencia de leds
            if (sequence_ready)
            {
                sequence_ready_reset;
                if (undersampling < UNDERSAMPLING_TICKS)
                    undersampling++;
                else
                {
                    //PID CH1
                    if (signal_to_gen.ch1_power_led >= I_MIN)
                    {                        
                        dummy = signal_to_gen.ch1_power_led * I_MAX;
                        dummy >>= 8;
                        d_ch1 = PID_roof (dummy, I_Sense_Ch1, d_ch1, &e_z1_ch1, &e_z2_ch1);
                    }
                    else
                        d_ch1 = 0;

                    if (d_ch1 < 0)
                        d_ch1 = 0;
                    else
                    {
                        if (d_ch1 > DUTY_70_PERCENT)
                            d_ch1 = DUTY_70_PERCENT;

                        Update_TIM3_CH1(d_ch1);
                    }

                    //PID CH2
                    if (signal_to_gen.ch2_power_led >= I_MIN)
                    {                        
                        dummy = signal_to_gen.ch2_power_led * I_MAX;
                        dummy >>= 8;
                        d_ch2 = PID_roof (dummy, I_Sense_Ch2, d_ch2, &e_z1_ch2, &e_z2_ch2);
                    }
                    else
                        d_ch2 = 0;

                    if (d_ch2 < 0)
                        d_ch2 = 0;
                    else
                    {
                        if (d_ch2 > DUTY_70_PERCENT)
                            d_ch2 = DUTY_70_PERCENT;

                        Update_TIM3_CH2(d_ch2);
                    }

                    //PID CH3
                    if (signal_to_gen.ch3_power_led >= I_MIN)
                    {                                            
                        dummy = signal_to_gen.ch3_power_led * I_MAX;
                        dummy >>= 8;
                        d_ch3 = PID_roof (dummy, I_Sense_Ch3, d_ch3, &e_z1_ch3, &e_z2_ch3);
                    }
                    else
                        d_ch3 = 0;

                    if (d_ch3 < 0)
                        d_ch3 = 0;
                    else
                    {
                        if (d_ch3 > DUTY_70_PERCENT)
                            d_ch3 = DUTY_70_PERCENT;

                        Update_TIM3_CH3(d_ch3);
                    }

                    //PID CH4
                    if (signal_to_gen.ch4_power_led >= I_MIN)
                    {                        
                        dummy = signal_to_gen.ch4_power_led * I_MAX;
                        dummy >>= 8;
                        d_ch4 = PID_roof (dummy, I_Sense_Ch4, d_ch4, &e_z1_ch4, &e_z2_ch4);
                    }
                    else
                        d_ch4 = 0;

                    if (d_ch4 < 0)
                        d_ch4 = 0;
                    else
                    {
                        if (d_ch4 > DUTY_70_PERCENT)
                            d_ch4 = DUTY_70_PERCENT;

                        Update_TIM3_CH4(d_ch4);
                    }
                }    //end undersampling
            }    //end sequence ready
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
    unsigned short dummy;
    
    switch (pulsed_state)
    {
        case INIT_PULSED:
            //por ahora solo laser
            UpdateLaserCh1(signal_to_gen.ch1_power_laser);

            UpdateLaserCh2(signal_to_gen.ch2_power_laser);

            UpdateLaserCh3(signal_to_gen.ch3_power_laser);

            UpdateLaserCh4(signal_to_gen.ch4_power_laser);

            undersampling = UNDERSAMPLING_TICKS;    //para que arranque pid
            d_ch1 = 0;
            e_z1_ch1 = 0;
            e_z2_ch1 = 0;

            d_ch2 = 0;
            e_z1_ch2 = 0;
            e_z2_ch2 = 0;

            d_ch3 = 0;
            e_z1_ch3 = 0;
            e_z2_ch3 = 0;

            d_ch4 = 0;
            e_z1_ch4 = 0;
            e_z2_ch4 = 0;
            
            if (signal_to_gen.frequency == 0)
                timer_signals_gen = 50;
            else
                timer_signals_gen = 1000 / (signal_to_gen.frequency * 2);

            pulsed_state = GEN_PULSED;            
            break;

        case GEN_PULSED:
            if (!timer_signals_gen)
            {
                UpdateLaserCh1(0);
                UpdateLaserCh2(0);
                UpdateLaserCh3(0);
                UpdateLaserCh4(0);

                Update_TIM3_CH1(0);
                Update_TIM3_CH2(0);
                Update_TIM3_CH3(0);
                Update_TIM3_CH4(0);
                
                if (signal_to_gen.frequency == 0)
                    timer_signals_gen = 50;
                else
                    timer_signals_gen = 1000 / (signal_to_gen.frequency * 2);

                pulsed_state = NO_GEN_PULSED;
            }
            else
            {
                //secuencia de leds
                if (sequence_ready)
                {
                    sequence_ready_reset;
                    if (undersampling < UNDERSAMPLING_TICKS)
                        undersampling++;
                    else
                    {
                        //PID CH1
                        if (signal_to_gen.ch1_power_led >= I_MIN)
                        {                        
                            dummy = signal_to_gen.ch1_power_led * I_MAX;
                            dummy >>= 8;
                            d_ch1 = PID_roof (dummy, I_Sense_Ch1, d_ch1, &e_z1_ch1, &e_z2_ch1);
                        }
                        else
                            d_ch1 = 0;

                        if (d_ch1 < 0)
                            d_ch1 = 0;
                        else
                        {
                            if (d_ch1 > DUTY_70_PERCENT)
                                d_ch1 = DUTY_70_PERCENT;

                            Update_TIM3_CH1(d_ch1);
                        }

                        //PID CH2
                        if (signal_to_gen.ch2_power_led >= I_MIN)
                        {                        
                            dummy = signal_to_gen.ch2_power_led * I_MAX;
                            dummy >>= 8;
                            d_ch2 = PID_roof (dummy, I_Sense_Ch2, d_ch2, &e_z1_ch2, &e_z2_ch2);
                        }
                        else
                            d_ch2 = 0;

                        if (d_ch2 < 0)
                            d_ch2 = 0;
                        else
                        {
                            if (d_ch2 > DUTY_70_PERCENT)
                                d_ch2 = DUTY_70_PERCENT;

                            Update_TIM3_CH2(d_ch2);
                        }

                        //PID CH3
                        if (signal_to_gen.ch3_power_led >= I_MIN)
                        {                        
                            dummy = signal_to_gen.ch3_power_led * I_MAX;
                            dummy >>= 8;
                            d_ch3 = PID_roof (dummy, I_Sense_Ch3, d_ch3, &e_z1_ch3, &e_z2_ch3);
                        }
                        else
                            d_ch3 = 0;

                        if (d_ch3 < 0)
                            d_ch3 = 0;
                        else
                        {
                            if (d_ch3 > DUTY_70_PERCENT)
                                d_ch3 = DUTY_70_PERCENT;

                            Update_TIM3_CH3(d_ch3);
                        }

                        //PID CH4
                        if (signal_to_gen.ch4_power_led >= I_MIN)
                        {                        
                            dummy = signal_to_gen.ch4_power_led * I_MAX;
                            dummy >>= 8;
                            d_ch4 = PID_roof (dummy, I_Sense_Ch4, d_ch4, &e_z1_ch4, &e_z2_ch4);
                        }
                        else
                            d_ch4 = 0;

                        if (d_ch4 < 0)
                            d_ch4 = 0;
                        else
                        {
                            if (d_ch4 > DUTY_70_PERCENT)
                                d_ch4 = DUTY_70_PERCENT;

                            Update_TIM3_CH4(d_ch4);
                        }
                    }    //end of undersampling
                }    //end of secuence ready
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
            UpdateLaserCh1(0);
            UpdateLaserCh2(0);
            UpdateLaserCh3(0);
            UpdateLaserCh4(0);

            undersampling = UNDERSAMPLING_TICKS;    //para que arranque pid
            d_ch1 = 0;
            e_z1_ch1 = 0;
            e_z2_ch1 = 0;

            d_ch2 = 0;
            e_z1_ch2 = 0;
            e_z2_ch2 = 0;

            d_ch3 = 0;
            e_z1_ch3 = 0;
            e_z2_ch3 = 0;

            d_ch4 = 0;
            e_z1_ch4 = 0;
            e_z2_ch4 = 0;            

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

                    //Update Laser Channels
                    dummy2 = signal_to_gen.ch1_power_laser * dummy;
                    dummy2 >>= 8;
                    UpdateLaserCh1(dummy2);

                    dummy2 = signal_to_gen.ch2_power_laser * dummy;
                    dummy2 >>= 8;
                    UpdateLaserCh2(dummy2);
                    
                    dummy2 = signal_to_gen.ch3_power_laser * dummy;
                    dummy2 >>= 8;
                    UpdateLaserCh3(dummy2);

                    dummy2 = signal_to_gen.ch4_power_laser * dummy;
                    dummy2 >>= 8;
                    UpdateLaserCh4(dummy2);

                    //Update LEDs, dummy ya cargado
                    dummy2 = signal_to_gen.ch1_power_led * dummy;
                    dummy2 >>= 8;
                    dummy2 = dummy2 * I_MAX;
                    dummy2 >>= 8;
                    mod_SP_ch1 = dummy2;

                    dummy2 = signal_to_gen.ch2_power_led * dummy;
                    dummy2 >>= 8;
                    dummy2 = dummy2 * I_MAX;
                    dummy2 >>= 8;                    
                    mod_SP_ch2 = dummy2;

                    dummy2 = signal_to_gen.ch3_power_led * dummy;
                    dummy2 >>= 8;
                    dummy2 = dummy2 * I_MAX;
                    dummy2 >>= 8;                    
                    mod_SP_ch3 = dummy2;

                    dummy2 = signal_to_gen.ch4_power_led * dummy;
                    dummy2 >>= 8;
                    dummy2 = dummy2 * I_MAX;
                    dummy2 >>= 8;                    
                    mod_SP_ch4 = dummy2;                    

                    timer_signals_gen = 5;
                }
                else
                    modulated_state = INIT_MODULATED; 

            }

            //para los led - secuencia de leds
            if (sequence_ready)
            {
                sequence_ready_reset;
                if (undersampling < UNDERSAMPLING_TICKS)
                    undersampling++;
                else
                {
                    //PID CH1, el setpoint se actualiza arriba
                    if (signal_to_gen.ch1_power_led >= I_MIN)
                    {                        
                        d_ch1 = PID_roof (mod_SP_ch1, I_Sense_Ch1, d_ch1, &e_z1_ch1, &e_z2_ch1);
                    }
                    else
                        d_ch1 = 0;

                    if (d_ch1 < 0)
                        d_ch1 = 0;
                    else
                    {
                        if (d_ch1 > DUTY_70_PERCENT)
                            d_ch1 = DUTY_70_PERCENT;

                        Update_TIM3_CH1(d_ch1);
                    }

                    //PID CH2, el setpoint se actualiza arriba
                    if (signal_to_gen.ch2_power_led >= I_MIN)
                    {                        
                        d_ch2 = PID_roof (mod_SP_ch2, I_Sense_Ch2, d_ch2, &e_z1_ch2, &e_z2_ch2);
                    }
                    else
                        d_ch2 = 0;

                    if (d_ch2 < 0)
                        d_ch2 = 0;
                    else
                    {
                        if (d_ch2 > DUTY_70_PERCENT)
                            d_ch2 = DUTY_70_PERCENT;

                        Update_TIM3_CH2(d_ch2);
                    }

                    //PID CH3, el setpoint se actualiza arriba
                    if (signal_to_gen.ch3_power_led >= I_MIN)
                    {                        
                        d_ch3 = PID_roof (mod_SP_ch3, I_Sense_Ch3, d_ch3, &e_z1_ch3, &e_z2_ch3);
                    }
                    else
                        d_ch3 = 0;

                    if (d_ch3 < 0)
                        d_ch3 = 0;
                    else
                    {
                        if (d_ch3 > DUTY_70_PERCENT)
                            d_ch3 = DUTY_70_PERCENT;

                        Update_TIM3_CH3(d_ch3);
                    }

                    //PID CH4, el setpoint se actualiza arriba
                    if (signal_to_gen.ch4_power_led >= I_MIN)
                    {                        
                        d_ch4 = PID_roof (mod_SP_ch4, I_Sense_Ch4, d_ch4, &e_z1_ch4, &e_z2_ch4);
                    }
                    else
                        d_ch4 = 0;

                    if (d_ch4 < 0)
                        d_ch4 = 0;
                    else
                    {
                        if (d_ch4 > DUTY_70_PERCENT)
                            d_ch4 = DUTY_70_PERCENT;

                        Update_TIM3_CH4(d_ch4);
                    }
                }    //end of undersampling
            }    //end of secuence ready

            break;

        default:
            //si me llaman y estoy en cualquiera igual genero
            modulated_state = INIT_MODULATED;
            break;            
    }
}


//--- end of file ---//
