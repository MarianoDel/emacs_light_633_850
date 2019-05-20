//---------------------------------------------
// #### PROYECTO LIPO LASER - Custom Board ####
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ##
// #### COMM.C ################################
//---------------------------------------------

/* Includes ------------------------------------------------------------------*/
#include "comm.h"
#include "signals.h"
#include "uart.h"
#include "hard.h"

#include "utils.h"

#include <string.h>
#include <stdio.h>



//--- VARIABLES EXTERNAS ---//
//del main
extern const char s_ok [];
// ------- Externals del Puerto serie  -------
extern unsigned char usart1_have_data;

//--- VARIABLES GLOBALES ---//
//globales de este modulo

//strings de comienzo o lineas intermedias
//--- Direccionamiento
#ifdef FIRST_POWER_BOARD
const char s_ch1 [] = {"ch1"};
const char s_ch2 [] = {"ch2"};
const char s_ch3 [] = {"ch3"};
const char s_ch4 [] = {"ch4"};
#endif
#ifdef SECOND_POWER_BOARD
const char s_ch1 [] = {"ch5"};
const char s_ch2 [] = {"ch6"};
const char s_ch3 [] = {"ch7"};
const char s_ch4 [] = {"ch8"};
#endif
const char s_chf [] = {"chf"};
//--- Available Settings
const char s_set_signal [] = {"signal"};
const char s_frequency [] = {"frequency"};
const char s_power_led [] = {"power led"};
const char s_power_laser [] = {"power laser"};
//--- Available Signals
const char s_cwave [] = {"cwave"};
const char s_pulsed [] = {"pulsed"};
const char s_modulated [] = {"modulated"};
//--- Manager
const char s_start_treatment [] = {"start treatment"};
const char s_stop_treatment [] = {"stop treatment"};
const char s_status [] = {"status"};
const char s_getall [] = {"get all conf"};
const char s_buzzer_short [] = {"buzzer short"};
const char s_buzzer_half [] = {"buzzer half"};
const char s_buzzer_long [] = {"buzzer long"};



char buffMessages [100];


//--- FUNCIONES DEL MODULO ---//
void UpdateCommunications (void)
{
    if (SerialProcess() > 2)	//si tiene algun dato significativo
    {
        InterpretarMsg();
    }
}

//Procesa consultas desde la raspberry
//carga el buffer buffMessages y avisa con el flag msg_ready
unsigned char SerialProcess (void)
{
    unsigned char bytes_readed = 0;

    if (usart1_have_data)
    {
        usart1_have_data = 0;
        bytes_readed = ReadUsart1Buffer((unsigned char *) buffMessages, sizeof(buffMessages));
    }
    return bytes_readed;
}

resp_t InterpretarMsg (void)
{
    resp_t resp = resp_not_own;
    unsigned char ch = 0;
    char * pStr = buffMessages;
    unsigned short new_power = 0;
    unsigned char decimales = 0;
    char b [30];

    //reviso canal propio o canal broadcast
    //TODO: despues revisar esto y cargar directamente canal (si falta memoria)
    if ((strncmp(pStr, s_ch1, sizeof(s_chf) - 1) == 0) ||
        (strncmp(pStr, s_ch2, sizeof(s_chf) - 1) == 0) ||
        (strncmp(pStr, s_ch3, sizeof(s_chf) - 1) == 0) ||
        (strncmp(pStr, s_ch4, sizeof(s_chf) - 1) == 0) ||
        (strncmp(pStr, s_chf, sizeof(s_chf) - 1) == 0))
    {
        resp = resp_ok;

        //es broadcast, o que canal
        if (*(pStr + 2) == 'f')
            ch = 0x0F;
        else
        {
#ifdef FIRST_POWER_BOARD
            ch = *(pStr + 2) - 48;
#endif
#ifdef SECOND_POWER_BOARD
            ch = *(pStr + 2) - 48;
            ch -= 4;
#endif
        }

        pStr += sizeof(s_chf);	//normalizo al mensaje, hay un espacio

        //-- Signal Setting
        if (strncmp(pStr, s_set_signal, sizeof(s_set_signal) - 1) == 0)
        {
            pStr += sizeof(s_set_signal);		//normalizo al payload, hay un espacio

            if (strncmp(pStr, s_cwave, sizeof(s_cwave) - 1) == 0)
                resp = SetSignalType (CWAVE_SIGNAL);
            else if (strncmp(pStr, s_pulsed, sizeof(s_pulsed) - 1) == 0)
                resp = SetSignalType (PULSED_SIGNAL);
            else if (strncmp(pStr, s_modulated, sizeof(s_modulated) - 1) == 0)
                resp = SetSignalType (MODULATED_SIGNAL);
            else
                resp = resp_error;
        }

        //-- Frequency Setting
        else if (strncmp(pStr, s_frequency, sizeof(s_frequency) - 1) == 0)
        {
            pStr += sizeof(s_frequency);		//normalizo al payload, hay un espacio

            //lo que viene es un byte de 0 a 9 (0 es 10Hz)
            decimales = StringIsANumber(pStr, &new_power);
            if (decimales == 1)
            {
                resp = SetFrequency (new_power);
                // sprintf(b, "dec: %d, freq: %d\n", decimales, new_power);
                // Usart1Send(b);
            }
            else
                resp = resp_error;

        }

        //-- Power Setting for Leds
        else if (strncmp(pStr, s_power_led, sizeof(s_power_led) - 1) == 0)
        {
            pStr += sizeof(s_power_led);		//normalizo al payload, hay un espacio

            //lo que viene son 1 2 o 3 bytes
            decimales = StringIsANumber(pStr, &new_power);
            if (decimales < 4)
            {
                resp = SetPowerLed (ch, new_power);
                // sprintf(b, "ch: %d, dec: %d, power: %d\n", ch, decimales, new_power);
                // Usart1Send(b);
            }
            else
                resp = resp_error;
        }

        //-- Power Setting for Laser
        else if (strncmp(pStr, s_power_laser, sizeof(s_power_laser) - 1) == 0)
        {
            pStr += sizeof(s_power_laser);		//normalizo al payload, hay un espacio

            //lo que viene son 1 2 o 3 bytes
            decimales = StringIsANumber(pStr, &new_power);
            if (decimales < 4)
            {
                resp = SetPowerLaser (ch, new_power);
                // sprintf(b, "dec: %d, power: %d\n", decimales, new_power);
                // Usart1Send(b);
            }
            else
                resp = resp_error;
        }
        
        //-- Start Treatment
        else if (strncmp(pStr, s_start_treatment, sizeof(s_start_treatment) - 1) == 0)
        {
            //se puede empezar
            if (GetTreatmentState() == TREATMENT_STANDBY)
            {
                resp = StartTreatment();
            }
            else
                resp = resp_error;
        }

        //-- Stop Treatment
        else if (strncmp(pStr, s_stop_treatment, sizeof(s_stop_treatment) - 1) == 0)
        {
            StopTreatment();
        }

        //-- Status
        else if (strncmp(pStr, s_status, sizeof(s_status) - 1) == 0)
        {
            switch (GetErrorStatus())
            {
            case ERROR_OK:
                sprintf(b, "Manager status: %d\n", GetTreatmentState());
                Usart1Send(b);
                break;

            case ERROR_OVERCURRENT:
                Usart1Send("Error: Overcurrent\n");
                break;

            case ERROR_NO_CURRENT:
                Usart1Send("Error: No current\n");
                break;

            case ERROR_SOFT_OVERCURRENT:
                Usart1Send("Error: Soft Overcurrent\n");
                break;

            case ERROR_OVERTEMP:
                Usart1Send("Error: Overtemp\n");
                break;

            }
        }

        //-- Buzzer Actions
        else if (strncmp(pStr, s_buzzer_short, sizeof(s_buzzer_short) - 1) == 0)
        {
            pStr += sizeof(s_buzzer_short);		//normalizo al payload, hay un espacio

            //lo que viene es un byte de 1 a 9
            decimales = StringIsANumber(pStr, &new_power);
            if (decimales == 1)
                BuzzerCommands(BUZZER_SHORT_CMD, (unsigned char) new_power);
            else
                resp = resp_error;
        }

        else if (strncmp(pStr, s_buzzer_half, sizeof(s_buzzer_half) - 1) == 0)
        {
            pStr += sizeof(s_buzzer_half);		//normalizo al payload, hay un espacio

            //lo que viene es un byte de 1 a 9
            decimales = StringIsANumber(pStr, &new_power);
            if (decimales == 1)
                BuzzerCommands(BUZZER_HALF_CMD, (unsigned char) new_power);
            else
                resp = resp_error;
        }

        else if (strncmp(pStr, s_buzzer_long, sizeof(s_buzzer_long) - 1) == 0)
        {
            pStr += sizeof(s_buzzer_long);		//normalizo al payload, hay un espacio

            //lo que viene es un byte de 1 a 9
            decimales = StringIsANumber(pStr, &new_power);
            if (decimales == 1)
                BuzzerCommands(BUZZER_LONG_CMD, (unsigned char) new_power);
            else
                resp = resp_error;
        }
        

        //reviso errores y envio
        // 	error_t e;
        //
        // 	e = GetErrorStatus();
        // 	if (e == ERROR_OK)
        // 	{
        // 		sprintf(b, "Manager status: %d\n", GetTreatmentState());
        // 		Usart1Send(b);
        // 	}
        // 	else
        // 	{
        // 		//tengo algun error, los mando en secuencias
        // 		if (e & ERROR_OVERCURRENT)
        // 			Usart1Send("Error: Overcurrent\n");
        //
        // 		if (e & ERROR_NO_CURRENT)
        // 			Usart1Send("Error: No current\n");
        //
        // 		if (e & ERROR_SOFT_OVERCURRENT)
        // 			Usart1Send("Error: Soft Overcurrent\n");
        //
        // 		if (e & ERROR_OVERTEMP)
        // 			Usart1Send("Error: Overtemp\n");
        // 	}
        // }

        //-- Get All Configuration
        else if (strncmp(pStr, s_getall, sizeof(s_getall) - 1) == 0)
        {
            SendAllConf();
        }

        //-- Ninguno de los anteriores
        else
            resp = resp_error;

    }	//fin if chx

    if (ch != 0x0F)
    {
        if (resp == resp_ok)
            Usart1Send("OK\n");

        if (resp == resp_error)
            Usart1Send("NOK\n");
    }

    return resp;
}

//--- end of file ---//
