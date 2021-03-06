//---------------------------------------------
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ## @CPU:    STM32F030
// ##
// #### HARD.C ################################
//---------------------------------------------

// Includes --------------------------------------------------------------------
#include "hard.h"
#include "tim.h"
#include "stm32f0xx.h"
#include "adc.h"
#include "dsp.h"

#include <stdio.h>
#include "uart.h"

// Externals -------------------------------------------------------------------
extern volatile unsigned short timer_led;
extern volatile unsigned short adc_ch[];


// Globals ---------------------------------------------------------------------
//para el led
led_state_t led_state = START_BLINKING;
unsigned char blink = 0;
unsigned char how_many_blinks = 0;


// Module Functions Definitions ------------------------------------------------

//cambia configuracion de bips del LED
void ChangeLed (unsigned char how_many)
{
    how_many_blinks = how_many;
    led_state = START_BLINKING;
}

//mueve el LED segun el estado del Pote
void UpdateLed (void)
{
    switch (led_state)
    {
        case START_BLINKING:
            blink = how_many_blinks;
            
            if (blink)
            {
                LED_ON;
                timer_led = 200;
                led_state++;
                blink--;
            }
            break;

        case WAIT_TO_OFF:
            if (!timer_led)
            {
                LED_OFF;
                timer_led = 200;
                led_state++;
            }
            break;

        case WAIT_TO_ON:
            if (!timer_led)
            {
                if (blink)
                {
                    blink--;
                    timer_led = 200;
                    led_state = WAIT_TO_OFF;
                    LED_ON;
                }
                else
                {
                    led_state = WAIT_NEW_CYCLE;
                    timer_led = 2000;
                }
            }
            break;

        case WAIT_NEW_CYCLE:
            if (!timer_led)
                led_state = START_BLINKING;

            break;

        default:
            led_state = START_BLINKING;
            break;
    }
}


// //Convierte el valor de ticks ADC Vout a tension
// unsigned short VoutTicksToVoltage (unsigned short sample_adc)
// {
//     unsigned int num;

//     if (sample_adc > VOUT_300V)
//     {
//         num = sample_adc * 350;
//         num = num / VOUT_350V;
//     }
//     else if (sample_adc > VOUT_200V)
//     {
//         num = sample_adc * 300;
//         num = num / VOUT_300V;
//     }
//     else if (sample_adc > VOUT_110V)
//     {
//         num = sample_adc * 200;
//         num = num / VOUT_200V;
//     }
//     else
//     {
//         num = sample_adc * 110;
//         num = num / VOUT_110V;
//     }
    
//     return (unsigned short) num;
// }

// //Convierte el valor de ticks ADC Vin a tension
// unsigned short VinTicksToVoltage (unsigned short sample_adc)
// {
//     unsigned int num;

//     if (sample_adc > VIN_30V)
//     {
//         num = sample_adc * 35;
//         num = num / VIN_35V;
//     }
//     else if (sample_adc > VIN_25V)
//     {
//         num = sample_adc * 30;
//         num = num / VIN_30V;
//     }
//     else if (sample_adc > VIN_20V)
//     {
//         num = sample_adc * 25;
//         num = num / VIN_25V;
//     }    
//     else
//     {
//         num = sample_adc * 20;
//         num = num / VIN_20V;
//     }
    
//     return (unsigned short) num;
// }



void WelcomeCodeFeatures (char * str)
{
    // Main Program Type
#ifdef HARD_TEST_MODE_STATIC_PWM
    sprintf(str,"[%s] %s\n", __FILE__, str_macro(HARD_TEST_MODE_STATIC_PWM));
    Usart1Send(str);
    Wait_ms(30);    
#endif

#ifdef HARD_TEST_MODE_DYNAMIC_PWM_STEP
    sprintf(str,"[%s] %s\n", __FILE__, str_macro(HARD_TEST_MODE_DYNAMIC_PWM_STEP));
    Usart1Send(str);
    Wait_ms(30);    
#endif

#ifdef HARD_TEST_MODE_DYNAMIC_PWM_INCREMENT
    sprintf(str,"[%s] %s\n", __FILE__, str_macro(HARD_TEST_MODE_DYNAMIC_PWM_INCREMENT));
    Usart1Send(str);
    Wait_ms(30);    
#endif
    
#ifdef HARD_TEST_MODE_VOUT_SENSE
    sprintf(str,"[%s] %s\n", __FILE__, str_macro(HARD_TEST_MODE_VOUT_SENSE));
    Usart1Send(str);
    Wait_ms(30);    
#endif

#ifdef HALF_BRIDGE_MODE
    sprintf(str,"[%s] %s\n", __FILE__, str_macro(HALF_BRIDGE_MODE));
    Usart1Send(str);
    Wait_ms(30);    
#endif
    
    // Features mostly on hardware
    
}

//---- end of file ----//
