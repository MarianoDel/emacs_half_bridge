//-----------------------------------------------------
// #### HALF-BRIDGE MICROINVERTER BOARD F030 - Custom Board ####
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ## @CPU:    STM32F030
// ##
// #### MAIN.C ########################################
//-----------------------------------------------------

// Includes -------------------------------------------------------------------
#include "stm32f0xx.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "gpio.h"
#include "tim.h"
#include "uart.h"
#include "hard.h"

#include "core_cm0.h"
#include "adc.h"
#include "dma.h"
#include "flash_program.h"
#include "dsp.h"
#include "it.h"


// Externals -------------------------------------------------------------------

// - Externals from or for Serial Port -
volatile unsigned char tx1buff [SIZEOF_DATA];
volatile unsigned char rx1buff [SIZEOF_DATA];
volatile unsigned char usart1_have_data = 0;

// - Externals from or for the ADC -----
volatile unsigned short adc_ch [ADC_CHANNEL_QUANTITY];
volatile unsigned char seq_ready = 0;

// - Externals for the timers ----------
volatile unsigned short timer_led = 0;

// - Externals used for analog or digital filters ---------
// volatile unsigned short take_temp_sample = 0;


// Globals ---------------------------------------------------------------------
volatile unsigned char overcurrent_shutdown = 0;

// - for the PID -----------------------
pid_data_obj_t voltage_pid;

// - for the timers --------------------
volatile unsigned short wait_ms_var = 0;
volatile unsigned short timer_standby;
volatile unsigned short timer_meas;
volatile unsigned char timer_filters = 0;


// Module Functions ------------------------------------------------------------
void PWM_Off (void);
void TimingDelay_Decrement (void);
extern void EXTI4_15_IRQHandler(void);



//-------------------------------------------//
// @brief  Main program.
// @param  None
// @retval None
//------------------------------------------//
int main(void)
{
    char s_lcd [120];

    board_state_t board_state = POWER_UP;
    unsigned char soft_start_cnt = 0;
    unsigned char undersampling = 0;

    //GPIO Configuration.
    GPIO_Config();

    //ACTIVAR SYSTICK TIMER
    if (SysTick_Config(48000))
    {
        while (1)	/* Capture error */
        {
            if (LED)
                LED_OFF;
            else
                LED_ON;

            for (unsigned char i = 0; i < 255; i++)
            {
                asm (	"nop \n\t"
                        "nop \n\t"
                        "nop \n\t" );
            }
        }
    }

//---------- Pruebas de Hardware --------//
    // EXTIOff();
    USART1Config();
    
    //---- Welcome Code ------------//
    //---- Defines from hard.h -----//
#ifdef HARD
    Usart1Send((char *) HARD);
    Wait_ms(100);
#else
#error	"No Hardware defined in hard.h file"
#endif

#ifdef SOFT
    Usart1Send((char *) SOFT);
    Wait_ms(100);
#else
#error	"No Soft Version defined in hard.h file"
#endif

#ifdef FEATURES
    WelcomeCodeFeatures(s_lcd);
#endif


    TIM_3_Init();    //Used for mosfet L_LEFT control and ADC synchro
    TIM_1_Init();    //mosfet H_LEFT link to tim3

    PWM_Off();

#ifdef HARD_TEST_MODE_STATIC_PWM
    UpdateTIMSync(DUTY_FOR_DMAX);
    while (1);
#endif

#ifdef HARD_TEST_MODE_DYNAMIC_PWM_STEP
    unsigned char state = 0;
    
    while (1)
    {
        if (TIM3->SR & TIM_SR_UIF)
        {
            TIM3->SR &= ~TIM_SR_UIF;
            if (state == 0)
            {
                UpdateTIMSync(DUTY_10_PERCENT);
                state = 1;
            }
            else if (state)
            {
                UpdateTIMSync(DUTY_45_PERCENT);
                state = 0;
            }
        }
    }
#endif

#ifdef HARD_TEST_MODE_DYNAMIC_PWM_INCREMENT
    while (1)
    {
        for (unsigned short i = 0; i < DUTY_FOR_DMAX; i++)
        {
            UpdateTIMSync(i);
            Wait_ms(10);
        }
        Wait_ms(100);

        for (unsigned short i = DUTY_FOR_DMAX; i > 0; i--)
        {
            UpdateTIMSync(i);
            Wait_ms(10);
        }
        Wait_ms(100);
    }
#endif
    
    //ADC and DMA configuration
    AdcConfig();
    DMAConfig();
    DMA1_Channel1->CCR |= DMA_CCR_EN;
    ADC1->CR |= ADC_CR_ADSTART;
    //end of ADC & DMA

#ifdef HARD_TEST_MODE_VOUT_SENSE
    unsigned short seq_cnt = 0;
    
    while (1)
    {
        if (sequence_ready)
        {
            sequence_ready_reset;
            seq_cnt++;
        }

        if (seq_cnt > 24000)    //una vez por segundo??
        {
            seq_cnt = 0;
            
            sprintf(s_lcd, "vout: %d\n", V_Sense);
            Usart1Send(s_lcd);
        }
        
    }
#endif

#ifdef HALF_BRIDGE_MODE
    voltage_pid.kp = 1;
    voltage_pid.ki = 3;
    voltage_pid.kd = 0;
    short d = 0;
    
    //timer to power up
    ChangeLed(LED_POWER_UP);
    timer_standby = 10;
    while (1)
    {
        //the most work involved is sample by sample
        if (sequence_ready)
        {
            sequence_ready_reset;

            switch (board_state)
            {
            case POWER_UP:
                //the filters completes their action on 16 * 1/24KHz = 666us
                if (!timer_standby)
                {
                    if (V_Sense < VOUT_MAX_THRESHOLD)
                    {
                        d = 0;
                        PID_Small_Ki_Flush_Errors(&voltage_pid);
                        soft_start_cnt = 0;
                        board_state = POWER_UP_SOFT_START;
                        Usart1Send("To Soft Start\n");
                    }
                }
                break;

            case POWER_UP_SOFT_START:
                soft_start_cnt++;
                
                //check to not go overvoltage
                if (V_Sense < VOUT_FOR_SOFT_START)
                {
                    //do a soft start checking the voltage
                    if (soft_start_cnt > SOFT_START_CNT_ROOF)    //update 200us aprox.
                    {
                        soft_start_cnt = 0;

                        if (d < DUTY_FOR_DMAX)
                        {
                            d++;
                            UpdateTIMSync(d);
                        }
                        else
                        {
                            //update PID
                            voltage_pid.last_d = d;
                            ChangeLed(LED_VOLTAGE_MODE);
                            board_state = VOLTAGE_MODE;
                            Usart1Send("To Voltage Mode\n");                            
                        }
                    }

#ifdef USE_OPEN_LOOP_PROTECTION
                    //check if V_Sense got some kind of reaction
                    if ((d > DUTY_10_PERCENT) &&
                        (V_Sense < VOUT_MIN_THRESHOLD))
                    {
                        PWM_Off();
                        board_state = OUTPUT_OPEN_LOOP;
                        ChangeLed(LED_OUTPUT_OPEN_LOOP);
                        timer_standby = 10000;
                        Usart1Send("OUTPUT_OPEN_LOOP\n");
                    }
#endif
                }
                else
                {
                    //update PID
                    voltage_pid.last_d = d;
                    ChangeLed(LED_VOLTAGE_MODE);
                    board_state = VOLTAGE_MODE;
                    Usart1Send("To Voltage Mode\n");
                }
                break;
                
            case VOLTAGE_MODE:
                if (undersampling > (UNDERSAMPLING_TICKS - 1))
                {
                    undersampling = 0;
                    voltage_pid.setpoint = VOUT_SETPOINT;
                    // voltage_pid.sample = sense_boost_filtered;    //only if undersampling > 16
                    voltage_pid.sample = V_Sense;    //                 

                    d = PID_Small_Ki(&voltage_pid);

                    if (d > 0)
                    {
                        if (d > DUTY_FOR_DMAX)
                        {
                            d = DUTY_FOR_DMAX;
                            voltage_pid.last_d = DUTY_FOR_DMAX;
                        }
                    }
                    else
                    {
                        d = 0;
                        voltage_pid.last_d = 0;
                    }
                    
                    UpdateTIMSync(d);
                }
                else
                    undersampling++;

                break;
            
            case INPUT_OVERVOLTAGE:
                break;

            case OUTPUT_OVERVOLTAGE:
                if (V_Sense < VOUT_SETPOINT)
                {
                    board_state = VOLTAGE_MODE;
                    ChangeLed(LED_VOLTAGE_MODE);
                }
                break;

            case OUTPUT_OPEN_LOOP:
                if (!timer_standby)
                {
                    board_state = POWER_UP;
                    ChangeLed(LED_POWER_UP);
                }
                break;
                
            case PEAK_OVERCURRENT:
                if (!timer_standby)
                {
                    board_state = POWER_UP;
                    ChangeLed(LED_POWER_UP);
                }
                break;

            case BIAS_OVERVOLTAGE:
                break;

            case JUMPER_PROTECTED:
                if (!timer_standby)
                {
                    if (!STOP_JUMPER)
                    {
                        board_state = JUMPER_PROTECT_OFF;
                        timer_standby = 400;
                    }
                }                
                break;

            case JUMPER_PROTECT_OFF:
                if (!timer_standby)
                {
                    board_state = POWER_UP;
                    Usart1Send((char *) "Protect OFF\n");                    
                }                
                break;

            default:
                board_state = POWER_UP;
                break;

            }

            //
            //The things that are directly attached to the samples period
            //
        }    //end if sequence

        //
        //The things that are not directly attached to the samples period
        //
        if ((board_state != OUTPUT_OVERVOLTAGE) &&
            (V_Sense > VOUT_MAX_THRESHOLD))
        {
            PWM_Off();
            board_state = OUTPUT_OVERVOLTAGE;
            ChangeLed(LED_OUTPUT_OVERVOLTAGE);
            Usart1Send("OUTPUT_OVERVOLTAGE\n");
        }

#ifdef SEND_V_SENSE_ON_USART
        if ((board_state == VOLTAGE_MODE) &&
            (!timer_standby))
        {
            sprintf(s_lcd, "vout: %d\n", V_Sense);
            Usart1Send(s_lcd);
            timer_standby = 2000;
        }
#endif

        //for overcurrent
        // if (overcurrent_shutdown)
        // {
        //     PWM_Off();

        //     timer_standby = 10000;
        //     overcurrent_shutdown = 0;
        //     board_state = PEAK_OVERCURRENT;
        // }
        if (overcurrent_shutdown > 40)
        {
            PWM_Off();
            board_state = PEAK_OVERCURRENT;
            ChangeLed(LED_PEAK_OVERCURRENT);
            overcurrent_shutdown = 0;
            timer_standby = 10000;
            Usart1Send("PEAK_OVERCURRENT\n");
        }

        //Cosas que no tienen tanto que ver con las muestras o el estado del programa
        if ((STOP_JUMPER) &&
            (board_state != JUMPER_PROTECTED) &&
            (board_state != JUMPER_PROTECT_OFF) &&            
            (board_state != PEAK_OVERCURRENT))
        {
            PWM_Off();
            ChangeLed(LED_JUMPER_PROTECTED);
            Usart1Send((char *) "Protect ON\n");
            timer_standby = 1000;
            board_state = JUMPER_PROTECTED;
        }
        
#ifdef USE_LED_FOR_MAIN_STATES
        UpdateLed();
#endif
    }    //end while 1
#endif    //end HALF_BRIDGE_MODE

    return 0;
}

//--- End of Main ---//


void PWM_Off (void)
{
    DisablePreload_Mosfet_HighLeft;
    DisablePreload_Mosfet_LowLeft;

    UpdateTIMSync(DUTY_NONE);

    EnablePreload_Mosfet_LowLeft;
    EnablePreload_Mosfet_HighLeft;
}


void TimingDelay_Decrement(void)
{
    if (wait_ms_var)
        wait_ms_var--;

    if (timer_standby)
        timer_standby--;

    // if (take_temp_sample)
    //     take_temp_sample--;

    // if (timer_meas)
    //     timer_meas--;

    if (timer_led)
        timer_led--;

    // if (timer_filters)
    //     timer_filters--;

    // //cuenta de a 1 minuto
    // if (secs > 59999)	//pasaron 1 min
    // {
    // 	minutes++;
    // 	secs = 0;
    // }
    // else
    // 	secs++;
    //
    // if (minutes > 60)
    // {
    // 	hours++;
    // 	minutes = 0;
    // }


}

#define AC_SYNC_Int        (EXTI->PR & 0x00000100)
#define AC_SYNC_Set        (EXTI->IMR |= 0x00000100)
#define AC_SYNC_Reset      (EXTI->IMR &= ~0x00000100)
#define AC_SYNC_Ack        (EXTI->PR |= 0x00000100)

#define AC_SYNC_Int_Rising          (EXTI->RTSR & 0x00000100)
#define AC_SYNC_Int_Rising_Set      (EXTI->RTSR |= 0x00000100)
#define AC_SYNC_Int_Rising_Reset    (EXTI->RTSR &= ~0x00000100)

#define AC_SYNC_Int_Falling          (EXTI->FTSR & 0x00000100)
#define AC_SYNC_Int_Falling_Set      (EXTI->FTSR |= 0x00000100)
#define AC_SYNC_Int_Falling_Reset    (EXTI->FTSR &= ~0x00000100)

#define OVERCURRENT_POS_Int        (EXTI->PR & 0x00000010)
#define OVERCURRENT_POS_Ack        (EXTI->PR |= 0x00000010)
#define OVERCURRENT_NEG_Int        (EXTI->PR & 0x00000020)
#define OVERCURRENT_NEG_Ack        (EXTI->PR |= 0x00000020)

// #define TIM16_HYSTERESIS    100
void EXTI4_15_IRQHandler(void)
{
#ifdef WITH_AC_SYNC_INT
    if (AC_SYNC_Int)
    {
        if (AC_SYNC_Int_Rising)
        {
            AC_SYNC_Int_Rising_Reset;
            AC_SYNC_Int_Falling_Set;

            SYNC_Rising_Edge_Handler();
#ifdef USE_LED_FOR_AC_PULSES
            LED_ON;
#endif
        }
        else if (AC_SYNC_Int_Falling)
        {
            AC_SYNC_Int_Falling_Reset;
            AC_SYNC_Int_Rising_Set;
            
            SYNC_Falling_Edge_Handler();
#ifdef USE_LED_FOR_AC_PULSES            
            LED_OFF;
#endif
        }
        AC_SYNC_Ack;
    }
#endif
    
#ifdef WITH_OVERCURRENT_SHUTDOWN
    if (OVERCURRENT_POS_Int)
    {
        DisablePreload_Mosfet_HighLeft;
        HIGH_LEFT(DUTY_NONE);
        //TODO: trabar el TIM3 aca!!!
        overcurrent_shutdown = 1;
        OVERCURRENT_POS_Ack;
    }

    if (OVERCURRENT_NEG_Int)
    {
        DisablePreload_Mosfet_HighRight;
        HIGH_RIGHT(DUTY_NONE);
        //TODO: trabar el TIM3 aca!!!
        overcurrent_shutdown = 2;
        OVERCURRENT_NEG_Ack;
    }
#endif
}

//------ EOF -------//
