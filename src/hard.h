//---------------------------------------------
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ## @CPU:    STM32F030
// ##
// #### HARD.H ################################
//---------------------------------------------
#ifndef _HARD_H_
#define _HARD_H_

//--- Defines For Configuration ------------------------------------------------
// where to go?
#define VOUT_SETPOINT    VOLTS_35
#define VOUT_FOR_SOFT_START    VOLTS_30
#define VOUT_MAX_THRESHOLD    VOLTS_40
#define VOUT_MIN_THRESHOLD    VOLTS_20

#define SOFT_START_CNT_ROOF    4
#define UNDERSAMPLING_TICKS    10
#define UNDERSAMPLING_TICKS_SOFT_START    20

#define VOLTS_48    580
#define VOLTS_40    480    //estimado
#define VOLTS_35    350
#define VOLTS_30    274
#define VOLTS_25    202
#define VOLTS_20    136


//--- Type of program ----------------------------------------------------------
// #define HARD_TEST_MODE_STATIC_PWM
// #define HARD_TEST_MODE_DYNAMIC_PWM_STEP
// #define HARD_TEST_MODE_DYNAMIC_PWM_INCREMENT
// #define HARD_TEST_MODE_VOUT_SENSE
#define HALF_BRIDGE_MODE

//----------- Hardware Board Version -------------
#define VER_1_0    //version original


//---- Configuration for Hardware Versions -------
#ifdef VER_1_0
#define HARDWARE_VERSION_1_0
#define SOFTWARE_VERSION_1_0
#endif


//---- Features Configuration ----------------
//features are activeted here and annouced in hard.c
#define FEATURES

// #define USE_OPEN_LOOP_PROTECTION
#define SEND_V_SENSE_ON_USART

// SOFTWARE Features -------------------------
#define USE_LED_FOR_MAIN_STATES

//---- End of Features Configuration ----------



//--- Stringtify Utils -----------------------
#define STRING_CONCAT(str1,str2) #str1 " " #str2
#define STRING_CONCAT_NEW_LINE(str1,str2) xstr(str1) #str2 "\n"
#define xstr_macro(s) str_macro(s)
#define str_macro(s) #s

//--- Hardware Welcome Code ------------------//
#ifdef HARDWARE_VERSION_1_0
#define HARD "Hardware V: 1.0\n"
#endif

//--- Software Welcome Code ------------------//
#ifdef SOFTWARE_VERSION_1_0
#define SOFT "Software V: 1.0\n"
#endif


//-------- Hysteresis Conf ------------------------

//-------- PWM Conf ------------------------

//-------- End Of Defines For Configuration ------


// #define VOUT_200V    415
#define VOUT_110V    151    //ajustado 05-08-18
#define VOUT_200V    386    //ajustado 24-07-18
#define VOUT_205V    399    
#define VOUT_195V    373
#define VOUT_300V    660    //ajustado 24-07-18
#define VOUT_350V    802    //ajustado 24-07-18
#define VOUT_400V    917    //

//------- PIN CONFIG ----------------------
#ifdef VER_1_0
//GPIOA pin0	V_Sense / Vline_Sense
//GPIOA pin1	I_Sense_Pos
//GPIOA pin2	I_Sense_Neg

//GPIOA pin3	NC

//GPIOA pin4	
#define PROT_POS    ((GPIOA->IDR & 0x0010) != 0)

//GPIOA pin5
#define PROT_NEG    ((GPIOA->IDR & 0x0020) != 0)

//GPIOA pin6    TIM3_CH1 (L_LEFT)
//GPIOA pin7	TIM3_CH2 (H_LEFT)

//GPIOB pin0    TIM3_CH3 (L_RIGHT)
//GPIOB pin1	TIM3_CH4 (H_RIGHT)

//GPIOA pin8	
#define AC_SYNC ((GPIOA->IDR & 0x0100) != 0)

//GPIOA pin9
//GPIOA pin10	usart1 tx rx

//GPIOA pin11	NC

//GPIOA pin12	
#define LED ((GPIOA->ODR & 0x1000) != 0)
#define LED_ON	GPIOA->BSRR = 0x00001000
#define LED_OFF GPIOA->BSRR = 0x10000000

//GPIOA pin13	NC
//GPIOA pin14	NC

//GPIOA pin15
#define RELAY ((GPIOA->ODR & 0x8000) != 0)
#define RELAY_ON  GPIOA->BSRR = 0x00008000
#define RELAY_OFF GPIOA->BSRR = 0x80000000

//GPIOB pin3	NC
//GPIOB pin4	NC
//GPIOB pin5	NC

//GPIOB pin6
#define STOP_JUMPER ((GPIOB->IDR & 0x0040) == 0)

//GPIOB pin7	NC
#endif

//------- END OF PIN CONFIG -------------------

//ESTADOS DEL PROGRAMA PRINCIPAL
typedef enum
{
    POWER_UP = 0,
    POWER_UP_SOFT_START,
    VOLTAGE_MODE,
    INPUT_OVERVOLTAGE,
    OUTPUT_OVERVOLTAGE,
    OUTPUT_OPEN_LOOP,
    PEAK_OVERCURRENT,    
    BIAS_OVERVOLTAGE,
    JUMPER_PROTECTED,
    JUMPER_PROTECT_OFF

} board_state_t;

//ESTADOS DEL LED
typedef enum
{    
    START_BLINKING = 0,
    WAIT_TO_OFF,
    WAIT_TO_ON,
    WAIT_NEW_CYCLE
} led_state_t;


//Estados Externos de LED BLINKING
#define LED_NO_BLINKING               0
#define LED_POWER_UP                  1
#define LED_POWER_UP_SOFT_START       2
#define LED_VOLTAGE_MODE              3
#define LED_JUMPER_PROTECTED          4
#define LED_INPUT_OVERVOLTAGE         5
#define LED_OUTPUT_OVERVOLTAGE        6
#define LED_OUTPUT_OPEN_LOOP          7
#define LED_PEAK_OVERCURRENT    8
#define LED_BIAS_OVERVOLTAGE    9



#define SIZEOF_DATA1	512
#define SIZEOF_DATA		256
#define SIZEOF_DATA512	SIZEOF_DATA1
#define SIZEOF_DATA256	SIZEOF_DATA
#define SIZEOF_BUFFTCP	SIZEOF_DATA


#define LED_TOGGLE do { if (LED) \
                            LED_OFF; \
                        else         \
                            LED_ON;  \
                      } while (0)

// Module Functions Declarations -----------------------------------------------
// unsigned short GetHysteresis (unsigned char);
// unsigned char GetNew1to10 (unsigned short);
// void UpdateVGrid (void);
// void UpdateIGrid (void);
// unsigned short GetVGrid (void);
// unsigned short GetIGrid (void);
// unsigned short PowerCalc (unsigned short, unsigned short);
// unsigned short PowerCalcMean8 (unsigned short * p);
// void ShowPower (char *, unsigned short, unsigned int, unsigned int);
// unsigned short VoutTicksToVoltage (unsigned short);
// unsigned short VinTicksToVoltage (unsigned short);
void ChangeLed (unsigned char);
void UpdateLed (void);
void WelcomeCodeFeatures (char *);
    
#endif /* _HARD_H_ */
