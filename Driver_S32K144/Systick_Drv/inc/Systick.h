/**
*   @file    Systick.h
*   @brief   Header file for the SysTick driver.
*   @details This file contains the declarations and definitions required to
*            initialize, start, stop, and configure the SysTick timer.
*/

#ifndef SYSTICK_H
#define SYSTICK_H



/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "Systick_Register.h" /**< Includes register definitions for SysTick timer */





/*==================================================================================================
*                                      DEFINES AND MACROS
==================================================================================================*/
/**
* @brief Macro for enabling the SysTick timer.
*/
#define SYST_CSR_ENALBE 0

/**
* @brief Macro for enabling SysTick exception request.
*/
#define SYST_CSR_TICKINT 1

/**
* @brief Macro for selecting the clock source of SysTick.
*/
#define SYST_CSR_CLKSOURCE 2

/**
* @brief Macro for checking if SysTick counted to 0.
*/
#define SYST_CSR_COUNTFLAG 16

/**
* @brief Macro to define the reload value for the timer period.
*/
#define SYST_RVR_RELOAD 0

/**
* @brief Macro for using an external clock source for SysTick.
*/
#define SYST_CKL_EXTERVAL 0

/**
* @brief Macro for using the processor clock as SysTick's clock source.
*/
#define SYST_CKL_PROCESSOR 1



/*==================================================================================================
*                                             STRUCTURES
==================================================================================================*/
/**
* @brief          Configuration structure for SysTick timer.
* @details        This structure contains the configuration parameters required to
*                 initialize the SysTick timer, such as clock source, timer period,
*                 and interrupt enable settings.
*/
typedef struct 
{
    unsigned int fSystick;                      /*!< F clock source (in Hz)                       */
    unsigned int period;                        /*!< Period of the SysTick timer (in ms)          */
    unsigned char isInterruptEnabled;           /*!< Flag to enable or disable SysTick interrupt  */
    unsigned char clockSource;                  /*!< Clock source selection (external or processor) */
} Systick_ConfigType;



/*==================================================================================================
*                                    FUNCTION PROTOTYPES
==================================================================================================*/
/**
* @brief          Initializes the SysTick timer with the provided configuration.
* @details        This function sets up the SysTick timer based on the provided 
*                 configuration parameters such as clock source, timer period, 
*                 and interrupt enable flag.
*
* @param[in]      ConfigPtr  Pointer to the configuration structure.
*/
void Systick_Init(const Systick_ConfigType* ConfigPtr);

/**
* @brief          Starts the SysTick timer.
* @details        This function enables and starts the SysTick timer.
*/
void Systick_Start(void);

/**
* @brief          Stops the SysTick timer.
* @details        This function disables the SysTick timer.
*/
void Systick_Stop(void);

/**
* @brief          Gets the current value of the SysTick counter.
* @details        This function returns the current count value of the SysTick timer.
*
* @return         The current count value of the SysTick timer.
*/
unsigned int Systick_GetCounter(void);

/**
* @brief          Delays execution by a specified time using the SysTick timer.
* @details        This function introduces a delay by using the SysTick timer based 
*                 on the provided configuration and delay time.
*
* @param[in]      ConfigPtr  Pointer to the configuration structure.
* @param[in]      ms         Delay time in milliseconds.
*/
void Delay_Systick(const Systick_ConfigType* ConfigPtr, unsigned int ms);

#endif /* SYSTICK_H */
