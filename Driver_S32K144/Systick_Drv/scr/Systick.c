#include "Systick.h"

/**
* @brief          Initializes the SysTick timer.
* @details        This function configures the SysTick timer based on the input configuration.
*                 It sets the clock source, reload value, interrupt settings, and disables the timer.
*
* @param[in]      ConfigPtr  Pointer to the configuration structure that contains the SysTick settings.
*/
void Systick_Init(const Systick_ConfigType* ConfigPtr)
{
    unsigned int Tval;
    
    /* Step 1. Check input parameters */
    if(ConfigPtr->period == 0)  /**< Return if the period is 0 (invalid configuration). */
    {
        return;
    }
    else if(ConfigPtr->fSystick == 0)  /**< Return if the clock source frequency is 0 (invalid configuration). */
    {
        return;
    }
    
    /* Step 2. Configuration for SysTick timer */
    /* Clear the clock source bit (select the external clock source by default) */
    SYST_CSR &= (unsigned int)~(1 << SYST_CSR_CLKSOURCE);
    
    /* Check and set the clock source based on the configuration */
    if(ConfigPtr->clockSource == SYST_CKL_EXTERVAL)  /**< If external clock is selected */
    {
        SYST_CSR &= (unsigned int)~(1 << SYST_CSR_CLKSOURCE);  /**< Clear the clock source bit. */
    }
    else if(ConfigPtr->clockSource == SYST_CKL_PROCESSOR)  /**< If processor clock is selected */
    {
        SYST_CSR |= (1 << SYST_CSR_CLKSOURCE);  /**< Set the clock source bit to use processor clock. */
    }    
    
    /* Step 2.1. Disable the SysTick timer before configuring */
    Systick_Stop();
    
    /* Step 2.2. Calculate and set the reload value */
    Tval = ConfigPtr->fSystick * ConfigPtr->period - 1;  /**< Calculate the timer reload value based on the frequency and period. */
    
    if(ConfigPtr->period != 0)  /**< If the period is valid, set the reload value */
    {
        SYST_RVR = Tval;  /**< Set the calculated reload value in the SysTick reload register. */
    } 
    else if (ConfigPtr->period >= 0xFFFFFF)
    {
        SYST_RVR = 0xFFFFFFu;  /**< Set the maximum value if the period is 0 (fail-safe). */
    }
    else
		{
			// Do nothing
		}
    /* Step 2.3. Clear the current value of the timer */
    SYST_CVR = 0;  /**< Reset the current value register to 0. */
    
    /* Step 2.4. Check if the interrupt should be enabled */
    if(ConfigPtr->isInterruptEnabled != 0)  /**< Enable the SysTick interrupt if specified in the configuration. */
    {
        SYST_CSR |= (1 << SYST_CSR_TICKINT);  /**< Set the TICKINT bit to enable the interrupt. */
    }
    else
    {
        SYST_CSR &= (unsigned int) ~(1 << SYST_CSR_TICKINT);  /**< Clear the TICKINT bit to disable the interrupt. */
    }        
}

/**
* @brief          Starts the SysTick timer.
* @details        This function enables the SysTick timer by setting the ENABLE bit.
*/
void Systick_Start(void)
{
    /* Step 1. Enable the SysTick counter */
    SYST_CSR |= (1 << SYST_CSR_ENALBE);  /**< Set the ENABLE bit to start the SysTick timer. */
}

/**
* @brief          Stops the SysTick timer.
* @details        This function disables the SysTick timer by clearing the ENABLE bit.
*/
void Systick_Stop(void)
{
    /* Disable the SysTick counter */
    SYST_CSR &= (unsigned int)~(1 << SYST_CSR_ENALBE);  /**< Clear the ENABLE bit to stop the SysTick timer. */
}

/**
* @brief          Gets the current SysTick reload value.
* @details        This function returns the current reload value set in the SysTick timer.
*
* @return         The current reload value in the SysTick Reload Value Register (RVR).
*/
unsigned int Systick_GetCounter(void)
{
    return SYST_RVR;  /**< Return the value in the SysTick reload register (RVR). */
}

/**
* @brief          Introduces a delay using the SysTick timer.
* @details        This function configures the SysTick timer to generate a delay based on the 
*                 provided time in milliseconds and the clock frequency.
*
* @param[in]      ConfigPtr  Pointer to the configuration structure for the SysTick timer.
* @param[in]      ms         The delay time in milliseconds.
*/
void Delay_Systick(const Systick_ConfigType* ConfigPtr, unsigned int ms)
{
    Systick_Stop();  /**< Stop the SysTick timer before configuring it for the delay. */
    
    /* Calculate the reload value based on the desired delay in milliseconds */
    SYST_RVR = ConfigPtr->fSystick * ms - 1;  /**< Set the reload value for the required delay. */
}
