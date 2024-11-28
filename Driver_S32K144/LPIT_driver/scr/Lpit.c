#include "Lpit.h"

/*!
 * @brief Initializes the LPIT module.
 *
 * This function configures the LPIT module, enabling debug mode and the
 * peripheral clock to the timers.
 */
void Lpit_Init( void )
{
	/* Step 1. Set Debug mode (Debug mode enabled in the MCR register) */
	LPIT0->MCR |= (1u<<LPIT_MCR_DBG_EN_SHIFT);
	
	/* Step 2. Enable peripheral clock to timers (Enable clock in MCR register) */
	LPIT0->MCR |= (1u<<LPIT_MCR_M_CEN_SHIFT);
}

/*!
 * @brief Initializes a specific LPIT channel with the provided configuration.
 *
 * This function sets up the operation mode of the timer channel and configures
 * the period and interrupt enablement for the specified channel.
 *
 * @param[in] channel The LPIT channel number (0-3)
 * @param[in] ConfigPtr Pointer to the channel configuration structure
 */
void Lpit_InitChannel(unsigned char channel, const Lpit_ChannelConfigType* ConfigPtr)
{
	/* Step 1. Check parameter (Ensure ConfigPtr is not NULL) */
	if(ConfigPtr == NULL)
	{
		return;
	}
	
	/* Step 2. Set the timer channel to 32-bit Periodic Counter mode */
	LPIT0->TMR[channel].TCTRL &=~ (3u<<LPIT_TMR_TCTRL_MODE_SHIFT);
	
	/* Step 3. Set the timer value register (TVAL) */
	if(ConfigPtr->period != 0 )
	{
		LPIT0->TMR[channel].TVAL = ConfigPtr->period;
	} 
	else 
	{
		LPIT0->TMR[channel].TVAL = 0xFFFFFFFFu;
	}
	
	/* Step 4. Enable the timeout interrupt if configured */
	if(ConfigPtr->isInterruptEnabled != 0 )
	{
		LPIT0->MIER |=(1u<<channel);
	}
}

/*!
 * @brief Starts the specified LPIT channel.
 *
 * This function enables counting for the specified timer channel.
 *
 * @param[in] channel The LPIT channel number (0-3)
 */
void Lpit_StartChannel(unsigned char channel)
{
	/* Step 1. Check parameter (Ensure valid channel number) */
	if(channel > 3)
	{
		return;
	}
	/* Step 2. Start the timer channel counting by enabling the timer */
	LPIT0->TMR[channel].TCTRL |= (1u<<LPIT_TMR_TCTRL_T_EN_SHIFT);
}

/*!
 * @brief Stops the specified LPIT channel.
 *
 * This function stops the counting of the specified timer channel.
 *
 * @param[in] channel The LPIT channel number (0-3)
 */
void Lpit_StopChannel(unsigned char channel)
{
	/* Step 1. Check parameter (Ensure valid channel number) */
	if(channel > 3)
	{
		return;
	}
	else
	{
		/* Step 2. Stop the timer channel by disabling the timer */
		LPIT0->TMR[channel].TCTRL &= ~(1u<<LPIT_TMR_TCTRL_T_EN_SHIFT);
	}
}

/*!
 * @brief Gets the current counter value of the specified LPIT channel.
 *
 * This function returns the current value of the timer counter for the
 * specified channel.
 *
 * @param[in] channel The LPIT channel number (0-3)
 * @return The current counter value, or 0 if the channel is invalid.
 */
unsigned int Lpit_GetCounterChannel(unsigned char channel)
{
	/* Step 1. Check parameter (Ensure valid channel number) */
	if(channel > 3)
	{
		return 0;
	}
	else
	{	
		/* Step 2. Return the current timer value (CVAL) */
		return LPIT0->TMR[channel].CVAL ;
	}
}
