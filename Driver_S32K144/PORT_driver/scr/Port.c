#include "Port.h"

/*!
 * @brief Initializes the pins with the given configuration structure
 *
 * This function configures the pins with the options provided in the
 * provided structure.
 *
 * @param[in] pinCount The number of configured pins in structure
 * @param[in] config The configuration structure
 * @return Port_ret_t ret
 */
Port_ret_t Port_Init(const Port_ConfigType* ConfigPtr)
{
	/* Check parameter */
	/*    if error return PORT_ERR_PARA */
	/**/
	Port_ret_t ret = PORT_ERR_PARA;
	unsigned int regValue = ConfigPtr->base->PCR[ConfigPtr->pinPortIdx];
	
	/* 1. Internal resistor pull feature selection. */
	switch (ConfigPtr->pullConfig)
  {
		case PORT_NO_PULL_UP_DOWN:
			{
					regValue &= ~(1u << 0);
					regValue &= ~(1u << 1);
			}
			break;
		case PORT_PULL_DOWN:
			{
					regValue &= ~1u;
					regValue |= (1u << 1);
			}
			break;
		case PORT_PULL_UP:
			{
					regValue |= 1u;
					regValue |= (1u << 1);
			}
			break;
	}
	/* 2. Configures the drive strength.*/
	switch (ConfigPtr->driveSelect)
  {
		case PORT_LOW_DRV_STRENGTH:
			{
					regValue &= ~(1u << 6);
			}
			break;
		case PORT_HIGH_DRV_STRENGTH:
			{
					regValue |= (1u << 6);
			}
			break;
	}
	
	/* 3. mux selection. */
	regValue &= ~(unsigned int)(0x0f << 8);
	regValue |= (unsigned int)(ConfigPtr->mux << 8);
	
	/* 4. Interrupt generation condition. */
	regValue |= (unsigned int)(ConfigPtr->intConfig << 16);


  ConfigPtr->base->PCR[ConfigPtr->pinPortIdx] = regValue;
	ret = PORT_OK;
	return ret;
	
}

unsigned char Check_IT_Flag(PORT_Type* base,unsigned int  Pin_Number)
{
	if(base->PCR[Pin_Number] & 0x01 << PORT_PCRn_ISF)
	{
		base->PCR[Pin_Number] |= (1<<PORT_PCRn_ISF);
		return 1;
	}
	else
	{
		return 0;
	}
	
}
