#include "GPIO.h"
/*!
 * @brief Initializes the pins with the given configuration structure
 *
 * This function configures the mode of GPIO pins with the options provided in the
 * provided structure.
 *
 * @param[in] ConfigPtr The address of struct configure pin
 * @return none
 */
void Gpio_Init(const Gpio_ConfigType* ConfigPtr)
{
	if(ConfigPtr->GPIO_PinMode == GPIO_IN_MODE)
	{
			ConfigPtr->base->PDDR &= ~(unsigned int)(1 << ConfigPtr->GPIO_PinMode);
	}
	else if(ConfigPtr->GPIO_PinMode == GPIO_OUT_MODE)
	{
		ConfigPtr->base->PDDR |=(unsigned int)(1 << ConfigPtr->GPIO_PinNumber);
	}
}
	
/*!
 * @brief Initializes the pins with the given configuration structure
 *
 * This function write value to the pins with the options provided in the
 * provided structure.
 *
 * @param[in] pGPIOx The number of configured pins in sructure
 * @param[in] PinNumber The number of configured pins in sructure
 * @param[in] value The configuration structure
 * @return 
 */

void GPIO_WriteToOutputPin(GPIO_Type *pGPIOx, unsigned char PinNumber, unsigned char value)
{
	if(value == SET)
	{
		pGPIOx->PDOR |= (unsigned int)(1 << PinNumber);
	}
	else if(value == RESET)
	{
		pGPIOx->PDOR &= ~(unsigned int)(1 << PinNumber);
	}
	
}

/*!
 * @brief Initializes the pins with the given configuration structure
 *
 * This function set output the pins with the options provided in the
 * provided structure.
 *
 * @param[in] *pGPIOx Pointer of base address of port 
 * @param[in] PinNumber The Pin number of port 
 * @return 
 */
void GPIO_SetOutputPin(GPIO_Type *pGPIOx, unsigned char PinNumber)
{
    /* Check parameter */
	pGPIOx->PSOR |= (unsigned int)(0x01 << PinNumber);

}
/*!
 * @brief Initializes the pins with the given configuration structure
 *
 * This function clear output the the pins with the options provided in the
 * provided structure.
 *
 * @param[in] *pGPIOx Pointer of base address of port 
 * @param[in] PinNumber The Pin number of port 
 * @return 
 */
void GPIO_ResetOutputPin(GPIO_Type *pGPIOx, unsigned char PinNumber)
{
	pGPIOx->PCOR |= (unsigned int)(0x01 << PinNumber);

}
/*!
 * @brief Initializes the pins with the given configuration structure
 *
 * This function configures the pins with the options provided in the
 * provided structure.
 *
 * @param[in] *pGPIOx Pointer of base address of port 
 * @param[in] PinNumber The Pin number of port 
 * @return 
 */
void GPIO_ToggleOutputPin(GPIO_Type *pGPIOx, unsigned char PinNumber)
{
    /* Check parameter */
    pGPIOx->PDOR ^= (1u << PinNumber);

}

/*!
 * @brief Read the input pin
 *
 * This function read the value of input pin the pins with the options provided in the
 * provided structure.
 *
 * @param[in] *pGPIOx Pointer of base address of port 
 * @param[in] PinNumber The Pin number of port 
 * @return 1 or 0 
 */
unsigned char GPIO_ReadFromInputPin(GPIO_Type *pGPIOx, unsigned char PinNumber)
{
    /* Check parameter */
		return (pGPIOx->PDIR >> PinNumber) & 0x01; 
	
}
