#ifndef GPIO_H
#define GPIO_H

#include "Gpio_Registers.h"

#define GPIO_IN_MODE 		0
#define GPIO_OUT_MODE 	1
#define SET 						1
#define RESET 					0

typedef struct
{
	  GPIO_Type*      					base;                  /*!< Gpio base pointer.  */
	  unsigned char            	GPIO_PinNumber;        /*!< Pin number.*/
    unsigned char            	GPIO_PinMode;          /*!< Pin_mode */
		unsigned char 						padding[2];						 /*!< Padding */
} Gpio_ConfigType;

/*!
 * Declare Function protocol
 */
void Gpio_Init(const Gpio_ConfigType* ConfigPtr);
void GPIO_WriteToOutputPin(GPIO_Type *pGPIOx, unsigned char PinNumber, unsigned char value);
void GPIO_SetOutputPin(GPIO_Type *pGPIOx, unsigned char PinNumber);
void GPIO_ResetOutputPin(GPIO_Type *pGPIOx, unsigned char PinNumber);
void GPIO_ToggleOutputPin(GPIO_Type *pGPIOx, unsigned char PinNumber);
unsigned char GPIO_ReadFromInputPin(GPIO_Type *pGPIOx, unsigned char PinNumber);

#endif
