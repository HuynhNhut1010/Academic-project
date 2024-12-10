/*
 * stm32f411xx_gpio.h
 *
 *  Created on: Jun 5, 2024
 *      Author: huynhnhut
 */

#ifndef INC_STM32F411XX_GPIO_H_
#define INC_STM32F411XX_GPIO_H_

#include "stm32f411xx.h"


typedef struct{
	__vo uint8_t GPIO_PIN_NUM;							/*!< Possible value from @GPIO_PIN_NUM>!*/
	__vo uint8_t GPIO_PIN_MODE;							/*!< Possible value from @GPIO_PIN_MODE>!*/
	__vo uint8_t GPIO_PIN_SPEED;						/*!< Possible value from @GPIO_SPEED_MODE>!*/
	__vo uint8_t GPIO_PIN_PuPdControl;					/*!< Possible value from @GPIO_INPUT_MODE>!*/
	__vo uint8_t GPIO_PIN_OPTType;						/*!< Possible value from @GPIO_OUTPUT_MODE>!*/
	__vo uint8_t GPIO_PIN_ATLFuct;

}GPIO_PIN_Config_t;

typedef struct{
	GPIO_RegDef_t *pGPIO;
	GPIO_PIN_Config_t pGPIO_Config;
}GPIO_Handle_t;

/*
 * @GPIO_PIN_NO
 * GPIO pin possible mode
 */

#define GPIO_PIN_NO_0						0
#define GPIO_PIN_NO_1 						1
#define GPIO_PIN_NO_2 						2
#define GPIO_PIN_NO_3						3
#define GPIO_PIN_NO_4						4
#define GPIO_PIN_NO_5						5
#define GPIO_PIN_NO_6						6
#define GPIO_PIN_NO_7						7
#define GPIO_PIN_NO_8 						8
#define GPIO_PIN_NO_9						9
#define GPIO_PIN_NO_10						10
#define GPIO_PIN_NO_11 						11
#define GPIO_PIN_NO_12 						12
#define GPIO_PIN_NO_13 						13
#define GPIO_PIN_NO_14 						14
#define GPIO_PIN_NO_15 						15





/*
 * @GPIO_PIN_MODE
 * GPIO pin possible mode
 */

#define GPIO_MODE_IN 						0
#define GPIO_MODE_OUT 						1
#define GPIO_MODE_ALFN 						2
#define GPIO_MODE_ANALOG 					3
#define GPIO_MODE_IT_FT 					4
#define GPIO_MODE_IT_RF 					5
#define GPIO_MODE_IT_RFT 					6

/*
 * @GPIO_INPUT_MODE
 * GPIO pin input mode
 */
#define GPIO_INPUT_TYPE_NO					0
#define GPIO_INPUT_TYPE_PU					1
#define GPIO_INPUT_TYPE_PD 					2


/*
 * @GPIO_OUTPUT_MODE
 * GPIO pin output mode
 */
#define GPIO_OUTPUT_TYPE_PP					0
#define GPIO_OUTPUT_TYPE_OD					1

/*
 * @GPIO_SPEED_MODE
 * GPIO pin SPPED mode
 */
#define GPIO_LOW_SPEED						0
#define GPIO_MED_SPEED						1
#define GPIO_FAST_SPEED						2
#define GPIO_HIGH_SPEED						3

//clock config
void GPIO_PCLK_Config(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI);


// Init and deinitilize pin
void GPIO_Init(GPIO_Handle_t *pGPIO);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// data read and wirte
uint8_t GPIO_ReadIutputFromPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadIutputFromPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteOutputFromPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteOutputFromPort(GPIO_RegDef_t *pGPIOx, uint8_t value);
void GPIO_ToggleOutputFromPort(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

//Interrupt config
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t Priority);
void GPIO_IRQHandling(uint8_t PinNumber);















#endif /* INC_STM32F411XX_GPIO_H_ */
