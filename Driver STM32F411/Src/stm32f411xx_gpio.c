/*
 * stm32f411xx_gpio.c
 *
 *  Created on: Jun 5, 2024
 *      Author: huynhnhut
 */
#include "stm32f411xx_gpio.h"


//clock config
void GPIO_PCLK_Config(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI){
	if(ENorDI == ENABLE)
	{
		if (pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}

	}
	else{
		if (pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}
	}
}


// Init and deinitilize pin
void GPIO_Init(GPIO_Handle_t *pGPIO_Handler){

	uint32_t temp = 0;
	// Enable clock
	GPIO_PCLK_Config(pGPIO_Handler->pGPIO, ENABLE);


	// 1. Configure the mode of gpio pin
	if (pGPIO_Handler->pGPIO_Config.GPIO_PIN_MODE < GPIO_MODE_ANALOG){
		temp =(uint32_t) ((pGPIO_Handler->pGPIO_Config.GPIO_PIN_MODE) << (2 * (pGPIO_Handler->pGPIO_Config.GPIO_PIN_NUM)));
		pGPIO_Handler->pGPIO->MODE &= ~(0x03<< (2 * (pGPIO_Handler->pGPIO_Config.GPIO_PIN_NUM)));
		pGPIO_Handler->pGPIO->MODE |= temp;
	}
	else{
		//Set as input
		pGPIO_Handler->pGPIO->MODE &= ~(0x03<< (2 * (pGPIO_Handler->pGPIO_Config.GPIO_PIN_NUM)));
		//set interrupt
		EXTI->IMR |= (1 << pGPIO_Handler->pGPIO_Config.GPIO_PIN_NUM);
		if(pGPIO_Handler->pGPIO_Config.GPIO_PIN_MODE == GPIO_MODE_IT_FT)
		{
			//1. configure the FTSR
			EXTI->FTSR |= (1 << pGPIO_Handler->pGPIO_Config.GPIO_PIN_NUM);
			EXTI->RTSR &= ~(1 << pGPIO_Handler->pGPIO_Config.GPIO_PIN_NUM);
		}
		else if(pGPIO_Handler->pGPIO_Config.GPIO_PIN_MODE == GPIO_MODE_IT_RF)
		{
			//2. configure the RFSR
			EXTI->RTSR |= (1 << pGPIO_Handler->pGPIO_Config.GPIO_PIN_NUM);
			EXTI->FTSR &= ~(1 << pGPIO_Handler->pGPIO_Config.GPIO_PIN_NUM);
		}
		else if(pGPIO_Handler->pGPIO_Config.GPIO_PIN_MODE == GPIO_MODE_IT_RFT)
		{
			//3. configure the both FTSR, RFSR
			EXTI->RTSR |= (1 << pGPIO_Handler->pGPIO_Config.GPIO_PIN_NUM);
			EXTI->FTSR |= (1 << pGPIO_Handler->pGPIO_Config.GPIO_PIN_NUM);
		}

		//2. configure the GPIO port selection on SYSCFG-EXTICT
		uint8_t temp1 = pGPIO_Handler->pGPIO_Config.GPIO_PIN_NUM/4 ;
		uint8_t temp2 = pGPIO_Handler->pGPIO_Config.GPIO_PIN_NUM%4 ;
		uint16_t postcode = GPIO_BASEADDR_TO_CODE(pGPIO_Handler->pGPIO);
		SYSCFG_PLCK_EN();
		SYSCFG->EXTICR[temp1] = postcode << (4 * temp2);
		//3. enable the exti interrupt delivery using IMR
		EXTI->EMR |= (1 << pGPIO_Handler->pGPIO_Config.GPIO_PIN_NUM);
	}
	temp = 0;
	//2. Config the Speed of gpio pin
	temp = pGPIO_Handler->pGPIO_Config.GPIO_PIN_SPEED << (2*pGPIO_Handler->pGPIO_Config.GPIO_PIN_NUM);
	pGPIO_Handler->pGPIO->OSPEEDR |= temp;


	temp = 0;
	//3. Config the pupd of gpio pin
	temp = pGPIO_Handler->pGPIO_Config.GPIO_PIN_PuPdControl << (2*pGPIO_Handler->pGPIO_Config.GPIO_PIN_NUM);
	pGPIO_Handler->pGPIO->OSPEEDR |= temp;

	temp = 0;
	//4. Config the opt type of gpio pin
	temp = pGPIO_Handler->pGPIO_Config.GPIO_PIN_OPTType << (pGPIO_Handler->pGPIO_Config.GPIO_PIN_NUM);
	pGPIO_Handler->pGPIO->OTYPER |= temp;

	temp = 0;
	//5. Config the alt function of gpio pin
	if (pGPIO_Handler->pGPIO_Config.GPIO_PIN_MODE == GPIO_MODE_ALFN){
		temp = pGPIO_Handler->pGPIO_Config.GPIO_PIN_MODE << (2 * pGPIO_Handler->pGPIO_Config.GPIO_PIN_NUM);
		pGPIO_Handler->pGPIO->MODE |= temp;
		uint8_t temp1,temp2;
		temp1 = pGPIO_Handler->pGPIO_Config.GPIO_PIN_NUM / 8;
		temp2 = pGPIO_Handler->pGPIO_Config.GPIO_PIN_NUM % 8;
		pGPIO_Handler->pGPIO->AFR[temp1] |= pGPIO_Handler->pGPIO_Config.GPIO_PIN_ATLFuct << 4*temp2;
	}
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if (pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}
	else if(pGPIOx==GPIOB){
		GPIOB_REG_RESET();
	}
	else if(pGPIOx==GPIOC){
		GPIOC_REG_RESET();
	}
	else if(pGPIOx==GPIOD){
		GPIOD_REG_RESET();
	}
	else if(pGPIOx==GPIOE){
		GPIOE_REG_RESET();
	}
	else if(pGPIOx==GPIOH){
		GPIOH_REG_RESET();
	}


}

/*
 * Read and write data
 */
uint8_t GPIO_ReadIutputFromPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	uint8_t value;
	value =	(uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

uint16_t GPIO_ReadIutputFromPort(GPIO_RegDef_t *pGPIOx){

	uint8_t value;
	value =	(uint8_t)((pGPIOx->IDR));
	return value;

}


void GPIO_WriteOutputFromPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value){

	if (value == SET){
		pGPIOx->ODR |= 0x01 << PinNumber;
	}
	else{
		pGPIOx->ODR &= ~(0x01 << PinNumber);
	}

}
void GPIO_WriteOutputFromPort(GPIO_RegDef_t *pGPIOx, uint8_t value){

	pGPIOx->ODR = value;

}
void GPIO_ToggleOutputFromPort(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1<<PinNumber);
}


//Interrupt config
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if (IRQNumber < 33){
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber >= 33 && IRQNumber < 64){
			*NVIC_ISER1 |= (1<<(IRQNumber%32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96){
			*NVIC_ISER2 |= (1<<(IRQNumber % 64));
		}
	}
	else{
		if (IRQNumber < 33){
			*NVIC_ICER0 &= ~(1<<IRQNumber);
		}
		else if(IRQNumber >= 33 && IRQNumber < 64){
			*NVIC_ICER1 &= ~(1<<(IRQNumber%32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96){
			*NVIC_ICER2 &= ~(1<<(IRQNumber % 64));
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t Priority){
	uint8_t temp1 = IRQNumber/4;
	uint8_t temp2 = IRQNumber%4;

	uint8_t shift_number = (8*temp2) + (8-NO_PR_BITS_IMPLEMENTS);
	*(NVIC_PR_BASEADDR + (temp1)) |= (Priority << shift_number);
}
void GPIO_IRQHandling(uint8_t PinNumber){
	if(EXTI->PR &(1<<PinNumber)){
		EXTI->PR |= 1<< PinNumber;
	}
}


