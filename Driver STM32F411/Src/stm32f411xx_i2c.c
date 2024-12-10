/*
 * stm32f411xx_i2c.c
 *
 *  Created on: Oct 3, 2024
 *      Author: huynhnhut
 */

#include "stm32f411xx.h"


uint32_t AHB_Prescaler[] = {2, 4, 8, 16, 64, 128, 256, 512};

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint8_t FlagName)
{
	if(pI2Cx->SR1 && FlagName)
	{
		return FLAG_SET;
	}
	else
	{
		return FLAG_RESET;
	}
}

static void I2C_ExeAddrPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1 << 0); // r/nw bit set to Write(0) (total 8 bits)
	pI2Cx->DR = SlaveAddr;
}
static void I2C_ClearAddrFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;
}

uint32_t RCC_GetPLLOutClock()
{
	return 1;
}



uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SysytemClk;
	uint8_t clkscr, temp, ahbp, ahb1p;
	clkscr = (RCC->CFGR >> 2) & 0x03;
	if(clkscr == 0)
	{
		SysytemClk = 16000000;
	}
	else if(clkscr == 1)
	{
		SysytemClk = 8000000;
	}
	else if(clkscr == 2)
	{
		SysytemClk = RCC_GetPLLOutClock();
	}
	else
	{
		// Do nothing
	}

	temp = (RCC->CFGR >> 4) & 0x0F;

	if(temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_Prescaler[temp-8];
	}

	temp = (RCC->CFGR >> 10) & 0x07;

	if(temp < 4)
	{
		ahb1p = 1;
	}
	else
	{
		ahb1p = AHB_Prescaler[temp-4];
	}

	pclk1 = (SysytemClk / ahbp) / ahb1p;

	return pclk1;

}

/********************************************************************************
 * @fn				- I2C_Init
 *
 *
 * @brief          	-
 * @details        	-
 *
 * @param[in]      	- *pI2CHandler
 *
 * @return         	void
 *
 *
 * @Note
 *
 *
 */
void I2C_Init(I2C_Handler_t *pI2CHandler)
{
	uint32_t tempReg = 0;
	// enable clock i2c
	I2C_PCLK_Config(pI2CHandler->pI2Cx, ENABLE);


	// ack control bit
	tempReg |= (pI2CHandler->I2C_Config.I2C_ACKControl) << 10;
	pI2CHandler->pI2Cx->CR1 |= tempReg;

	/* Configure the mode (standard or fast mode)*/

	// Program FREQ field of CR2 register with the value of PCLK1
	tempReg = 0;
	tempReg |= RCC_GetPCLK1Value() / 1000000u;
	pI2CHandler->pI2Cx->CR2 = (tempReg & 0X3F);

	// Configure the device address (slave mode)
	tempReg = 0;
	tempReg |= pI2CHandler->I2C_Config.I2C_DeviceAddress << 1;
	tempReg |= (1<<14); // always keep the 14th bit is HIGH by sofware
	pI2CHandler->pI2Cx->OAR1 = tempReg;

	// Calculate and program CCR value in CCR field of CCR register
	uint16_t ccr_value = 0;
	tempReg = 0;
	if(pI2CHandler->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// Configure the standard mode
		ccr_value = RCC_GetPCLK1Value() / (2*(pI2CHandler->I2C_Config.I2C_SCLSpeed));
		tempReg = ccr_value & 0xFFF;
	}
	else
	{
		// Configure fast mode
		tempReg |= 1 << I2C_CCR_FS; // set up fast mode
		tempReg |= (pI2CHandler->I2C_Config.I2C_FMDutyCycle << 14); // set up duty cycle

		if(pI2CHandler->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = RCC_GetPCLK1Value() / (3*(pI2CHandler->I2C_Config.I2C_SCLSpeed));
		}
		else
		{
			ccr_value = RCC_GetPCLK1Value() / (25*(pI2CHandler->I2C_Config.I2C_SCLSpeed));
		}
		tempReg = ccr_value & 0xFFF;
	}
	pI2CHandler->pI2Cx->CCR = tempReg;

	// Enable the ACKing

	// Configure the rise time for I2C pin
	tempReg = 0;
	if(pI2CHandler->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// Configure the standard mode
		tempReg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}
	else
	{
		// Configure the fast mode
		tempReg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}
	pI2CHandler->pI2Cx->TRISE = tempReg & 0x3F;
}






/********************************************************************************
 * @fn				- I2C_PCLK_Config
 *
 *
 * @brief          	-
 * @details        	-
 *
 * @param[in]      	- I2C_RegDef_t *pI2Cx
 * @param[in]      	- uint8_t ENorDI
 *
 * @return         	void
 *
 *
 * @Note
 *
 *
 */
void I2C_PCLK_Config(I2C_RegDef_t *pI2Cx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
		{
			if (pI2Cx == I2C1){
				I2C1_PCLK_EN();
			}
			else if(pI2Cx == I2C2){
				I2C2_PCLK_EN();
			}
			else if(pI2Cx == I2C3){
				I2C3_PCLK_EN();
			}
		}
		else{
			if (pI2Cx == I2C1){
				I2C1_PCLK_DI();
			}
			else if(pI2Cx == I2C2){
				I2C2_PCLK_DI();
			}
			else if(pI2Cx == I2C3){
				I2C3_PCLK_DI();
			}
		}
}

/********************************************************************************
 * @fn				- I2C_PeripheralControl
 *
 *
 * @brief          	-
 * @details        	-
 *
 * @param[in]      	- I2C_RegDef_t *pI2Cx,
 * @param[in]      	- uint8_t ENorDI
 *
 * @return         	void
 *
 *
 * @Note			- This is a blocking call (polling API)
 *
 *
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t ENorDI)
{
	if(ENorDI)
	{
		pI2Cx->CR1 |= ENorDI << I2C_CR1_PE;
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}



/********************************************************************************
 * @fn				- I2C_MasterSentData
 *
 *
 * @brief          	-
 * @details        	-
 *
 * @param[in]      	- I2C_Handler_t *pI2CHandler
 * @param[in]      	- uint8_t *pTxBuffer
 * @param[in]      	- uint8_t Len
 * @param[in]      	- uint8_t SlaveAddr
 * @return         	- void
 *
 *
 * @Note			- Master sent data to slave
 *
 *
 */

void I2C_MasterSentData(I2C_Handler_t *pI2CHandler, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr)
{
	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandler->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SRI
	// 		Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(! I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits)
	I2C_ExeAddrPhase(pI2CHandler->pI2Cx, SlaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while(! I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_ADDR));

	//5. clear the ADDR flag according to its software sequence
	I2C_ClearAddrFlag(pI2CHandler->pI2Cx);

	// Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)

	//6. send the data until Len becomes 0
	while(Len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_TxNE));
		pI2CHandler->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}
	//7. when Len becomes zero wait for TXE-1 and BTF-1 before generating the STOP condition
	while(! I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_TxNE));
	while(! I2C_GetFlagStatus(pI2CHandler->pI2Cx, I2C_FLAG_BTF));
	// Note: TXE-1, BTF-1, means that both SR and DR are empty and next transmission should begin

	// when BTF-1 SCL will be stretched (pulled to LOW)

	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	// Note: generating STOP, automatically clears the BTF
	I2C_GenerateStopCondition(pI2CHandler->pI2Cx);



}






















/********************************************************************************
 * @fn				- I2C_IRQITConfig
 *
 *
 * @brief          	-
 * @details        	-
 *
 * @param[in]      	- IRQNumber
 * @param[in]      	- uint8_t ENorDI
 *
 * @return         	void
 *
 *
 * @Note
 *
 *
 */
void I2C_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi){
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



















/********************************************************************************
 * @fn				- I2C_IRQPriorityConfig
 *
 *
 * @brief          	-
 * @details        	-
 *
 * @param[in]      	- IRQNumber
 * @param[in]      	- Priority
 *
 * @return         	void
 *
 *
 * @Note
 *
 *
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t Priority){
	uint8_t temp1 = IRQNumber/4;
	uint8_t temp2 = IRQNumber%4;

	uint8_t shift_number = (8*temp2) + (8-NO_PR_BITS_IMPLEMENTS);
	*(NVIC_PR_BASEADDR + (temp1)) |= (Priority << shift_number);
}
