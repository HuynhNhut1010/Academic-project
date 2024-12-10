/*
 * stm32f411xx_spi.c
 *
 *  Created on: Sep 9, 2024
 *      Author: huynhnhut
 */

#include "stm32f411xx_spi.h"



static void SPI_OVR_Interupt_Handler(SPI_Handler_t *pSPI_Handler);
static void SPI_RXE_Interupt_Handler(SPI_Handler_t *pSPI_Handler);
static void SPI_TXE_Interupt_Handler(SPI_Handler_t *pSPI_Handler);




/********************************************************************************
 * @fn				- SPI_PCLK_Config
 *
 *
 * @brief          	-
 * @details        	-
 *
 * @param[in]      	- SPI_RegDef_t *pSPIx
 * @param[in]      	- uint8_t ENorDI
 *
 * @return         	void
 *
 *
 * @Note
 *
 *
 */
void SPI_PCLK_Config(SPI_RegDef_t *pSPIx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
		{
			if (pSPIx == SPI1){
				SPI1_PCLK_EN();
			}
			else if(pSPIx == SPI2){
				SPI2_PCLK_EN();
			}
			else if(pSPIx == SPI3){
				SPI3_PCLK_EN();
			}
			else if(pSPIx == SPI4){
				SPI4_PCLK_EN();
			}
			else if(pSPIx == SPI5){
				SPI5_PCLK_EN();
			}
		}
		else{
			if (pSPIx == SPI1){
				SPI1_PCLK_DI();
			}
			else if(pSPIx == SPI2){
				SPI2_PCLK_DI();
			}
			else if(pSPIx == SPI3){
				SPI3_PCLK_DI();
			}
			else if(pSPIx == SPI4){
				SPI4_PCLK_DI();
			}
			else if(pSPIx == SPI5){
				SPI5_PCLK_DI();
			}
		}
}
/********************************************************************************
 * @fn				- SPI_Init
 *
 *
 * @brief          	-
 * @details        	-
 *
 * @param[in]      	- SPI_RegDef_t *pSPI
 *
 *
 * @return         	void
 *
 *
 * @Note
 *
 *
 */
void SPI_Init(SPI_Handler_t *pSPIHandler)
{
	/* Enable peripheral clock */
	SPI_PCLK_Config(pSPIHandler->pSPIx, ENABLE);
	uint32_t temp = 0;
	//1. Configure the device mode
	temp |= (pSPIHandler->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);
	//2. Configure the bus
	if(pSPIHandler->SPIConfig.SPI_BusCongfig == SPI_BUS_CONFIG_FD)
	{
		temp &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandler->SPIConfig.SPI_BusCongfig == SPI_BUS_CONFIG_HD)
	{
		temp |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandler->SPIConfig.SPI_BusCongfig == SPI_BUS_CONFIG_RXONLY)
	{
		temp &= ~(1 << SPI_CR1_BIDIMODE);
		temp |= (1 << SPI_CR1_RXONLY);
	}
	//3. configure the baud rate
	temp |= (pSPIHandler->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);
	//4. configure the data frame format
	temp |= (pSPIHandler->SPIConfig.SPI_DFF <<  SPI_CR1_DFF);
	//5. configure the cpol
	temp |= (pSPIHandler->SPIConfig.SPI_CPOL <<  SPI_CR1_CPOL);
	//6. configure the cpol
	temp |= (pSPIHandler->SPIConfig.SPI_CPHA <<  SPI_CR1_CPHA);
	//7. Configure SSM
	temp |= (pSPIHandler->SPIConfig.SPI_SSM <<  SPI_CR1_SSM);

	pSPIHandler->pSPIx->CR1 = temp;
}




/********************************************************************************
 * @fn				- SPI_DeInit
 *
 *
 * @brief          	-
 * @details        	-
 *
 * @param[in]      	-
 *
 *
 * @return         	void
 *
 *
 * @Note
 *
 *
 */
void SPI_DeInit(SPI_RegDef_t *pGPIOx)
{

}

/********************************************************************************
 * @fn				- SPI_GetFlagStatus
 *
 *
 * @brief          	-
 * @details        	-
 *
 * @param[in]      	- SPI_RegDef_t *pSPIx,
 * @param[in]      	- uint8_t *pTxBuffer
 * @param[in]      	- uint32_t LenData
 *
 * @return         	void
 *
 *
 * @Note
 *
 *
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/********************************************************************************
 * @fn				- SPI_SendData
 *
 *
 * @brief          	-
 * @details        	-
 *
 * @param[in]      	- SPI_RegDef_t *pSPIx,
 * @param[in]      	- uint8_t *pTxBuffer
 * @param[in]      	- uint32_t LenData
 *
 * @return         	void
 *
 *
 * @Note			- This is a blocking call (polling API)
 *
 *
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t LenData)
{
	while(LenData > 0)
	{
		//1. wait until tx buffer is empty
		while((SPI_GetFlagStatus(pSPIx, SPI_FLAG_TXE) == FLAG_RESET));
		//2.  Check DFF
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 bit transfer
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			LenData --;
			LenData --;
			(uint16_t*)pTxBuffer ++;
		}
		else
		{
			// 8 bit transfer
			pSPIx->DR = *(pTxBuffer);
			LenData -= 1;
			pTxBuffer ++;
		}

	}
}


/********************************************************************************
 * @fn				- SPI_SendData
 *
 *
 * @brief          	-
 * @details        	-
 *
 * @param[in]      	- SPI_RegDef_t *pSPIx,
 * @param[in]      	- uint8_t *pTxBuffer
 * @param[in]      	- uint32_t LenData
 *
 * @return         	void
 *
 *
 * @Note			- This is a blocking call (polling API)
 *
 *
 */
void SPI_ReadData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t LenData)
{
	while(LenData)
	{
		//1. wait until rx buffer is empty
		while((SPI_GetFlagStatus(pSPIx, SPI_FLAG_RXNE) == FLAG_RESET));
		//2.  Check DFF
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 bit transfer
			*((uint16_t*)pRxBuffer) = pSPIx->DR ;
			LenData --;
			LenData --;
			(uint16_t*)pRxBuffer ++;
		}
		else
		{
			// 8 bit transfer
			*((uint8_t*)pRxBuffer) = pSPIx->DR ;
			LenData -= 1;
			pRxBuffer ++;
		}

	}
}



/********************************************************************************
 * @fn				- SPI_PeripheralControl
 *
 *
 * @brief          	-
 * @details        	-
 *
 * @param[in]      	- SPI_RegDef_t *pSPIx,
 * @param[in]      	- uint8_t ENorDI
 *
 * @return         	void
 *
 *
 * @Note			- This is a blocking call (polling API)
 *
 *
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI)
{
	if(ENorDI)
	{
		pSPIx->CR1 |= ENorDI << SPI_CR1_SP;
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SP);
	}
}

/********************************************************************************
 * @fn				- SPI_SSI_Config
 *
 *
 * @brief          	-
 * @details        	-
 *
 * @param[in]      	- SPI_RegDef_t *pSPIx,
 * @param[in]      	- uint8_t ENorDI
 *
 * @return         	void
 *
 *
 * @Note			- This is a blocking call (polling API)
 *
 *
 */
void SPI_SSI_Config(SPI_RegDef_t *pSPIx, uint8_t ENorDI)
{
	if(ENorDI)
	{
		pSPIx->CR1 |= 1 << SPI_CR1_SSI;
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}
/********************************************************************************
 * @fn				- SPI_SSOE_Config
 *
 *
 * @brief          	-
 * @details        	-
 *
 * @param[in]      	- SPI_RegDef_t *pSPIx,
 * @param[in]      	- uint8_t ENorDI
 *
 * @return         	void
 *
 *
 * @Note			- This is a blocking call (polling API)
 *
 *
 */
void SPI_SSOE_Config(SPI_RegDef_t *pSPIx, uint8_t ENorDI)
{
	if(ENorDI)
	{
		pSPIx->CR2 |= 1 << SPI_CR2_SSOE;
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}

}

/********************************************************************************
 * @fn				- SPI_SSOE_Config
 *
 *
 * @brief          	-
 * @details        	-
 *
 * @param[in]      	- SPI_RegDef_t *pSPIx,
 * @param[in]      	- uint8_t ENorDI
 *
 * @return         	void
 *
 *
 * @Note			- This is a blocking call (polling API)
 *
 *
 */
void SPI_FRF_Config(SPI_RegDef_t *pSPIx, uint8_t ENorDI)
{
	if(ENorDI)
	{
		pSPIx->CR2 |= 1 << SPI_CR2_FRF;
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_FRF);
	}

}

uint8_t SPI_SendDataIT(SPI_Handler_t *pSPI_Handler, uint8_t *pTxBuffer, uint32_t LenData)
{
	uint8_t state = pSPI_Handler->TxState;
	if (state != SPI_BUSY_IN_TX)
	{
		// 1. Save the buffer address and Len information in some global variables
		pSPI_Handler->TxLen = LenData;
		pSPI_Handler->pTxBuffer = pTxBuffer;

		// 2. Mark the SPI state as busy in transmission so that no other code can take over same SPI peripheral until transmission is over
		pSPI_Handler->TxState = SPI_BUSY_IN_TX;

		// 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPI_Handler->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		// 4, Data trasnmision will be handled by ISR code
	}
	return state;
}
uint8_t SPI_ReadDataIT(SPI_Handler_t *pSPI_Handler, uint8_t *pRxBuffer, uint32_t LenData)
{
	uint8_t state = pSPI_Handler->TxState;
	if (state != SPI_BUSY_IN_RX)
	{
		// 1. Save the buffer address and Len information in some global variables
		pSPI_Handler->RxLen = LenData;
		pSPI_Handler->pRxBuffer = pRxBuffer;

		// 2. Mark the SPI state as busy in transmission so that no other code can take over same SPI peripheral until transmission is over
		pSPI_Handler->TxState = SPI_BUSY_IN_RX;

		// 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPI_Handler->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		// 4, Data trasnmision will be handled by ISR code
	}
	return state;
}


//Interrupt config
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi){
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

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t Priority){
	uint8_t temp1 = IRQNumber/4;
	uint8_t temp2 = IRQNumber%4;

	uint8_t shift_number = (8*temp2) + (8-NO_PR_BITS_IMPLEMENTS);
	*(NVIC_PR_BASEADDR + (temp1)) |= (Priority << shift_number);
}


void SPI_IRQHandling(SPI_Handler_t *pSPI_Handler)
{
	// Let check for TXE
	uint8_t temp1, temp2;
	temp1 = pSPI_Handler->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPI_Handler->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
	if (temp1 && temp2)
	{
		SPI_TXE_Interupt_Handler(pSPI_Handler);
	}
	temp1 = pSPI_Handler->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPI_Handler->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
	if (temp1 && temp2)
	{
		SPI_RXE_Interupt_Handler(pSPI_Handler);
	}
	temp1 = pSPI_Handler->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPI_Handler->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	if (temp1 && temp2)
	{
		SPI_OVR_Interupt_Handler(pSPI_Handler);
	}

}





/*
 * Some helper functions
 */

static void SPI_TXE_Interupt_Handler(SPI_Handler_t *pSPI_Handler)
{
	if(pSPI_Handler->pSPIx->CR1 & (1<<SPI_CR1_DFF))
	{
		pSPI_Handler->pSPIx->DR = (*(uint16_t*)pSPI_Handler->pTxBuffer);
		pSPI_Handler->TxLen --;
		pSPI_Handler->TxLen --;
		(uint16_t*)pSPI_Handler->pTxBuffer ++;
	}
	else
	{
		pSPI_Handler->pSPIx->DR = (*(uint8_t*)pSPI_Handler->pTxBuffer);
		pSPI_Handler->TxLen --;
		(uint8_t*)pSPI_Handler->pTxBuffer ++;
	}
	if(!pSPI_Handler->TxLen)
	{
		// tx len is zero, close interrupted
		// TX is over
		pSPI_Handler->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
		pSPI_Handler->pTxBuffer = 0;
		pSPI_Handler->TxLen = 0;
		pSPI_Handler->TxState = SPI_READY;
		SPI_ApplicationEventCallback(pSPI_Handler, SPI_EVENT_TX_CMPLT);
	}
}


static void SPI_RXE_Interupt_Handler(SPI_Handler_t *pSPI_Handler)
{
	if(pSPI_Handler->pSPIx->CR1 & (1<<SPI_CR1_DFF))
	{
		*(uint16_t*)pSPI_Handler->pRxBuffer = pSPI_Handler->pSPIx->DR;
		pSPI_Handler->TxLen --;
		pSPI_Handler->TxLen --;
		(uint16_t*)pSPI_Handler->pRxBuffer ++;
	}
	else
	{
		*(uint8_t*)pSPI_Handler->pTxBuffer = pSPI_Handler->pSPIx->DR ;
		pSPI_Handler->TxLen --;
		(uint8_t*)pSPI_Handler->pTxBuffer ++;
	}
	if(!pSPI_Handler->TxLen)
	{
		// tx len is zero, close interrupted
		// TX is over
		pSPI_Handler->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
		pSPI_Handler->pRxBuffer = 0;
		pSPI_Handler->RxLen = 0;
		pSPI_Handler->RxState = SPI_READY;
		SPI_ApplicationEventCallback(pSPI_Handler, SPI_EVENT_RX_CMPLT);
	}
}

static void SPI_OVR_Interupt_Handler(SPI_Handler_t *pSPI_Handler)
{
	uint8_t temp;
	// 1. CLEAR THE ovr flag
	if(pSPI_Handler->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPI_Handler->pSPIx->DR;
		temp = pSPI_Handler->pSPIx->SR;
	}
	// 2. inform the application
	SPI_ApplicationEventCallback(pSPI_Handler, SPI_EVENT_OVR_ERR);
	(void)temp;
}


void SPI_CloseTramission(SPI_Handler_t *pSPI_Handler)
{
	pSPI_Handler->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPI_Handler->pTxBuffer = 0;
	pSPI_Handler->TxLen = 0;
	pSPI_Handler->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handler_t *pSPI_Handler)
{
	pSPI_Handler->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPI_Handler->pRxBuffer = 0;
	pSPI_Handler->RxLen = 0;
	pSPI_Handler->RxState = SPI_READY;
}


void SPI_ClearOVRFlag(SPI_Handler_t *pSPI_Handler)
{
	uint8_t temp;
	temp = pSPI_Handler->pSPIx->DR;
	temp = pSPI_Handler->pSPIx->SR;
	(void)temp;
}



__weak void SPI_ApplicationEventCallback(SPI_Handler_t *pSPI_Handler, uint8_t AppEv)
{

}


















