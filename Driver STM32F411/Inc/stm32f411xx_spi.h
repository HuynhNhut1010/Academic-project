/*
 * stm32f411xx_SPI.h
 *
 *  Created on: Sep 4, 2024
 *      Author: huynhnhut
 */

#ifndef INC_STM32F411XX_SPI_H_
#define INC_STM32F411XX_SPI_H_

#include "stm32f411xx.h"

/*
 *  Configuration structure for SPIx peripheral
 */

typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusCongfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;

}SPI_Config_t;

/*
 *  Configuration structure for SPIx Handler
 */

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
	uint8_t *pTxBuffer; 	/* !< To store the app. Tx buffer address> */
	uint8_t *pRxBuffer;		/* !< To store the app. Tx buffer address> */
	uint8_t TxLen;			/* !< To store the TxLen> */
	uint8_t RxLen;			/* !< To store the TxLen> */
	uint8_t TxState;		/* !< To store the TxState */
	uint8_t RxState;		/* !< To store the RxState */
}SPI_Handler_t;




/*
 * @SPI_DeviceMode
 */
#define SPI_MASTER_MODE 	1
#define SPI_SLAVE_MODE 		0


/*
 * @SPI_BusCongfig
 */

#define SPI_BUS_CONFIG_FD 		1
#define SPI_BUS_CONFIG_HD 		2
#define SPI_BUS_CONFIG_RXONLY 	3


/*
 * @SPI_SclkSpeed
 */

#define SPI_SCLK_SPEED_DIV2 		0
#define SPI_SCLK_SPEED_DIV4 		1
#define SPI_SCLK_SPEED_DIV8 		2
#define SPI_SCLK_SPEED_DIV16 		3
#define SPI_SCLK_SPEED_DIV32 		4
#define SPI_SCLK_SPEED_DIV64 		5
#define SPI_SCLK_SPEED_DIV128 		6
#define SPI_SCLK_SPEED_DIV256 		7


/*
 * @SPI_DFF
 */
#define SPI_DFF_8BIT 		0
#define SPI_DFF_16BIT 		1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW 		0
#define SPI_CPOL_HIGH 		1



/*
 * @SPI_CPHA
 */
#define SPI_CPHA_LOW 		0
#define SPI_CPHA_HIGH 		1



/*
 * @SPI_SSM
 */
#define SPI_SSM_EN 		1
#define SPI_SSM_DI		0


/*
 * SPI status flag
 */
#define SPI_FLAG_TXE		(1 << SPI_SR_TXE)
#define SPI_FLAG_RXNE		(1 << SPI_SR_RXNE)
#define SPI_FLAG_CHSIDE		(1 << SPI_SR_CHSIDE)
#define SPI_FLAG_UDR		(1 << SPI_SR_UDR)
#define SPI_FLAG_CRCERR		(1 << SPI_SR_CRCERR)
#define SPI_FLAG_MODF		(1 << SPI_SR_MODF)
#define SPI_FLAG_OVR		(1 << SPI_SR_OVR)
#define SPI_FLAG_BSY		(1 << SPI_SR_BSY)
#define SPI_FLAG_FRE		(1 << SPI_SR_FRE)



/*
 * SPI app state
 */
#define SPI_READY						0
#define SPI_BUSY_IN_RX					1
#define SPI_BUSY_IN_TX					2



/*
 * SPI application even
 */
#define SPI_EVENT_TX_CMPLT				0
#define SPI_EVENT_RX_CMPLT				1
#define SPI_EVENT_OVR_ERR				2


/************************************************************************************************
 * 									APIs supported by this driver
 * 					For more information about the APIs check the function definitions
 ***********************************************************************************************
 */

/*
 * Peripheral clock setup
 */
void SPI_PCLK_Config(SPI_RegDef_t *pSPIx, uint8_t ENorDI);

/*
 *  Init and De-Init SPI
 */
void SPI_Init(SPI_Handler_t *pSPIHandler);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


/*
 *  TX and RX SPI
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t LenData);
void SPI_ReadData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t LenData);

uint8_t SPI_SendDataIT(SPI_Handler_t *pSPI_Handler, uint8_t *pTxBuffer, uint32_t LenData);
uint8_t SPI_ReadDataIT(SPI_Handler_t *pSPI_Handler, uint8_t *pRxBuffer, uint32_t LenData);

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t FlagName);

//Interrupt config
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t Priority);
void SPI_IRQHandling(SPI_Handler_t *pSPI_Handler);

// Enable peripheral
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI);
void SPI_SSI_Config(SPI_RegDef_t *pSPIx, uint8_t ENorDI);
void SPI_SSOE_Config(SPI_RegDef_t *pSPIx, uint8_t ENorDI);
void SPI_FRF_Config(SPI_RegDef_t *pSPIx, uint8_t ENorDI);

void SPI_ClearOVRFlag(SPI_Handler_t *pSPI_Handler);
void SPI_CloseTramission(SPI_Handler_t *pSPI_Handler);
void SPI_CloseReception(SPI_Handler_t *pSPI_Handler);

void SPI_ApplicationEventCallback(SPI_Handler_t *pSPI_Handler, uint8_t AppEv);

#endif /* INC_STM32F411XX_SPI_H_ */
