/*
 * stm32f411xx_i2c.h
 *
 *  Created on: Oct 3, 2024
 *      Author: huynhnhut
 */

#ifndef INC_STM32F411XX_I2C_H_
#define INC_STM32F411XX_I2C_H_
/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;

/*
 * Handler structure for I2Cx peripheral
 */

typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
}I2C_Handler_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM 100000
#define I2C_SCL_SPEED_FM 400000

/*
 * @I2C_DeviceAddress
 */
#define I2C_SCL_SPEED_SM 100000
#define I2C_SCL_SPEED_FM 400000

/*
 * @I2C_ACKControl
 */
#define I2C_ACK_ENABLE 1
#define I2C_ACK_DISABLE 0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

/*
 * @I2C_FLAGNAME
 */
#define I2C_FLAG_ADDR 		(0x01 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF 		(0x01 << I2C_SR1_BTF)
#define I2C_FLAG_SB 		(0x01 << I2C_SR1_SB)
#define I2C_FLAG_ADD10 		(0x01 << I2C_SR1_ADD10)
#define I2C_FLAG_STOPF 		(0x01 << I2C_SR1_STOPF)
#define I2C_FLAG_RxNE 		(0x01 << I2C_SR1_RXNE)
#define I2C_FLAG_TxNE 		(0x01 << I2C_SR1_TXE)
#define I2C_FLAG_BERR 		(0x01 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO 		(0x01 << I2C_SR1_ARLO)
#define I2C_FLAG_AF 		(0x01 << I2C_SR1_AF)
#define I2C_FLAG_OVR 		(0x01 << I2C_SR1_OVR)
#define I2C_FLAG_PECERR 	(0x01 << I2C_SR1_PECERR)
#define I2C_FLAG_TIMEOUT 	(0x01 << I2C_SR1_TIMEOUT)
#define I2C_FLAG_SMB_ALERT 	(0x01 << I2C_SR1_SMBALERT)


/************************************************************************************************
 * 									APIs supported by this driver
 * 					For more information about the APIs check the function definitions
 ***********************************************************************************************
 */

/*
 * Peripheral clock setup
 */
void I2C_PCLK_Config(I2C_RegDef_t *pI2Cx, uint8_t ENorDI);

/*
 *  Init and De-Init I2C
 */
void I2C_Init(I2C_Handler_t *pI2CHandler);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 *  TX and RX I2C
 */
void I2C_MasterSentData(I2C_Handler_t *pI2CHandler, uint8_t *pTxBuffer, uint8_t Len, uint8_t SlaveAddr);


//Interrupt config
void I2C_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t Priority);
void I2C_IRQHandling(I2C_Handler_t *pI2C_Handler);

// Enable peripheral
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t ENorDI);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint8_t FlagName);

/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handler_t *pI2C_Handler, uint8_t AppEv);

#endif /* INC_STM32F411XX_I2C_H_ */
