/*
 * stm32f411xx.h
 *
 *  Created on: May 29, 2024
 *      Author: huynhnhut
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include <stdint.h>
#define __vo volatile
#define __weak __attribute__((weak))
/*
 * ARM cortex processor NVIC ISERx register Address
 */
#define  NVIC_ISER0			 		(__vo uint32_t*)(0xE000E100)
#define  NVIC_ISER1			 		(__vo uint32_t*)(0xE000E104)
#define  NVIC_ISER2			 		(__vo uint32_t*)(0xE000E108)
#define  NVIC_ISER3					(__vo uint32_t*)(0xE000E10C)


/*
 * ARM cortex processor NVIC ICERx register Address
 */
#define  NVIC_ICER0			 		(__vo uint32_t*)(0xE000E180)
#define  NVIC_ICER1			 		(__vo uint32_t*)(0xE000E184)
#define  NVIC_ICER2			 		(__vo uint32_t*)(0xE000E188)
#define  NVIC_ICER3			 		(__vo uint32_t*)(0xE000E18C)

#define NVIC_PR_BASEADDR			((__vo uint32_t*)0xE000E400)
#define NO_PR_BITS_IMPLEMENTS		4


#define FLASH_BASEADDR				0x08000000U
#define SRAM_BASEADDR				0x20000000U
#define SRAM 						SRAM_BASEADDR
#define ROM_BASEADDR				0x1FFF0000U


//AHB and APB bus peripheral base address

#define PERIPH_BASEADDR				0x40000000U
#define AHB1_BASEADDR				0x40020000U
#define AHB2_BASEADDR				0x50000000U
#define APB1_BASEADDR 				PERIPH_BASEADDR
#define APB2_BASEADDR				0x40010000U

//GPIO BASE ADDRESS

#define GPIOA_BASEADDR				(AHB1_BASEADDR + 0x0000)
#define GPIOB_BASEADDR				(AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR				(AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR				(AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR				(AHB1_BASEADDR + 0x1000)
#define GPIOH_BASEADDR				(AHB1_BASEADDR + 0x1C00)
#define RCC_BASEADDR				(AHB1_BASEADDR + 0x3800)


#define GPIOA 						((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 						((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 						((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 						((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 						((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH 						((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define RCC 						((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI						((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG						((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1						((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2						((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3						((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4						((SPI_RegDef_t*)SPI4_BASEADDR)
#define SPI5						((SPI_RegDef_t*)SPI5_BASEADDR)

#define I2C1						((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2						((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3						((I2C_RegDef_t*)I2C3_BASEADDR)



//Communication protocol HANGING ON APB1 BUS

#define I2C1_BASEADDR				(APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR				(APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR				(APB1_BASEADDR + 0x5C00)

#define SPI2_BASEADDR				(APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR				(APB1_BASEADDR + 0x3C00)

#define PWR_BASEADDR				(APB1_BASEADDR + 0x7000)

#define USART2_BASEADDR				(APB1_BASEADDR + 0x4400)



//Communication protocol HANGING ON APB2 BUS

#define UART1_BASEADDR				(APB2_BASEADDR + 0x1000)
#define UART6_BASEADDR				(APB2_BASEADDR + 0x1400)

#define ADC1_BASEADDR				(APB2_BASEADDR + 0x2000)

#define SPI1_BASEADDR				(APB2_BASEADDR + 0x3000)
#define SPI4_BASEADDR				(APB2_BASEADDR + 0x3400)
#define SPI5_BASEADDR				(APB2_BASEADDR + 0x5000)

#define EXTI_BASEADDR				(APB2_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR				(APB2_BASEADDR + 0x3800)

/*
 * NVIC number or EXTI position
 */
#define EXTI0						6
#define EXTI1						7
#define EXTI2						8
#define EXTI3						9
#define EXTI4						10
#define EXTI9_5						23
#define EXTI15_10					40

/*
 * NVIC number of SPI
 */
#define NVIC_SPI1_IRQn				42
#define NVIC_SPI2_IRQn				43
#define NVIC_SPI3_IRQn				58
#define NVIC_SPI4_IRQn				91
#define NVIC_SPI5_IRQn				92

/*
 * GPIO_RegDef_t;
 */

typedef struct{
	__vo uint32_t MODE;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];
}GPIO_RegDef_t;


/*
 * RCC_RegDef_t;
 */

typedef struct{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	uint32_t Reserved1;
	uint32_t Reserved2;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t Reserved3;
	uint32_t Reserved4;
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	uint32_t Reserved5;
	uint32_t Reserved6;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t Reserved7;
	uint32_t Reserved8;
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	uint32_t Reserved9;
	uint32_t Reserved10;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t Reserved11;
	uint32_t Reserved12;
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	uint32_t Reserved13;
	__vo uint32_t DCKCFGR;
}RCC_RegDef_t;

/*
 * EXTI_RegDef_t
 */

typedef struct{
	__vo uint32_t IMR;							//Address offset: 0x00
	__vo uint32_t EMR;							//Address offset: 0x04
	__vo uint32_t RTSR;							//Address offset: 0x08
	__vo uint32_t FTSR;							//Address offset: 0x0C
	__vo uint32_t SWIER;						//Address offset: 0x10
	__vo uint32_t PR;							//Address offset: 0x14
}EXTI_RegDef_t;

/*
 * SYSCFG_RegDef_t
 */

typedef struct{
	__vo uint32_t MEMRMP;						//Address offset: 0x00
	__vo uint32_t PMC;							//Address offset: 0x04
	__vo uint32_t EXTICR[3];					//Address offset: 0x08 - 0x14
	uint32_t RESERVE[2]; 						//Address offset: 0x18 - 0x1C
	__vo uint32_t CMPCR;						//Address offset: 0x20
}SYSCFG_RegDef_t;


/*
 * SPI_RegDef_t
 */
typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
}SPI_RegDef_t;



/*
 * I2C_RegDef_t
 */

typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;
}I2C_RegDef_t;





/*
 * clock  ENABLE macro for GPIOx peripheral
 */


#define GPIOA_PCLK_EN() 				(RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN() 				(RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN() 				(RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN() 				(RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN() 				(RCC->AHB1ENR |= (1<<4))
#define GPIOH_PCLK_EN() 				(RCC->AHB1ENR |= (1<<7))

//clock  ENABLE macro for I2Cx peripheral

#define I2C1_PCLK_EN() 					(RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN() 					(RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN() 					(RCC->APB1ENR |= (1<<23))

//clock  ENABLE macro for SPI peripheral

#define SPI1_PCLK_EN() 					(RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN() 					(RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN() 					(RCC->AHB1ENR |= (1<<15))
#define SPI4_PCLK_EN() 					(RCC->APB2ENR |= (1<<13))
#define SPI5_PCLK_EN() 					(RCC->APB2ENR |= (1<<20))


//clock  ENABLE macro for USART peripheral

#define USART1_PCLK_EN() 				(RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN() 				(RCC->APB1EN |= (1<<17))
#define USART6_PCLK_EN() 				(RCC->APB2ENR |= (1<<5))

//clock  disENABLE macro for GPIOx peripheral
#define GPIOA_PCLK_DI() 				(RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI() 				(RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI() 				(RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI() 				(RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI() 				(RCC->AHB1ENR &= ~(1<<4))
#define GPIOH_PCLK_DI() 				(RCC->AHB1ENR &= ~(1<<7))


//clock  DISABLE macro for I2Cx peripheral

#define I2C1_PCLK_DI() 					(RCC->AHB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI() 					(RCC->AHB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI() 					(RCC->AHB1ENR &= ~(1<<23))

//clock  DISABLE macro for SPI peripheral

#define SPI1_PCLK_DI() 					(RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI() 					(RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI() 					(RCC->APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DI() 					(RCC->APB2ENR &= ~(1<<13))
#define SPI5_PCLK_DI() 					(RCC->APB2ENR &= ~(1<<20))

//clock  DISABLE macro for USART peripheral

#define USART1_PCLK_DI() 				(RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI() 				(RCC->APB1EN &= ~(1<<17))
#define USART6_PCLK_DI() 				(RCC->APB2ENR &= ~(1<<5))

#define SYSCFG_PLCK_EN()				(RCC->APB2ENR |= ~(1<<14))

/*
 * RESET macro for GPIOx peripheral
 */
#define GPIOA_REG_RESET() 				do{(RCC->AHB1RSTR |= (1<<0));(RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET() 				do{(RCC->AHB1RSTR |= (1<<1));(RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET() 				do{(RCC->AHB1RSTR |= (1<<2));(RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET() 				do{(RCC->AHB1RSTR |= (1<<3));(RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET() 				do{(RCC->AHB1RSTR |= (1<<4));(RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOH_REG_RESET() 				do{(RCC->AHB1RSTR |= (1<<7));(RCC->AHB1RSTR &= ~(1<<7));}while(0)

/*
 * GPIO-BASEADDR to code
 */
#define GPIO_BASEADDR_TO_CODE(x)		(	(x == GPIOA) ? 0:\
											(x == GPIOB) ? 1:\
											(x == GPIOC) ? 2:\
											(x == GPIOD) ? 3:\
											(x == GPIOE) ? 4:\
											(x == GPIOH) ? 5:0 )



/************************************************************************************************
 *  Bit position definition of SPI peripheral
 *****************************************************************************************
 */

/*
 * @SPI_CR1_REG
 */
#define SPI_CR1_CPHA 		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR 		2
#define SPI_CR1_BR			3
#define SPI_CR1_SP			6
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM 		9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

/*
 * @SPI_CR2_REG
 */
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE 		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE 		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

/*
 * @SPI_CR2_SR
 */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE 		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR 		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8


/************************************************************************************************
 *  Bit position definition of I2C peripheral
 *****************************************************************************************
 */

/*
 * @I2C_CR1_REG
 */
#define I2C_CR1_PE 				0
#define I2C_CR1_SMBUS 			1
#define I2C_CR1_SMBTYPE 		3
#define I2C_CR1_ENARP 			4
#define I2C_CR1_ENPEC 			5
#define I2C_CR1_ENGC 			6
#define I2C_CR1_NOSTRETCH 		7
#define I2C_CR1_START 			8
#define I2C_CR1_STOP 			9
#define I2C_CR1_ACK 			10
#define I2C_CR1_POS 			11
#define I2C_CR1_PEC 			12
#define I2C_CR1_ALERT 			13
#define I2C_CR1_SWRST			15

/*
 * @I2C_CR2_REG
 */
#define I2C_CR2_FREQ         0  // Bits [5:0] - Peripheral clock frequency in MHz
#define I2C_CR2_ITERREN      8  // Error interrupt enable
#define I2C_CR2_ITEVTEN      9  // Event interrupt enable
#define I2C_CR2_ITBUFEN     10  // Buffer interrupt enable
#define I2C_CR2_DMAEN       11  // DMA requests enable
#define I2C_CR2_LAST        12  // DMA last transfer

/*
 * @I2C_CR2_REG
 */
#define I2C_CR2_FREQ 		0
#define I2C_CR2_ITERREN 	8
#define I2C_CR2_ITEVTEN	 	9
#define I2C_CR2_ITBUFEN 	10
#define I2C_CR2_DMAEN 		11
#define I2C_CR2_LAST 		12
/*
 * @I2C_SR1_REG
 */
#define I2C_SR1_SB           0  // Start bit (Master mode)
#define I2C_SR1_ADDR         1  // Address sent (master mode) / matched (slave mode)
#define I2C_SR1_BTF          2  // Byte transfer finished
#define I2C_SR1_ADD10        3  // 10-bit header sent (Master mode)
#define I2C_SR1_STOPF        4  // Stop detection (Slave mode)
#define I2C_SR1_RXNE         6  // Data register not empty (Receive)
#define I2C_SR1_TXE          7  // Data register empty (Transmit)
#define I2C_SR1_BERR         8  // Bus error
#define I2C_SR1_ARLO         9  // Arbitration lost (Master mode)
#define I2C_SR1_AF           10 // Acknowledge failure
#define I2C_SR1_OVR          11 // Overrun/Underrun
#define I2C_SR1_PECERR       12 // PEC error in reception
#define I2C_SR1_TIMEOUT      14 // Timeout or Tlow error
#define I2C_SR1_SMBALERT     15 // SMBus alert

/*
 * @I2C_SR2_REG
 */
#define I2C_SR2_MSL           0  // Master/slave
#define I2C_SR2_BUSY          1  // Bus busy
#define I2C_SR2_TRA           2  // Transmitter/receiver
#define I2C_SR2_GENCALL       4  // General call address (slave mode)
#define I2C_SR2_SMBDEFAULT    5  // SMBus device default address (slave mode)
#define I2C_SR2_SMBHOST       6  // SMBus host header (slave mode)
#define I2C_SR2_DUALF         7  // Dual flag (slave mode)
#define I2C_SR2_PEC           8  // Packet error checking register

/*
 * @I2C_CCR_REG
 */
#define I2C_CCR_CCR           0  // Clock control register in Fm/Sm mode (master mode)
#define I2C_CCR_DUTY         14  // Fast mode duty cycle
#define I2C_CCR_FS           15  // I2C master mode selection

//generic marcros
#define ENABLE 							1
#define DISABLE 						0
#define SET 							ENABLE
#define RESET 							DISABLE
#define FLAG_SET						ENABLE
#define FLAG_RESET						DISABLE


#include "stm32f411xx_gpio.h"
#include "stm32f411xx_spi.h"
#include "stm32f411xx_i2c.h"
#endif /* INC_STM32F411XX_H_ */
