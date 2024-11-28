#ifndef NVIC_H
#define NVIC_H

#include "Nvic_Registers.h"

typedef enum
{
  DMA_0_IRQn                   = 0u,
  LPUART0_RxTx_IRQn            = 31u,              /**< LPUART0 Transmit / Receive  Interrupt */
	LPUART1_RxTx_IRQn            = 33u,              /**< LPUART1 Transmit / Receive  Interrupt */
	LPUART2_RxTx_IRQn            = 35u,              /**< LPUART2 Transmit / Receive  Interrupt */
	ADC0_IRQn										 = 39u,							 /**< ADC0 Interrupt */
	ADC1_IRQn										 = 40u,							 /**< ADC1 Interrupt */
	LIPT_C0_IRQn								 = 48u,							 /**< LIPT_C0 channel 0 */
	LIPT_C1_IRQn								 = 49u,							 /**< LIPT_C0 channel 1 */
	LIPT_C2_IRQn								 = 50u,							 /**< LIPT_C0 channel 2 */
	LIPT_C3_IRQn								 = 51u,							 /**< LIPT_C0 channel 3 */
  PORTA_IRQn                   = 59u,              /**< Port A pin detect interrupt */
  PORTB_IRQn                   = 60u,              /**< Port B pin detect interrupt */
  PORTC_IRQn                   = 61u,              /**< Port C pin detect interrupt */
  PORTD_IRQn                   = 62u,              /**< Port D pin detect interrupt */
  PORTE_IRQn                   = 63u,              /**< Port E pin detect interrupt */
	
} IRQn_Type;


/************************************************
*
* 						Define API for NVIC driver
*
************************************************/
void NVIC_EnableInterrupt(IRQn_Type IRQ_number);
void NVIC_DisableInterrupt(IRQn_Type IRQ_number);
void NVIC_ClearPendingFlag(IRQn_Type IRQ_number);
void NVIC_SetPriority(IRQn_Type IRQ_number, unsigned char priority);

#endif
