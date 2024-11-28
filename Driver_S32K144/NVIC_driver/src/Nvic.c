#include "Nvic.h"

/*!
 * @brief Enables the specified interrupt in the NVIC.
 *
 * This function enables the interrupt associated with the specified IRQ number by
 * setting the corresponding bit in the Interrupt Set Enable Register (ISER).
 *
 * @param[in] IRQ_number The IRQ number to enable in the NVIC.
 * @return none
 */
void NVIC_EnableInterrupt(IRQn_Type IRQ_number)
{
	unsigned char temp1 = IRQ_number / 32;
	unsigned char temp2 = IRQ_number % 32;
	NVIC->ISER[temp1] = (1 << temp2);
}

/*!
 * @brief Disables the specified interrupt in the NVIC.
 *
 * This function disables the interrupt associated with the specified IRQ number by
 * clearing the corresponding bit in the Interrupt Clear Enable Register (ICER).
 *
 * @param[in] IRQ_number The IRQ number to disable in the NVIC.
 * @return none
 */
void NVIC_DisableInterrupt(IRQn_Type IRQ_number)
{
	NVIC->ICER[IRQ_number/32] = (1 << IRQ_number%32);
}

/*!
 * @brief Clears the pending flag for the specified interrupt.
 *
 * This function clears the pending interrupt flag associated with the specified IRQ number
 * by setting the corresponding bit in the Interrupt Clear Pending Register (ICPR).
 *
 * @param[in] IRQ_number The IRQ number to clear the pending flag for in the NVIC.
 * @return none
 */
void NVIC_ClearPendingFlag(IRQn_Type IRQ_number)
{
	NVIC->ICPR[IRQ_number/32] = (1 << IRQ_number%32);
}

/*!
 * @brief Sets the priority for the specified interrupt.
 *
 * This function assigns the priority level for the interrupt associated with the
 * specified IRQ number. The priority is shifted and placed in the Interrupt Priority Register (IPR).
 *
 * @param[in] IRQ_number The IRQ number to set the priority for in the NVIC.
 * @param[in] priority The priority value to assign to the interrupt (lower value = higher priority).
 * @return none
 */
void NVIC_SetPriority(IRQn_Type IRQ_number, unsigned char priority)
{
	unsigned char temp = (unsigned char)(priority << 4);
	NVIC->IPR[(unsigned char)IRQ_number/4] = (unsigned int)(temp << (IRQ_number%4)*8);
}
