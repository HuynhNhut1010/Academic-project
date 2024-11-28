#include "Lpspi.h"

/*!
 * @brief Initializes the LPSPI module with the given configuration
 *
 * This function configures the LPSPI registers based on the values in the provided configuration structure.
 * It sets up clock polarity, clock phase, frame size, chip select, and prescaler, and enables the LPSPI module.
 *
 * @param[in] ConfigPtr Pointer to the Lpspi_ConfigType structure with configuration settings
 */
void Lpspi_Init(const Lpspi_ConfigType* ConfigPtr)
{
    unsigned int SCK_diver = 0;
    unsigned int TCR_value = 0;
    unsigned int FCR_value = 0;
    unsigned char Error_flag = 0;

    /* Step 1. Check parameter */
    if (    ConfigPtr->Init.F_lpspi < 1 ||
            ConfigPtr->Init.spi_cpha > 1 ||
            ConfigPtr->Init.spi_cpol > 1 ||
            (ConfigPtr->Init.spi_frame != LPSPI_FRAME_8  && 
             ConfigPtr->Init.spi_frame != LPSPI_FRAME_16 && 
             ConfigPtr->Init.spi_frame != LPSPI_FRAME_32) ||
            ConfigPtr->Init.spi_pcs > 3 ||
            ConfigPtr->Init.spi_prescaler > 8 ||
            ConfigPtr->Init.spi_speed < 1 ||
            ConfigPtr->Init.spi_type_tran > 1
       )
    {
        Error_flag = 1;
    }

    if (Error_flag == 0)
    {
        /* Step 2. Set divide ratio of the SCK pin */
        SCK_diver = (ConfigPtr->Init.F_lpspi / (1 << ConfigPtr->Init.spi_prescaler) / ConfigPtr->Init.spi_speed) - 2;
        ConfigPtr->pSPIx->CCR |= (SCK_diver);

        /* Step 3. Configure Clock Phase, Clock Polarity, Prescaler, Frame Size, Transfer Type, and PCS */
        TCR_value = (unsigned int)((ConfigPtr->Init.spi_cpha << LPSPI_TCR_CPHA_SHIFT) |
                                   (ConfigPtr->Init.spi_cpol << LPSPI_TCR_CPOL_SHIFT) |
                                   (ConfigPtr->Init.spi_prescaler << LPSPI_TCR_PRESCALE_SHIFT) |
                                   (ConfigPtr->Init.spi_frame << LPSPI_TCR_FRAME_SHIFT) |
                                   (ConfigPtr->Init.spi_type_tran << LPSPI_TCR_LSBF_SHIFT) |
                                   (ConfigPtr->Init.spi_pcs << LPSPI_TCR_PCS_SHIFT));
        ConfigPtr->pSPIx->TCR = TCR_value;

        /* Step 7. Set Transmit/Receive FIFO settings */
        FCR_value = (0 << 16) | (3 << 0);
        ConfigPtr->pSPIx->FCR = FCR_value;

        /* Step 8. Configure LPSPI mode */
        /* Master mode */
        if (ConfigPtr->Init.spi_mode == LPSPI_MODE_MASTER)
        {
            ConfigPtr->pSPIx->CFGR1 |= (1 << 3) | (1 << 0);
        }
        else
        {
            ConfigPtr->pSPIx->CFGR1 &= (unsigned int)~(1 << 0);
        }

        /* Step 9. Enable LPSPI module */
        ConfigPtr->pSPIx->CR |= (1 << 3);
        ConfigPtr->pSPIx->CR |= (1 << 0);
    }
}

/*!
 * @brief Transmits data using LPSPI
 *
 * This function sends data through the LPSPI module. It waits until the transmit buffer
 * is empty and then loads data into the transmit data register (TDR). It handles data sizes
 * of 8, 16, or 32 bits.
 *
 * @param[in] pLpspi Pointer to the LPSPI_Type structure representing the LPSPI instance
 * @param[in] pTxBuffer Pointer to the data buffer that needs to be transmitted
 * @param[in] Size Size of the data to be transmitted
 */
void Lpspi_Transmit(LPSPI_Type *pLpspi, unsigned char *pTxBuffer, unsigned short Size)
{
    unsigned short data = 0;

    while (Size > 0)
    {
        /* Wait for transmit buffer to be empty */
        while (~(pLpspi->SR >> 0) & 0x01);

        /* Check frame size and send data accordingly */
        if ((pLpspi->TCR & 0xFFF) == 15)
        {
            /* If frame size is 16 bits */
            /* Load data to TDR */
            pLpspi->TDR = *((unsigned short*)pTxBuffer);
            data = *((unsigned short*)pTxBuffer);

            /* Increment buffer address */
            pTxBuffer += 2;
            Size -= 2;
        }
        else if ((pLpspi->TCR & 0xFFF) == 7)
        {
            /* If frame size is 8 bits */
            pLpspi->TDR = *pTxBuffer;
            pTxBuffer++;
            Size--;
        } 
        else 
        {
            /* If frame size is 32 bits */
            pLpspi->TDR = *((unsigned int*)pTxBuffer);
            pTxBuffer += 4;
            Size -= 4;
        }
    }
}
