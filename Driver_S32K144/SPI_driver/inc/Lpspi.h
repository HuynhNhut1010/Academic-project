#include "Lpspi_Register.h"

/*!
 * @brief Enumeration for LPSPI operating modes.
 *
 * This enum defines the LPSPI mode as either master or slave.
 */
typedef enum
{
    LPSPI_MODE_SLAVE  	= 0U,  /*!< LPSPI in Slave mode */
    LPSPI_MODE_MASTER  	= 1U,   /*!< LPSPI in Master mode */
} spi_mode_t;

/*!
 * @brief Enumeration for LPSPI clock polarity (CPOL).
 *
 * This enum defines the idle state of the clock signal.
 */
typedef enum
{
    LPSPI_CPOL_0  = 0U,  /*!< Clock idle low */
    LPSPI_CPOL_1  = 1U,   /*!< Clock idle high */
} spi_clock_polarity_t;

/*!
 * @brief Enumeration for LPSPI clock phase (CPHA).
 *
 * This enum defines the clock phase for data sampling.
 */
typedef enum
{
    LPSPI_CPHA_0  = 0U,  /*!< Data sampled on the first clock edge */
    LPSPI_CPHA_1  = 1U   /*!< Data sampled on the second clock edge */
} spi_clock_phase_t;

/*!
 * @brief Enumeration for LPSPI Peripheral Chip Select (PCS).
 *
 * This enum defines the chip select pin for the SPI peripheral.
 */
typedef enum
{
    LPSPI_PCS_0  = 0U,  /*!< Chip Select 0 */
    LPSPI_PCS_1  = 1U,  /*!< Chip Select 1 */
	LPSPI_PCS_2  = 2U,  /*!< Chip Select 2 */
    LPSPI_PCS_3  = 3U   /*!< Chip Select 3 */
} spi_peripheral_chip_select_t;

/*!
 * @brief Enumeration for LPSPI frame size.
 *
 * This enum defines the frame size for SPI data transfers.
 */
typedef enum
{
    LPSPI_FRAME_8  	= 7U,  /*!< 8-bit frame size */
    LPSPI_FRAME_16  = 15U,  /*!< 16-bit frame size */
	LPSPI_FRAME_32  = 31U,   /*!< 32-bit frame size */
} spi_frame_size_t;

/*!
 * @brief Enumeration for LPSPI data transfer type (MSB/LSB).
 *
 * This enum defines whether data is transferred LSB first or MSB first.
 */
typedef enum
{
    LPSPI_MSB_FIRST  = 0U,  /*!< Most Significant Bit (MSB) first */
    LPSPI_LSB_FIRST  = 1U   /*!< Least Significant Bit (LSB) first */
} spi_type_transfer_t;

/*!
 * @brief Enumeration for LPSPI clock prescaler.
 *
 * This enum defines the prescaler that divides the clock by powers of 2.
 */
typedef enum
{
    LPSPI_PRE_DIV_BY_1   = 0U,  /*!< Divide clock by 1 */
    LPSPI_PRE_DIV_BY_2   = 1U,  /*!< Divide clock by 2 */
    LPSPI_PRE_DIV_BY_4   = 2U,  /*!< Divide clock by 4 */
    LPSPI_PRE_DIV_BY_8   = 3U,  /*!< Divide clock by 8 */
    LPSPI_PRE_DIV_BY_16  = 4U,  /*!< Divide clock by 16 */
    LPSPI_PRE_DIV_BY_32  = 5U,  /*!< Divide clock by 32 */
    LPSPI_PRE_DIV_BY_64  = 6U,  /*!< Divide clock by 64 */
    LPSPI_PRE_DIV_BY_128 = 7U   /*!< Divide clock by 128 */
} spi_prescaler_t;

/*!
 * @brief Structure for LPSPI initialization parameters.
 *
 * This structure holds the configuration parameters for initializing the LPSPI module.
 */
typedef struct
{
		unsigned int                  F_lpspi;      /*!< Clock supply for LPSPI */
		unsigned int                  spi_speed;    /*!< SPI communication speed */
		unsigned char 								rx_water;    /*!< RX FIFO watermark level */
		unsigned char 								tx_water;    /*!< TX FIFO watermark level */
		spi_mode_t 										spi_mode;    /*!< Operating mode (Master/Slave) */
		spi_prescaler_t               spi_prescaler; /*!< SPI clock prescaler */
		spi_type_transfer_t     	  	spi_type_tran; /*!< Transfer type (MSB/LSB first) */
		spi_frame_size_t              spi_frame;    /*!< Frame size for SPI transfer */
		spi_clock_polarity_t          spi_cpol;     /*!< Clock polarity */
		spi_clock_phase_t             spi_cpha;     /*!< Clock phase */
		spi_peripheral_chip_select_t  spi_pcs;      /*!< Peripheral Chip Select (PCS) */
		unsigned char 								padding[3];
		
} LPUART_InitType;

/*!
 * @brief Structure for LPSPI configuration.
 *
 * This structure contains the configuration and state variables for the LPSPI module.
 */
typedef struct
{
	LPSPI_Type *pSPIx;               /*!< Pointer to the LPSPI peripheral */
	LPUART_InitType Init;            /*!< LPSPI initialization parameters */
	unsigned char *pTxBuffer;        /*!< Pointer to the transmit buffer */
	unsigned char *pRxBuffer;        /*!< Reserved pointer to the receive buffer */
	unsigned short TxLen;            /*!< Transmit buffer length */
	unsigned short RxLen;	        /*!< Reserved receive buffer length */
	
} Lpspi_ConfigType;

/*!
 * @brief Initializes the LPSPI module with the provided configuration.
 *
 * This function configures the LPSPI module using the values provided in the
 * `Lpspi_ConfigType` structure.
 *
 * @param[in] ConfigPtr Pointer to the configuration structure
 */
void Lpspi_Init (const Lpspi_ConfigType* ConfigPtr);

/*!
 * @brief Transmits data using the LPSPI module.
 *
 * This function transmits data via the LPSPI peripheral. It sends data from
 * the `pTxBuffer` buffer of length `Size`.
 *
 * @param[in] pLpspi Pointer to the LPSPI peripheral
 * @param[in] pTxBuffer Pointer to the transmit data buffer
 * @param[in] Size Number of bytes to transmit
 */
void Lpspi_Transmit(LPSPI_Type *pLpspi, unsigned char *pTxBuffer, unsigned short Size);
