#ifndef LPSPI_REG_H
#define LPSPI_REG_H

/* Define bit shifts for various fields in the TCR (Transmit Control Register) */
#define LPSPI_TCR_CPOL_SHIFT                     (31u)   /* Clock Polarity shift */
#define LPSPI_TCR_CPHA_SHIFT                     (30u)   /* Clock Phase shift */
#define LPSPI_TCR_PRESCALE_SHIFT                 (27u)   /* Prescaler shift */
#define LPSPI_TCR_PCS_SHIFT                      (24u)   /* Peripheral Chip Select shift */
#define LPSPI_TCR_LSBF_SHIFT                     (23u)   /* LSB First shift */
#define LPSPI_TCR_FRAME_SHIFT                    (0u)    /* Frame Size shift */

/** 
 * LPSPI - Register Layout Typedef
 * This typedef represents the structure layout of the LPSPI registers.
 * It includes all the necessary registers for configuring, transmitting,
 * and receiving data over the LPSPI interface.
 */
typedef struct {
  volatile const  unsigned int VERID;  /* Version ID register (read-only) */
  volatile const  unsigned int PARAM;  /* Parameter register (read-only) */
  unsigned char RESERVED_0[8];         /* Reserved memory space */
  volatile unsigned int CR;            /* Control register */
  volatile unsigned int SR;            /* Status register */
  volatile unsigned int IER;           /* Interrupt Enable register */
  volatile unsigned int DER;           /* DMA Enable register */
  volatile unsigned int CFGR0;         /* Configuration Register 0 */
  volatile unsigned int CFGR1;         /* Configuration Register 1 */
  unsigned char RESERVED_1[8];         /* Reserved memory space */
  volatile unsigned int DMR0;          /* Data Match Register 0 */
  volatile unsigned int DMR1;          /* Data Match Register 1 */
  unsigned char RESERVED_2[8];         /* Reserved memory space */
  volatile unsigned int CCR;           /* Clock Configuration Register */
  unsigned char RESERVED_3[20];        /* Reserved memory space */
  volatile unsigned int FCR;           /* FIFO Control Register */
  volatile const  unsigned int FSR;    /* FIFO Status Register (read-only) */
  volatile unsigned int TCR;           /* Transmit Command Register */
  volatile unsigned int TDR;           /* Transmit Data Register */
  unsigned char RESERVED_4[8];         /* Reserved memory space */
  volatile const  unsigned int RSR;    /* Receive Status Register (read-only) */
  volatile const  unsigned int RDR;    /* Receive Data Register (read-only) */
} LPSPI_Type;

/* Define base address for LPSPI0 peripheral */
#define LPSPI0_base_address  		(0x4002C000u)
/* Define LPSPI0 as a pointer to the LPSPI register structure */
#define LPSPI0                  ((LPSPI_Type *)LPSPI0_base_address)

/* Define base address for LPSPI1 peripheral */
#define LPSPI1_base_address  		(0x4002D000u)
/* Define LPSPI1 as a pointer to the LPSPI register structure */
#define LPSPI1                	((LPSPI_Type *)LPSPI1_base_address)

/* Define base address for LPSPI2 peripheral */
#define LPSPI2_base_address  		(0x4002E000u)
/* Define LPSPI2 as a pointer to the LPSPI register structure */
#define LPSPI2                  ((LPSPI_Type *)LPSPI2_base_address)

#endif /* LPSPI_REG_H */
