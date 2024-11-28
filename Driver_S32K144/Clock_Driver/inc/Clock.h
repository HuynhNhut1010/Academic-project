#ifndef CLOCK_H
#define CLOCK_H

#include "Clock_Registers.h"

/*! @brief Flags to enable system clocks */
#define SCS_SOSC_ENABLE_FLAG        (1u)   /*!< Enable SOSC clock */
#define SCS_SIRC_ENABLE_FLAG        (2u)   /*!< Enable SIRC clock */
#define SCS_FIRC_ENABLE_FLAG        (3u)   /*!< Enable FIRC clock */
#define SCS_SPLL_ENABLE_FLAG        (6u)   /*!< Enable SPLL clock */

/*! @brief Enumeration for system clock sources */
typedef enum {
    SOSC_CLK = 1u,     /*!< SOSC clock */
    SIRC_CLK = 2u,     /*!< SIRC clock */
    FIRC_CLK = 3u,     /*!< FIRC clock */
    SPLL_CLK = 6u      /*!< SPLL clock */
} system_clock_source_t;

/*! @brief Enumeration for core clock divide ratios */
typedef enum {
    CORE_CLK_DIV_BY_1 = 0,  /*!< Core clock divided by 1 */
    CORE_CLK_DIV_BY_2,      /*!< Core clock divided by 2 */
    CORE_CLK_DIV_BY_3,      /*!< Core clock divided by 3 */
    CORE_CLK_DIV_BY_4,      /*!< Core clock divided by 4 */
    CORE_CLK_DIV_BY_5,      /*!< Core clock divided by 5 */
    CORE_CLK_DIV_BY_6,      /*!< Core clock divided by 6 */
    CORE_CLK_DIV_BY_7,      /*!< Core clock divided by 7 */
    CORE_CLK_DIV_BY_8,      /*!< Core clock divided by 8 */
    CORE_CLK_DIV_BY_9,      /*!< Core clock divided by 9 */
    CORE_CLK_DIV_BY_10,     /*!< Core clock divided by 10 */
    CORE_CLK_DIV_BY_11,     /*!< Core clock divided by 11 */
    CORE_CLK_DIV_BY_12,     /*!< Core clock divided by 12 */
    CORE_CLK_DIV_BY_13,     /*!< Core clock divided by 13 */
    CORE_CLK_DIV_BY_14,     /*!< Core clock divided by 14 */
    CORE_CLK_DIV_BY_15,     /*!< Core clock divided by 15 */
    CORE_CLK_DIV_BY_16      /*!< Core clock divided by 16 */
} core_clock_divide_ratio_t;

/*! @brief Enumeration for bus clock divide ratios */
typedef enum {
    BUS_CLK_DIV_BY_1 = 0,  /*!< Bus clock divided by 1 */
    BUS_CLK_DIV_BY_2,      /*!< Bus clock divided by 2 */
    BUS_CLK_DIV_BY_3,      /*!< Bus clock divided by 3 */
    BUS_CLK_DIV_BY_4,      /*!< Bus clock divided by 4 */
    BUS_CLK_DIV_BY_5,      /*!< Bus clock divided by 5 */
    BUS_CLK_DIV_BY_6,      /*!< Bus clock divided by 6 */
    BUS_CLK_DIV_BY_7,      /*!< Bus clock divided by 7 */
    BUS_CLK_DIV_BY_8,      /*!< Bus clock divided by 8 */
    BUS_CLK_DIV_BY_9,      /*!< Bus clock divided by 9 */
    BUS_CLK_DIV_BY_10,     /*!< Bus clock divided by 10 */
    BUS_CLK_DIV_BY_11,     /*!< Bus clock divided by 11 */
    BUS_CLK_DIV_BY_12,     /*!< Bus clock divided by 12 */
    BUS_CLK_DIV_BY_13,     /*!< Bus clock divided by 13 */
    BUS_CLK_DIV_BY_14,     /*!< Bus clock divided by 14 */
    BUS_CLK_DIV_BY_15,     /*!< Bus clock divided by 15 */
    BUS_CLK_DIV_BY_16      /*!< Bus clock divided by 16 */
} bus_clock_divide_ratio_t;

/*! @brief Enumeration for slow clock divide ratios */
typedef enum {
    SLOW_CLK_DIV_BY_1 = 0,  /*!< Slow clock divided by 1 */
    SLOW_CLK_DIV_BY_2,      /*!< Slow clock divided by 2 */
    SLOW_CLK_DIV_BY_3,      /*!< Slow clock divided by 3 */
    SLOW_CLK_DIV_BY_4,      /*!< Slow clock divided by 4 */
    SLOW_CLK_DIV_BY_5,      /*!< Slow clock divided by 5 */
    SLOW_CLK_DIV_BY_6,      /*!< Slow clock divided by 6 */
    SLOW_CLK_DIV_BY_7,      /*!< Slow clock divided by 7 */
    SLOW_CLK_DIV_BY_8       /*!< Slow clock divided by 8 */
} slow_clock_divide_ratio_t;

/*! @brief Enumeration for peripheral clock names */
typedef enum {
    PCC_FTFC = 32u,   /*!< FTFC clock source */
    PCC_DMAMUX = 33u, /*!< DMAMUX clock source */
    PCC_FlexCAN0 = 36u, /*!< FlexCAN0 clock source */
    PCC_FlexCAN1 = 37u, /*!< FlexCAN1 clock source */
    PCC_FTM3 = 38u,   /*!< FTM3 clock source */
    PCC_ADC1 = 39u,   /*!< ADC1 clock source */
    PCC_FlexCAN2 = 43u, /*!< FlexCAN2 clock source */
    PCC_LPSPI0 = 44u, /*!< LPSPI0 clock source */
    PCC_LPSPI1 = 45u, /*!< LPSPI1 clock source */
    PCC_LPSPI2 = 46u, /*!< LPSPI2 clock source */
    PCC_PDB1 = 49u,   /*!< PDB1 clock source */
    PCC_CRC = 50u,    /*!< CRC clock source */
    PCC_PDB0 = 54u,   /*!< PDB0 clock source */
    PCC_LPIT = 55u,   /*!< LPIT clock source */
    PCC_FTM0 = 56u,   /*!< FTM0 clock source */
    PCC_FTM1 = 57u,   /*!< FTM1 clock source */
    PCC_FTM2 = 58u,   /*!< FTM2 clock source */
    PCC_ADC0 = 59u,   /*!< ADC0 clock source */
    PCC_RTC = 61u,    /*!< RTC clock source */
    PCC_LPTMR0 = 64u, /*!< LPTMR0 clock source */
    PCC_PORTA = 73u,  /*!< PORTA clock source */
    PCC_PORTB = 74u,  /*!< PORTB clock source */
    PCC_PORTC = 75u,  /*!< PORTC clock source */
    PCC_PORTD = 76u,  /*!< PORTD clock source */
    PCC_PORTE = 77u,  /*!< PORTE clock source */
    PCC_SAI0 = 84u,   /*!< SAI0 clock source */
    PCC_SAI1 = 85u,   /*!< SAI1 clock source */
    PCC_FlexIO = 90u, /*!< FlexIO clock source */
    PCC_EWM = 97u,    /*!< EWM clock source */
    PCC_LPI2C0 = 102u, /*!< LPI2C0 clock source */
    PCC_LPI2C1 = 103u, /*!< LPI2C1 clock source */
    PCC_LPUART0 = 106u, /*!< LPUART0 clock source */
    PCC_LPUART1 = 107u, /*!< LPUART1 clock source */
    PCC_LPUART2 = 108u, /*!< LPUART2 clock source */
    PCC_FTM4 = 110u,  /*!< FTM4 clock source */
    PCC_FTM5 = 111u,  /*!< FTM5 clock source */
    PCC_FTM6 = 112u,  /*!< FTM6 clock source */
    PCC_FTM7 = 113u,  /*!< FTM7 clock source */
    PCC_CMP0 = 115u,  /*!< CMP0 clock source */
    PCC_QSPI = 118u,  /*!< QSPI clock source */
    PCC_ENET = 121u   /*!< ENET clock source */
} clock_names_t;

/*! @brief Enumeration for peripheral clock sources */
typedef enum {
    CLK_SRC_OFF = 0u, /*!< Clock is off */
    CLK_SRC_OP_1 = 1u, /*!< Clock option 1 */
    CLK_SRC_OP_2 = 2u, /*!< Clock option 2 */
    CLK_SRC_OP_3 = 3u, /*!< Clock option 3 */
    CLK_SRC_OP_4 = 4u, /*!< Clock option 4 */
    CLK_SRC_OP_5 = 5u, /*!< Clock option 5 */
    CLK_SRC_OP_6 = 6u, /*!< Clock option 6 */
    CLK_SRC_OP_7 = 7u  /*!< Clock option 7 */
} peripheral_clock_source_t;

/*! @brief Enumeration for clock gate control */
typedef enum {
    CLK_GATE_DISABLE = 0u, /*!< Clock gate disabled */
    CLK_GATE_ENABLE = 1u   /*!< Clock gate enabled */
} clock_gate_t;

/*! @brief Peripheral clock configuration structure */
typedef struct {
    clock_names_t clockName;         /*!< Peripheral clock name */
    clock_gate_t clkGate;            /*!< Peripheral clock gate */
    peripheral_clock_source_t clkSrc; /*!< Peripheral clock source */
} Pcc_ConfigType;

/*! @brief Enumeration for SCG asynchronous clock dividers */
typedef enum {
    SCG_CLOCK_DISABLE = 0U,  /*!< Clock output disabled */
    SCG_CLOCK_DIV_BY_1 = 1U, /*!< Divided by 1 */
    SCG_CLOCK_DIV_BY_2 = 2U, /*!< Divided by 2 */
    SCG_CLOCK_DIV_BY_4 = 3U, /*!< Divided by 4 */
    SCG_CLOCK_DIV_BY_8 = 4U, /*!< Divided by 8 */
    SCG_CLOCK_DIV_BY_16 = 5U, /*!< Divided by 16 */
    SCG_CLOCK_DIV_BY_32 = 6U, /*!< Divided by 32 */
    SCG_CLOCK_DIV_BY_64 = 7U  /*!< Divided by 64 */
} scg_async_clock_div_t;

/*! @brief SCG FIRC configuration structure */
typedef struct {
    scg_async_clock_div_t div1; /*!< Clock Divider 1 */
    scg_async_clock_div_t div2; /*!< Clock Divider 2 */
} Scg_Firc_ConfigType;

/*! @brief SCG SIRC configuration structure */
typedef struct {
    scg_async_clock_div_t div1; /*!< Clock Divider 1 */
    scg_async_clock_div_t div2; /*!< Clock Divider 2 */
} Scg_Sirc_ConfigType;

/*! @brief SCG SOSC configuration structure */
typedef struct {
    scg_async_clock_div_t div1; /*!< Clock Divider 1 */
    scg_async_clock_div_t div2; /*!< Clock Divider 2 */
} Scg_Sosc_ConfigType;

/*! @brief SCG SPLL configuration structure */
typedef struct {
    unsigned char prediv;        /*!< PLL reference clock divider */
    unsigned char mult;          /*!< System PLL multiplier */
    unsigned char src;           /*!< System PLL source */
    scg_async_clock_div_t div1;  /*!< Clock Divider 1 */
    scg_async_clock_div_t div2;  /*!< Clock Divider 2 */
} Scg_Spll_ConfigType;

/*! @brief SCG Run Mode configuration structure */
typedef struct {
    system_clock_source_t sys_clk_src; /*!< System clock source */
    core_clock_divide_ratio_t core_div; /*!< Core clock divide ratio */
    bus_clock_divide_ratio_t bus_div;   /*!< Bus clock divide ratio */
    slow_clock_divide_ratio_t slow_div; /*!< Slow clock divide ratio */
    scg_async_clock_div_t div1;         /*!< Clock Divider 1 */
    scg_async_clock_div_t div2;         /*!< Clock Divider 2 */
} Scg_RunMode_ConfigType;

/*! @brief Function to set PCC configuration */
void Clock_SetPccConfig(const Pcc_ConfigType* ConfigPtr);

/*! @brief Function to set SCG FIRC configuration */
void Clock_SetScgFircConfig(const Scg_Firc_ConfigType * ConfigPtr);

/*! @brief Function to set SCG SIRC configuration */
void Clock_SetScgSircConfig(const Scg_Sirc_ConfigType * ConfigPtr);

/*! @brief Function to set SCG SOSC configuration */
void Clock_SetScgSoscConfig(const Scg_Sosc_ConfigType * ConfigPtr);

/*! @brief Function to set SCG SPLL configuration */
void Clock_SetScgSpllConfig(const Scg_Spll_ConfigType * ConfigPtr);

/*! @brief Function to set SCG Run Mode configuration */
void Clock_SetScgRunModeConfig(const Scg_RunMode_ConfigType * ConfigPtr);

#endif
