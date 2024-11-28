#ifndef LPIT_REG_H
#define LPIT_REG_H

/*!
 * @brief LPIT Control Register shifts.
 *
 * These define the bit shift positions for various control settings
 * within the LPIT registers.
 */
#define LPIT_MCR_M_CEN_SHIFT                     (0u)    /*!< Module Clock Enable shift */
#define LPIT_MCR_DBG_EN_SHIFT                    (3u)    /*!< Debug Mode Enable shift */
#define LPIT_TMR_TCTRL_T_EN_SHIFT                (0u)    /*!< Timer Channel Enable shift */
#define LPIT_TMR_TCTRL_MODE_SHIFT                (2u)    /*!< Timer Channel Mode shift */

/*!
 * @brief Defines the number of LPIT timer channels.
 *
 * The LPIT module has 4 timer channels available for configuration.
 */
#define LPIT_TMR_COUNT                           (4u)    /*!< Number of LPIT Timer Channels */

/*!
 * @brief LPIT Register Layout Type Definition.
 *
 * This structure defines the memory layout for the LPIT peripheral,
 * providing access to control, status, and timer registers.
 */
typedef struct {
    volatile unsigned int VERID;         /*!< Version ID Register */
    volatile unsigned int PARAM;         /*!< Parameter Register */
    volatile unsigned int MCR;           /*!< Module Control Register */
    volatile unsigned int MSR;           /*!< Module Status Register */
    volatile unsigned int MIER;          /*!< Module Interrupt Enable Register */
    volatile unsigned int SETTEN;        /*!< Set Timer Enable Register */
    volatile unsigned int CLRTEN;        /*!< Clear Timer Enable Register */
    unsigned int RESERVED_0;             /*!< Reserved memory space */
    
    /*!
     * @brief Timer Channel Registers.
     *
     * Each LPIT channel has its own set of registers for controlling
     * the timer value, current value, and control operations.
     */
    struct {
        volatile unsigned int TVAL;      /*!< Timer Value Register */
        volatile unsigned int CVAL;      /*!< Current Timer Value Register */
        volatile unsigned int TCTRL;     /*!< Timer Control Register */
        unsigned int RESERVED_0;         /*!< Reserved memory space */
    } TMR[LPIT_TMR_COUNT];               /*!< Timer Channel Array */
} LPIT_Type;

/*!
 * @brief Base address for LPIT0 peripheral.
 *
 * This macro defines the memory address for the LPIT0 peripheral instance.
 */
#define LPIT0_BASE_ADDRESS                              (0x40037000u)

/*!
 * @brief Pointer to the LPIT0 peripheral instance.
 *
 * This macro provides a pointer to the LPIT0 peripheral, allowing access
 * to the registers defined in the LPIT_Type structure.
 */
#define LPIT0                                    ((LPIT_Type *)LPIT0_BASE_ADDRESS)

#endif
