#include "Clock.h"

/*!
 * @brief Configures the Peripheral Clock Control (PCC) register.
 *
 * This function disables the peripheral clock, configures the clock source,
 * and enables the clock if the gate is enabled in the provided configuration.
 *
 * @param[in] ConfigPtr Pointer to the PCC configuration structure
 * @return None
 */
void Clock_SetPccConfig(const Pcc_ConfigType* ConfigPtr)
{
    /* 1. Disable the peripheral clock */
    PCC->PCCn[ConfigPtr->clockName] &= ~(1u << PCC_CGC_SHIFT);

    /* 2. Check whether Clock Gate Control is enabled or disabled */
    if (ConfigPtr->clkGate == CLK_GATE_ENABLE)
    {
        /* 2.1. Check if the Peripheral Clock Source Select is configured */
        if (ConfigPtr->clkSrc != CLK_SRC_OFF)
        {
            /* Set Peripheral Clock Source Select */
            PCC->PCCn[ConfigPtr->clockName] |= ((unsigned int)ConfigPtr->clkSrc << PCC_PCS_SHIFT);
        }

        /* 2.2. Enable the peripheral clock */
        PCC->PCCn[ConfigPtr->clockName] |= (1u << PCC_CGC_SHIFT);
    }
}

/*!
 * @brief Configures the Fast Internal Reference Clock (FIRC) settings.
 *
 * This function sets up the dividers for the FIRC based on the provided configuration.
 *
 * @param[in] ConfigPtr Pointer to the FIRC configuration structure
 * @return None
 */
void Clock_SetScgFircConfig(const Scg_Firc_ConfigType * ConfigPtr)
{
    /* Step 1. Setup divider 1 */
    SCG->FIRCDIV |= ((unsigned int)(ConfigPtr->div1) << SCG_FIRCDIV_FIRCDIV1_SHIFT);

    /* Step 2. Setup divider 2 */
    SCG->FIRCDIV |= ((unsigned int)(ConfigPtr->div2) << SCG_FIRCDIV_FIRCDIV2_SHIFT);
}

/*!
 * @brief Configures the Slow Internal Reference Clock (SIRC) settings.
 *
 * This function sets up the dividers for the SIRC based on the provided configuration.
 *
 * @param[in] ConfigPtr Pointer to the SIRC configuration structure
 * @return None
 */
void Clock_SetScgSircConfig(const Scg_Sirc_ConfigType * ConfigPtr)
{
    /* Step 1. Setup divider 1 */
    SCG->SIRCDIV |= ((unsigned int)(ConfigPtr->div1) << SCG_FIRCDIV_FIRCDIV1_SHIFT);

    /* Step 2. Setup divider 2 */
    SCG->SIRCDIV |= ((unsigned int)(ConfigPtr->div2) << SCG_FIRCDIV_FIRCDIV2_SHIFT);
}

/*!
 * @brief Configures the System Oscillator (SOSC) settings.
 *
 * This function sets up the dividers, configuration, and control for the SOSC 
 * and waits for its initialization.
 *
 * @param[in] ConfigPtr Pointer to the SOSC configuration structure
 * @return None
 */
void Clock_SetScgSoscConfig(const Scg_Sosc_ConfigType * ConfigPtr)
{
    /* Step 1. Setup divider 1 */
    SCG->SOSCDIV |= ((unsigned int)(ConfigPtr->div1) << SCG_SOSCDIV_SOSCDIV1_SHIFT);

    /* Step 2. Setup divider 2 */
    SCG->SOSCDIV |= ((unsigned int)(ConfigPtr->div2) << SCG_SOSCDIV_SOSCDIV2_SHIFT);

    /* Step 3. Set SOSC configuration */
    SCG->SOSCCFG = (2u << SCG_SOSCCFG_RANGE_SHIFT) | (1u << SCG_SOSCCFG_EREFS_SHIFT);

    /* Step 4. Clear Lock Register */
    SCG->SOSCCSR &= ~((unsigned int)(1 << SCG_CSR_LK_SHIFT));

    /* Step 5. Enable SOSC clock */
    SCG->SOSCCSR = 1u;

    /* Step 6. Wait for System OSC to initialize */
    while (!((SCG->SOSCCSR >> SCG_SOSCCSR_SOSCVLD_SHIFT) & 0x01));
}

/*!
 * @brief Configures the System PLL (SPLL) settings.
 *
 * This function disables the SPLL, sets up dividers, configures the PLL, and
 * enables the SPLL.
 *
 * @param[in] ConfigPtr Pointer to the SPLL configuration structure
 * @return None
 */
void Clock_SetScgSpllConfig(const Scg_Spll_ConfigType * ConfigPtr)
{
    /* Step 1. Disable SPLL */
    SCG->SPLLCSR &= ~((unsigned int)(1 << SCG_SPLLDIV_SPLLEN_SHIFT));

    /* Step 2. Setup divider 1 */
    SCG->SPLLDIV |= ((unsigned int)(ConfigPtr->div1) << SCG_SPLLDIV_SPLLDIV1_SHIFT);

    /* Step 3. Setup divider 2 */
    SCG->SPLLDIV |= ((unsigned int)(ConfigPtr->div2) << SCG_SPLLDIV_SPLLDIV2_SHIFT);

    /* Step 4. Set PLL configuration */
    if (ConfigPtr->src == 1)
    {
        SCG->SPLLCFG = (0x01 << 0);
    }
    else
    {
        SCG->SPLLCFG &= ~(unsigned int)(0x01 << 0);
    }
    SCG->SPLLCFG |= (unsigned int)((ConfigPtr->mult) << 16);
    SCG->SPLLCFG |= (unsigned int)((ConfigPtr->prediv) << 8);

    /* Step 5. Clear Lock Register */
    SCG->SPLLCSR &= (unsigned int)(~(1 << 23));

    /* Step 6. Enable SPLL */
    SCG->SPLLCFG = 1u;

    /* Step 7. Wait for SPLL to initialize */
    while (!((SCG->SPLLCFG >> SCG_SPLLCFG_SPLLVLD_SHIFT) & 0x01));
}

/*!
 * @brief Configures the system clock for the RUN mode.
 *
 * This function sets the system clock source, core, bus, and slow dividers for the RUN mode.
 *
 * @param[in] ConfigPtr Pointer to the RUN mode clock configuration structure
 * @return None
 */
void Clock_SetScgRunModeConfig(const Scg_RunMode_ConfigType * ConfigPtr)
{
    /* Step 1. Set the RUN clock control (system clock source, bus, core, and slow dividers) */
    unsigned int value = 0;

    /* Step 1.1. Set system clock source */
    value |= ((unsigned int)ConfigPtr->sys_clk_src << SCG_RCCR_SCS_SHIFT);

    /* Step 1.2. Set Core Clock Divide Ratio */
    value |= ((unsigned int)ConfigPtr->core_div << SCG_RCCR_DIVCORE_SHIFT);

    /* Step 1.3. Set Bus Clock Divide Ratio */
    value |= ((unsigned int)ConfigPtr->sys_clk_src << SCG_RCCR_DIVBUS_SHIFT);

    /* Step 1.4. Set Slow Clock Divide Ratio */
    value |= ((unsigned int)ConfigPtr->slow_div << SCG_RCCR_DIVSLOW_SHIFT);

    SCG->RCCR = value;

    /* Step 2. Confirm the system clock source is set as configured */
}
