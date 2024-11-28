#ifndef LPIT_H
#define LPIT_H

#include "Lpit_Register.h"
#include "stddef.h"

/*!
 * @brief Structure to hold configuration parameters for a LPIT channel.
 *
 * This structure contains the timer channel period and interrupt 
 * enablement status.
 */
typedef struct
{
    unsigned int period;                        /*!< Period of timer channel (in timer ticks) */
    unsigned char isInterruptEnabled;           /*!< Timer channel interrupt generation enable (0: disabled, 1: enabled) */
} Lpit_ChannelConfigType;

/*!
 * @brief Initializes the LPIT module.
 *
 * This function configures the LPIT module, enabling debug mode and
 * the peripheral clock to the timers.
 */
void Lpit_Init( void );

/*!
 * @brief Initializes a specific LPIT channel with the provided configuration.
 *
 * This function sets up the operation mode of the timer channel and
 * configures the period and interrupt enablement for the specified channel.
 *
 * @param[in] channel The LPIT channel number (0-3)
 * @param[in] ConfigPtr Pointer to the channel configuration structure
 */
void Lpit_InitChannel(unsigned char channel, const Lpit_ChannelConfigType* ConfigPtr);

/*!
 * @brief Starts the specified LPIT channel.
 *
 * This function enables counting for the specified timer channel.
 *
 * @param[in] channel The LPIT channel number (0-3)
 */
void Lpit_StartChannel(unsigned char channel);

/*!
 * @brief Stops the specified LPIT channel.
 *
 * This function stops the counting of the specified timer channel.
 *
 * @param[in] channel The LPIT channel number (0-3)
 */
void Lpit_StopChannel(unsigned char channel);

/*!
 * @brief Gets the current counter value of the specified LPIT channel.
 *
 * This function returns the current value of the timer counter for the
 * specified channel.
 *
 * @param[in] channel The LPIT channel number (0-3)
 * @return The current counter value, or 0 if the channel is invalid.
 */
unsigned int Lpit_GetCounterChannel(unsigned char channel);

#endif
