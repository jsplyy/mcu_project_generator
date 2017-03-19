/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Copyright (c) 2016, NXP Semiconductors, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _FSL_CS_H_
#define _FSL_CS_H_

#include "fsl_common.h"

/*!
 * @addtogroup qn_cs
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
#define FSL_CS_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Version 2.0.0 */
                                                      /*@}*/

/*!
 * @brief CS mode
 */
typedef enum _cs_mode
{
    kCS_ActiveMode = 0U,
    kCS_WakeupMode,
} cs_mode_t;

/*!
 * @brief CS configuration
 */
typedef struct _cs_config
{
    uint16_t activeChannelEnable; /*!< Channel enable mask */
    uint16_t activeClockDivider;  /*!< Clock divider, occupy 9 bits.
                                 F(CS) = F(APB) / (clockDivider+1) */
    uint16_t activeDetectPeriod;  /*!< Detect period, occupy 16 bits */
    uint16_t activeIdlePeriod; /*!< Idle period between previous scanning end and next scanning start, occupy 16 bits */
    uint16_t activeOscFreq;    /*!< OSC bias select, occupy 6 bits */

    uint8_t lowPowerChannelNum;
    uint16_t lowPowerThreshold;
    uint16_t lowPowerDebonceNum;
    uint16_t
        lowPowerIdlePeriod;   /*!< Idle period between previous scanning end and next scanning start, occupy 16 bits */
    uint16_t lowPowerOscFreq; /*!< OSC bias select, occupy 6 bits */
} cs_config_t;

/*!
 * @brief interrupts
 */
enum _cs_interrupt_enable
{
    kCS_InterruptFifoNotFullEnable = CS_INTEN_FIFO_NOTEMPTY_INTEN_MASK, /*!< Enable Fifo not empty interrupt */
    kCS_InterruptFifoHalfFullEnable = CS_INTEN_FIFO_HFULL_INTEN_MASK,   /*!< Enable Fifo half full interrupt */
    kCS_InterruptFifoFullEnable = CS_INTEN_FIFO_FULL_INTEN_MASK,        /*!< Enable Fifo full interrupt */
    kCS_InterruptScanCompleteEnable = CS_INTEN_SCAN_INTEN_MASK,         /*!< Enable Scan complete interrupt */
};

/*!
 * @brief Flags
 */
enum _cs_status_flags
{
    kCS_InterruptFifoNotFullFlag = CS_INT_FIFO_NOTEMPTY_INT_MASK, /*!< Fifo not empty interrupt flag */
    kCS_InterruptFifoHalfFullFlag = CS_INT_FIFO_HFULL_INT_MASK,   /*!< Fifo half full interrupt flag */
    kCS_InterruptFifoFullFlag = CS_INT_FIFO_FULL_INT_MASK,        /*!< Fifo full interrupt flag */
    kCS_InterruptScanCompleteFlag = CS_INT_SCAN_INT_MASK,         /*!< Scan complete interrupt flag */
};

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Initialization and deinitialization
 * @{
 */

/*!
 * @brief Initialize the CS module.
 *
 * @param base CS peripheral base address.
 * @param config Pointer to configuration structure, see to cs_config_t.
 */
void CS_Init(CS_Type *base, const cs_config_t *config);

/*!
 * @brief Deinitialize the CS module.
 *
 * @param base CS peripheral base address.
 */
void CS_DeInit(CS_Type *base);

/*!
 * @brief Gets an available pre-defined settings for initial configuration.
 *
 * This function initializes the initial configuration structure with an available settings. The default values are:
 * @code
 *  config->activeChannelEnable = CS_ACTIVE_CHANNEL_ENABLE;
 *  config->activeClockDivider = CS_ACTIVE_CLOCK_DIVIDER;
 *  config->activeDetectPeriod = CS_ACTIVE_DETECT_PERIOD;
 *  config->activeIdlePeriod = CS_ACTIVE_IDLE_PERIOD;
 *  config->activeOscFreq = CS_ACTIVE_OSC_FREQ;
 *  config->lowPowerChannelNum = CS_LOWPOWER_CHANNEL_NUM;
 *  config->lowPowerThreshold = CS_LOWPOWER_THRESHOLD;
 *  config->lowPowerDebonceNum = CS_LOWPOWER_DEBONCE_NUM;
 *  config->lowPowerIdlePeriod = CS_LOWPOWER_IDLE_PERIOD;
 *  config->lowPowerOscFreq = CS_LOWPOWER_OSC_FREQ;
 * @endcode
 * @param config Pointer to configuration structure.
 */
void CS_GetDefaultConfig(cs_config_t *config);

/*!
 * @brief Clear CS FIFO.
 *
 * @param base CS peripheral base address.
 */
static inline void CS_ClearFifo(CS_Type *base)
{
    base->CTRL0 |= CS_CTRL0_SRST_MASK;
    base->CTRL0 &= ~CS_CTRL0_SRST_MASK;
}

/*!
 * @brief Enable CS with specified mode.
 *
 * @param base CS peripheral base address.
 * @param config Pointer to configuration structure, see to cs_config_t.
 * @param mode Choose CS work mode, see to cs_mode_t.
 */
void CS_Enable(CS_Type *base, const cs_config_t *config, cs_mode_t mode);

/*!
 * @brief Disable CS.
 *
 * @param base CS peripheral base address.
 */
static inline void CS_Disable(CS_Type *base)
{
    /* Disable CS low power */
    base->LP_CTRL &= ~CS_LP_CTRL_LP_EN_MASK;
    /* Disable CS */
    base->CTRL0 &= ~CS_CTRL0_ENABLE_MASK;
}

/*! @} */

/*!
 * @name Data result.
 * @{
 */

/*!
 * @brief Read the data directly.
 *
 * @param base CS peripheral base address.
 *
 * @return The word read from CS data register.
 */
static inline uint32_t CS_ReadData(CS_Type *base)
{
    return base->DATA;
}

/*! @} */

/*!
 * @name Interrupts.
 * @{
 */

/*!
 * @brief Enables CS interrupts according to the provided mask.
 *
 * @param base CS peripheral base address.
 * @param mask The interrupts to enable. Logical OR of @ref _cs_interrupt_enable.
 */
static inline void CS_EnableInterrupts(CS_Type *base, uint32_t mask)
{
    base->INTEN |= mask;
}

/*!
 * @brief Disables CS interrupts according to a provided mask.
 *
 * @param base CS peripheral base address.
 * @param mask The interrupts to disable. Logical OR of @ref _cs_interrupt_enable.
 */
static inline void CS_DisableInterrupts(CS_Type *base, uint32_t mask)
{
    base->INTEN &= ~mask;
}

/*! @} */

/*!
 * @name Status.
 * @{
 */

/*!
 * @brief Get status flags of CS module.
 *
 * @param base CS peripheral base address.
 * @return CS status flags which are ORed by the enumerators in the @ref _cs_status_flags.
 */
static inline uint32_t CS_GetStatusFlags(CS_Type *base)
{
    return base->INT;
}

/*! @} */

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */

#endif /* _FSL_CS_H_ */
