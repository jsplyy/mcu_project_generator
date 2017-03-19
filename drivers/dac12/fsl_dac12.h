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

#ifndef _FSL_DAC12_H_
#define _FSL_DAC12_H_

#include "fsl_common.h"

/*!
 * @addtogroup dac12
 * @{
 */

/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief DAC12 driver version 2.0.0. */
#define FSL_DAC12_DRIVER_VERSION (MAKE_VERSION(2, 0, 0))
/*@}*/

/*! @brief Define "write 1 to clear" flags. */
#define DAC12_CR_W1C_FLAGS_MASK (DAC_CR_OVFF_MASK | DAC_CR_UDFF_MASK)
/*! @brief Define all the flag bits in DACx_CR register. */
#define DAC12_CR_ALL_FLAGS_MASK (DAC12_CR_W1C_FLAGS_MASK | DAC_CR_WMF_MASK | DAC_CR_NEMPTF_MASK | DAC_CR_FULLF_MASK)

/*!
 * @brief DAC12 flags.
 */
enum _dac12_status_flags
{
    kDAC12_OverflowFlag = DAC_CR_OVFF_MASK,  /*!< FIFO overflow status flag, which indicates that more data has been
                                                  written into FIFO than it can hold. */
    kDAC12_UnderflowFlag = DAC_CR_UDFF_MASK, /*!< FIFO underflow status flag, which means that there is a new trigger
                                                  after the FIFO is nearly empty. */
    kDAC12_WatermarkFlag = DAC_CR_WMF_MASK, /*!< FIFO wartermark status flag, which indicates the remaining FIFO data is
                                                 less than the watermark setting. */
    kDAC12_NearlyEmptyFlag = DAC_CR_NEMPTF_MASK, /*!< FIFO nearly empty flag, which means there is only one data
                                                      remaining in FIFO. */
    kDAC12_FullFlag = DAC_CR_FULLF_MASK /*!< FIFO full status flag, which means that the FIFO read pointer equals the
                                             write pointer, as the write pointer increase. */
};

/*!
 * @brief DAC12 interrupts.
 */
enum _dac12_interrupt_enable
{
    kDAC12_UnderOrOverflowInterruptEnable = DAC_CR_UVIE_MASK, /*!< Underflow and overflow interrupt enable. */
    kDAC12_WatermarkInterruptEnable = DAC_CR_WTMIE_MASK,      /*!< Watermark interrupt enable. */
    kDAC12_NearlyEmptyInterruptEnable = DAC_CR_EMPTIE_MASK,   /*!< Nearly empty interrupt enable. */
    kDAC12_FullInterruptEnable = DAC_CR_FULLIE_MASK           /*!< Full interrupt enable. */
};

/*!
 * @brief DAC12 FIFO size information provided by hardware.
 */
typedef enum _dac12_fifo_size_info
{
    kDAC12_FIFOSize2 = 0U,   /*!< FIFO depth is 2. */
    kDAC12_FIFOSize4 = 1U,   /*!< FIFO depth is 4. */
    kDAC12_FIFOSize8 = 2U,   /*!< FIFO depth is 8. */
    kDAC12_FIFOSize16 = 3U,  /*!< FIFO depth is 16. */
    kDAC12_FIFOSize32 = 4U,  /*!< FIFO depth is 32. */
    kDAC12_FIFOSize64 = 5U,  /*!< FIFO depth is 64. */
    kDAC12_FIFOSize128 = 6U, /*!< FIFO depth is 128. */
    kDAC12_FIFOSize256 = 7U, /*!< FIFO depth is 256. */
} dac12_fifo_size_info_t;

/*!
 * @brief DAC12 FIFO work mode.
 */
typedef enum _dac12_fifo_work_mode
{
    kDAC12_FIFODisabled = 0U, /*!< FIFO disabled and only one level buffer is enabled. Any data written from this buffer
                                   goes to conversion. */
    kDAC12_FIFOWorkAsNormalMode = 1U, /*!< Data will first read from FIFO to buffer then go to conversion. */
    kDAC12_FIFOWorkAsSwingMode = 2U   /*!< In Swing mode, the FIFO must be set up to be full. In Swing back mode, a
                                           trigger changes the read pointer to make it swing between the FIFO Full and
                                           Nearly Empty state. That is, the trigger increases the read pointer till FIFO
                                           is nearly empty and decreases the read pointer till the FIFO is full. */
} dac12_fifo_work_mode_t;

/*!
 * @brief DAC12 reference voltage source.
 */
typedef enum _dac12_reference_voltage_source
{
    kDAC12_ReferenceVoltageSourceAlt1 = 0U, /*!< The DAC selects DACREF_1 as the reference voltage. */
    kDAC12_ReferenceVoltageSourceAlt2 = 1U, /*!< The DAC selects DACREF_2 as the reference voltage. */
} dac12_reference_voltage_source_t;

/*!
 * @brief DAC12 FIFO trigger mode.
 */
typedef enum _dac12_fifo_trigger_mode
{
    kDAC12_FIFOTriggerByHardwareMode = 0U, /*!< Buffer would be triggered by hardware. */
    kDAC12_FIFOTriggerBySoftwareMode = 1U, /*!< Buffer would be triggered by software. */
} dac12_fifo_trigger_mode_t;

/*!
 * @brief DAC internal reference current source.
 *
 * Analog module needs reference current to keep working . Such reference current can generated by IP itself, or by
 * on-chip PMC's "reference part". If no current reference be selected, analog module can’t working normally ,even when
 * other register can still be assigned, DAC would waste current but no function.
 * To make the DAC work, either kDAC12_ReferenceCurrentSourceAltx should be selected.
 */
typedef enum _dac12_reference_current_source
{
    kDAC12_ReferenceCurrentSourceDisabled = 0U, /*!< None of reference current source is enabled. */
    kDAC12_ReferenceCurrentSourceAlt0 = 1U, /*!< Use the internal reference current generated by the module itself. */
    kDAC12_ReferenceCurrentSourceAlt1 = 2U, /*!< Use the ZTC(Zero Temperature Coefficient) reference current generated
                                                 by on-chip power management module. */
    kDAC12_ReferenceCurrentSourceAlt2 = 3U, /*!< Use the PTAT(Proportional To Absolution Temperature) reference current
                                                 generated by power management module. */
} dac12_reference_current_source_t;

/*!
 * @brief DAC analog buffer speed mode for conversion.
 */
typedef enum _dac12_speed_mode
{
    kDAC12_SpeedLowMode = 0U,    /*!< Low speed mode. */
    kDAC12_SpeedMiddleMode = 1U, /*!< Middle speed mode. */
    kDAC12_SpeedHighMode = 2U,   /*!< High speed mode. */
} dac12_speed_mode_t;

/*!
 * @brief DAC12 hardware information.
 */
typedef struct _dac12_hardware_info
{
    dac12_fifo_size_info_t fifoSizeInfo; /*!< The number of words in this device's DAC buffer. */
} dac12_hardware_info_t;

/*!
 * @brief DAC12 module configuration.
 *
 * Actually, the most fields are for FIFO buffer.
 */
typedef struct
{
    uint32_t fifoWatermarkLevel;         /*!< FIFO's watermark, the max value can be the hardware FIFO size. */
    dac12_fifo_work_mode_t fifoWorkMode; /*!< FIFI's work mode about pointers. */
    dac12_reference_voltage_source_t referenceVoltageSource; /*!< Select the reference voltage source. */
    dac12_fifo_trigger_mode_t fifoTriggerMode;               /*! Select the trigger mode for FIFO. */

    /* Analog part configuration. */
    dac12_reference_current_source_t referenceCurrentSource; /*!< Select the reference current source. */
    dac12_speed_mode_t speedMode;                            /*!< Select the speed mode for conversion. */
    bool enableAnalogBuffer;                                 /*!< Enable analog buffer for high drive. */
    uint32_t currentReferenceInternalTrimValue; /*!< Internal reference current trim value. 3-bit value is available.*/
} dac12_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Initialization and de-initialization
 * @{
 */

/*!
 * @brief Get hardware information about this module.
 *
 * @param base DAC12 peripheral base address.
 * @param info Pointer to info structure, see to #dac12_hardware_info_t.
 */
void DAC12_GetHardwareInfo(DAC_Type *base, dac12_hardware_info_t *info);

/*!
 * @brief Initialize the DAC12 module.
 *
 * @param base DAC12 peripheral base address.
 * @param config Pointer to configuration structure, see to #dac12_config_t.
 */
void DAC12_Init(DAC_Type *base, const dac12_config_t *config);

/*!
 * @brief Initializes the DAC12 user configuration structure.
 *
 * This function initializes the user configuration structure to a default value. The default values are:
 * @code
 *   config->fifoWatermarkLevel = 0U;
 *   config->fifoWorkMode = kDAC12_FIFODisabled;
 *   config->referenceVoltageSource = kDAC12_ReferenceVoltageSourceAlt1;
 *   config->fifoTriggerMode = kDAC12_FIFOTriggerByHardwareMode;
 *   config->referenceCurrentSource = kDAC12_ReferenceCurrentSourceAlt0;
 *   config->speedMode = kDAC12_SpeedLowMode;
 *   config->speedMode = false;
 *   config->currentReferenceInternalTrimValue = 0x4;
 * @endcode
 * @param config Pointer to the configuration structure. See "dac12_config_t".
 */
void DAC12_GetDefaultConfig(dac12_config_t *config);

/*!
 * @brief De-initialize the DAC12 module.
 *
 * @param base DAC12 peripheral base address.
 */
void DAC12_Deinit(DAC_Type *base);

/*!
 * @brief Enable the DAC12's converter or not.
 *
 * @param base DAC12 peripheral base address.
 * @param enable Enable the DAC12's converter or not.
 */
static inline void DAC12_Enable(DAC_Type *base, bool enable)
{
    if (enable)
    {
        base->CR = (base->CR & ~DAC12_CR_W1C_FLAGS_MASK) | DAC_CR_DACEN_MASK;
    }
    else
    {
        base->CR &= ~DAC_CR_DACEN_MASK;
    }
}

/*!
 * @brief Reset all internal logic and registers.
 *
 * @param base DAC12 peripheral base address.
 */
static inline void DAC12_ResetConfig(DAC_Type *base)
{
    base->CR = DAC_CR_SWRST_MASK;
}

/*!
 * @brief Reset the FIFO pointers.
 *
 * FIFO pointers should only be reset when the DAC12 is disabled. This function can be used to configure both pointers
 * to the same address to reset the FIFO as empty.
 *
 * @param base DAC12 peripheral base address.
 */
static inline void DAC12_ResetFIFO(DAC_Type *base)
{
    /* FIFO pointers should only be reset when the module is disabled. */
    base->CR = (base->CR & ~DAC12_CR_W1C_FLAGS_MASK) | DAC_CR_FIFORST_MASK;
}

/* @} */

/*!
 * @name Status
 * @{
 */

/*!
 * @brief Get status flags.
 *
 * @param base DAC12 peripheral base address.
 * @return Mask of current status flags. See to #_dac12_status_flags.
 */
static inline uint32_t DAC12_GetStatusFlags(DAC_Type *base)
{
    return (DAC12_CR_ALL_FLAGS_MASK & base->CR);
}

/*!
 * @brief Clear status flags.
 *
 * Note: Not all the flags can be cleared by this API. Several flags need special condition to clear them according to
 * target chip's reference manual document.
 *
 * @param base DAC12 peripheral base address.
 * @param flags Mask of status flags to be cleared. See to #_dac12_status_flags.
 */
static inline void DAC12_ClearStatusFlags(DAC_Type *base, uint32_t flags)
{
    base->CR |= (flags & DAC12_CR_W1C_FLAGS_MASK);
}

/* @} */

/*!
 * @name Interrupts
 * @{
 */

/*!
 * @brief Enable interrupts.
 *
 * @param base DAC12 peripheral base address.
 * @param mask Mask value of interrupts to be enabled. See to #_dac12_interrupt_enable.
 */
static inline void DAC12_EnableInterrutps(DAC_Type *base, uint32_t mask)
{
    base->CR = (base->CR & ~DAC12_CR_W1C_FLAGS_MASK) | mask;
}

/*!
 * @brief Disable interrupts.
 *
 * @param base DAC12 peripheral base address.
 * @param mask Mask value of interrupts to be disabled. See to #_dac12_interrupt_enable.
 */
static inline void DAC12_DisableInterrupts(DAC_Type *base, uint32_t mask)
{
    base->CR &= ~mask;
}

/* @} */

/*!
 * @name DMA control
 * @{
 */

/*!
 * @brief Enable DMA or not.
 *
 * When DMA is enabled, the DMA request will be generated by original interrupts. The interrupts will not be presented
 * on this module at the same time.
 */
static inline void DAC12_EnableDMA(DAC_Type *base, bool enable)
{
    if (enable)
    {
        base->CR = (base->CR & ~DAC12_CR_W1C_FLAGS_MASK) | DAC_CR_DMAEN_MASK;
    }
    else
    {
        base->CR &= ~DAC_CR_DMAEN_MASK;
    }
}

/* @} */

/*!
 * @name Functional feature
 * @{
 */

/*!
 * @brief Set data into the entry of FIFO buffer.
 *
 * When the DAC FIFO is disabled, and the one entry buffer is enabled, the DAC converts the data in the buffer to analog
 * output voltage. Any write to the DATA register will replace the data in the buffer and push data to analog conversion
 * without trigger support.
 * When the DAC FIFO is enabled. Writing data would increase the write pointer of FIFO. Also, the data would be restored
 * into the FIFO buffer.
 *
 * @param base DAC12 peripheral base address.
 * @param value Setting value into FIFO buffer.
 */
static inline void DAC12_SetData(DAC_Type *base, uint32_t value)
{
    /* The module is connected internally to a 32-bit interface.
     * For the 8-bit or 16-bit, the write might be ignored. */
    base->DATA = DAC_DATA_DATA0(value);
}

/*!
 * @brief Do trigger the FIFO by software.
 *
 * When the DAC FIFO is enabled, and software trigger is used. Doing trigger would increase the read pointer, and the
 * data in the entry pointed by read pointer would be converted as new output.
 *
 * @param base DAC12 peripheral base address.
 */
static inline void DAC12_DoSoftwareTrigger(DAC_Type *base)
{
    base->CR = (base->CR & ~DAC12_CR_W1C_FLAGS_MASK) | DAC_CR_SWTRG_MASK;
}

/*!
 * @brief Get the current read pointer of FIFO.
 *
 * @param base DAC12 peripheral base address.
 * @return Read pointer index of FIFO buffer.
 */
static inline uint32_t DAC12_GetFIFOReadPointer(DAC_Type *base)
{
    return (DAC_PTR_DACRFP_MASK & base->CR) >> DAC_PTR_DACRFP_SHIFT;
}

/*!
 * @brief Get the current write pointer of FIFO.
 *
 * @param base DAC12 peripheral base address.
 * @return Write pointer index of FIFO buffer
 */
static inline uint32_t DAC12_GetFIFOWritePointer(DAC_Type *base)
{
    return (DAC_PTR_DACWFP_MASK & base->CR) >> DAC_PTR_DACWFP_SHIFT;
}

/* @} */

#if defined(__cplusplus)
}
#endif
/*!
 * @}
 */
#endif /* _FSL_DAC12_H_ */
