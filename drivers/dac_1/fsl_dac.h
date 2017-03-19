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

#ifndef _FSL_DAC_H_
#define _FSL_DAC_H_

#include "fsl_common.h"

/*!
 * @addtogroup dac
 * @{
 */

/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief DAC driver version 2.0.0. */
#define FSL_DAC_DRIVER_VERSION (MAKE_VERSION(2, 0, 0))
/*@}*/

/*!
 * @brief DAC reset control.
 */
enum _dac_reset_control
{
    kDAC_ResetFIFO = DAC_RCR_FIFORST_MASK, /*!< Resets the FIFO pointers and flags. */
    kDAC_ResetLogic = DAC_RCR_SWRST_MASK,  /*!< Resets all DAC registers and internal logic. */
};

/*!
 * @brief DAC interrupts.
 */
enum _dac_interrupt_enable
{
    kDAC_FIFOFullInterruptEnable = DAC_IER_FULL_IE_MASK,    /*!< FIFO full interrupt enable. */
    kDAC_FIFOEmptyInterruptEnable = DAC_IER_EMPTY_IE_MASK,  /*!< FIFO empty interrupt enable. */
    kDAC_FIFOWatermarkInterruptEnable = DAC_IER_WM_IE_MASK, /*!< FIFO watermark interrupt enable. */
    kDAC_SwingBackInterruptEnable = DAC_IER_SWBK_IE_MASK,   /*!< Swing back one cycle complete interrupt enable. */
    kDAC_FIFOOverflowInterruptEnable = DAC_IER_OF_IE_MASK,  /*!< FIFO overflow interrupt enable. */
    kDAC_FIFOUnderflowInterruptEnable = DAC_IER_UF_IE_MASK, /*!< FIFO underflow interrupt enable. */
};

/*!
 * @brief DAC DMA switchers.
 */
enum _dac_dma_enable
{
    kDAC_FIFOEmptyDMAEnable = DAC_DER_EMPTY_DMAEN_MASK,  /*!< FIFO empty DMA enable. */
    kDAC_FIFOWatermarkDMAEnable = DAC_DER_WM_DMAEN_MASK, /*!< FIFO watermark DMA enable. */
};

/*!
 * @brief DAC status flags.
 */
enum _dac_status_flags
{
    kDAC_FIFOUnderflowFlag = DAC_FSR_UF_MASK, /*!< This flag means that there is a new trigger after the buffer is
empty. The FIFO read pointer will not
increase in this case and the data sent to DAC analog conversion will not changed. This flag is cleared by writing a 1
to it. */

    kDAC_FIFOOverflowFlag = DAC_FSR_OF_MASK, /*!< This flag indicates that data is intended to write into FIFO after the
buffer is full. The writer pointer will
not increase in this case. The extra data will not be written into the FIFO. This flag is cleared by writing a 1 to it.
*/

    kDAC_FIFOSwingBackFlag = DAC_FSR_SWBK_MASK, /*!< This flag indicates that the DAC has completed one period of
conversion in swing back mode. It means
that the read pointer has increased to the top (write pointer) once and then decreased to zero once. For
example, after three data is written to FIFO, the writer pointer is now 3. Then, if continually triggered, the
read pointer will swing like: 0-1-2-1-0-1-2-, and so on. After the fourth trigger, the flag is set. This flag is
cleared by writing a 1 to it. */

    kDAC_FIFOWatermarkFlag = DAC_FSR_WM_MASK, /*!< This field is set if the remaining data in FIFO is less than or equal
to the setting value of wartermark. By writing data into
FIFO by DMA or CPU, this flag is cleared automatically when the data in FIFO is more than the setting value of
watermark. */

    kDAC_FIFOEmptyFlag = DAC_FSR_EMPTY_MASK, /*!< FIFO empty flag. */
    kDAC_FIFOFullFlag = DAC_FSR_FULL_MASK,   /*!< FIFO full flag. */
};

/*!
 * @brief DAC FIFO trigger mode.
 */
typedef enum _dac_fifo_trigger_mode
{
    kDAC_FIFOTriggerByHardwareMode = 0U, /*!< Buffer would be triggered by hardware. */
    kDAC_FIFOTriggerBySoftwareMode = 1U, /*!< Buffer would be triggered by software. */
} dac_fifo_trigger_mode_t;

/*!
 * @brief DAC FIFO work mode.
 */
typedef enum _dac_fifo_work_mode
{
    kDAC_FIFODisabled = 0U, /*!< FIFO mode is disabled and buffer mode is enabled. Any data written to DATA[DATA] goes
                                 to buffer then goes to conversion. */
    kDAC_FIFOWorkAsNormalMode = 1U, /*!< FIFO mode is enabled. Data will be first read from FIFO to buffer then goes to
                                         conversion. */
    kDAC_FIFOWorkAsSwingMode = 2U, /*!< In swing mode, the read pointer swings between the writer pointer and zero. That
                                        is, the trigger increases the read pointer till reach the writer pointer and
                                        decreases the read pointer till zero, and so on. The FIFO empty/full/watermark
                                        flag will not update during swing back mode. */
} dac_fifo_work_mode_t;

/*!
 * @brief DAC reference voltage source.
 */
typedef enum _dac_reference_voltage_source
{
    kDAC_ReferenceVoltageSourceAlt1 = 0U, /*!< The DAC selects VREFH_INT as the reference voltage. */
    kDAC_ReferenceVoltageSourceAlt2 = 1U, /*!< The DAC selects VREFH_EXT as the reference voltage. */
} dac_reference_voltage_source_t;

/*!
 * @brief DAC configuration structure.
 */
typedef struct _dac_config
{
    uint32_t fifoWatermarkLevel;             /*!< FIFO's watermark, the max value can be the hardware FIFO size. */
    dac_fifo_trigger_mode_t fifoTriggerMode; /*!< Select the trigger mode for FIFO. */
    dac_fifo_work_mode_t fifoWorkMode;       /*!< Select the work mode for FIFO. */
    bool enableLowPowerMode;                 /*!< Enable the low power mode. */
    dac_reference_voltage_source_t referenceVoltageSource; /*!< Select the reference voltage source. */
} dac_config_t;

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
 * @brief Initialize the DAC module with common configuartion.
 *
 * The clock will be enabled in this function.
 *
 * @param base DAC peripheral base address.
 * @param config Pointer to configuration structure.
 */
void DAC_Init(DAC_Type *base, const dac_config_t *config);

/*!
 * @brief Get the default settings for initialization's configuration.
 *
 * This function initializes the user configuration structure to a default value. The default values are:
 * @code
 *   config->fifoWatermarkLevel = 0U;
 *   config->fifoTriggerMode = kDAC_FIFOTriggerByHardwareMode;
 *   config->fifoWorkMode = kDAC_FIFODisabled;
 *   config->enableLowPowerMode = false;
 *   config->referenceVoltageSource = kDAC_ReferenceVoltageSourceAlt1;
 * @endcode
 *
 * @param config Pointer to configuration structure.
 * @param
 */
void DAC_GetDefaultConfig(dac_config_t *config);

/*!
 * @brief De-initialize the DAC module.
 *
 * The clock will be disabled in this function.
 *
 * @param base DAC peripheral base address.
 * @param
 */
void DAC_Deinit(DAC_Type *base);

/*!
 * @brief Assert the reset control to part hardware.
 *
 * This fucntion is to assert the reset control to part hardware. Responding part hardware would remain reset untill
 * cleared by software.
 *
 * @param base DAC peripheral base address.
 * @param mask The reset control mask, see to #_dac_reset_control_t.
 */
static inline void DAC_SetReset(DAC_Type *base, uint32_t mask)
{
    base->RCR |= mask;
}

/*!
 * @brief Clear the reset control to part hardware.
 *
 * This fucntion is to clear the reset control to part hardware. Responding part hardware would work after the reset
 * control is cleared by software.
 *
 * @param base DAC peripheral base address.
 * @param mask The reset control mask, see to #_dac_reset_control_t.
 */
static inline void DAC_ClearReset(DAC_Type *base, uint32_t mask)
{
    base->RCR &= ~mask;
}

/*!
 * @brief Enable the DAC hardware system or not.
 *
 * This function is to start the Programmable Reference Generator operation or not.
 *
 * @param base DAC peripheral base address.
 * @param enable Assertion of indicated event.
 */
static inline void DAC_Enable(DAC_Type *base, bool enable)
{
    if (enable)
    {
        base->GCR |= DAC_GCR_DACEN_MASK;
    }
    else
    {
        base->GCR &= ~DAC_GCR_DACEN_MASK;
    }
}

/* @} */

/*!
 * @name Interrupts
 * @{
 */

/*!
 * @brief Enable the interrupts.
 *
 * @param base DAC peripheral base address.
 * @param mask Mask value of indicated interrupt events. See to #_dac_interrupt_enable.
 */
static inline void DAC_EnableInterrupts(DAC_Type *base, uint32_t mask)
{
    base->IER |= mask;
}

/*!
 * @brief Disable the interrupts.
 *
 * @param base DAC peripheral base address.
 * @param mask Mask value of indicated interrupt events. See to #_dac_interrupt_enable.
 */
static inline void DAC_DisableInterrupts(DAC_Type *base, uint32_t mask)
{
    base->IER &= ~mask;
}

/* @} */

/*!
 * @name DMA control
 * @{
 */

/*!
 * @brief Enable the DMA switchers or not.
 *
 * @param base DAC peripheral base address.
 * @param mask Mask value of indicated DMA requeset. See to #_dac_dma_enable.
 * @param enable Enable the DMA or not.
 */
static inline void DAC_EnableDMA(DAC_Type *base, uint32_t mask, bool enable)
{
    if (enable)
    {
        base->DER |= mask;
    }
    else
    {
        base->DER &= ~mask;
    }
}

/* @} */

/*!
 * @name Status flags
 * @{
 */

/*!
 * @brief Get status flags of DAC module.
 *
 * @param base DAC peripheral base address.
 * @return Mask value of status flags. See to #_dac_status_flags.
 */
static inline uint32_t DAC_GetStatusFlags(DAC_Type *base)
{
    return base->FSR;
}

/*!
 * @brief Clear status flags of DAC module.
 *
 * @param base DAC peripheral base address.
 * @param flags Mask value of status flags to be cleared. See to #_dac_status_flags.
 */
static inline void DAC_ClearStatusFlags(DAC_Type *base, uint32_t flags)
{
    base->FSR = flags;
}

/* @} */

/*!
 * @name Functional feature
 * @{
 */

/*!
 * @brief Set data into the entry of FIFO buffer.
 *
 * @param base DAC peripheral base address.
 * @param value Setting value into FIFO buffer.
 */
static inline void DAC_SetData(DAC_Type *base, uint32_t value)
{
    base->DATA = DAC_DATA_DATA(value);
}

/*!
 * @brief Get the value of the FIFO write pointer.
 *
 * @param base DAC peripheral base address.
 * @return Current value of the FIFO write pointer.
 */

static inline uint32_t DAC_GetFIFOWritePointer(DAC_Type *base)
{
    return (DAC_FPR_FIFO_WPT_MASK & base->FPR) >> DAC_FPR_FIFO_WPT_SHIFT;
}

/*!
 * @brief  Get the value of the FIFO read pointer.
 *
 * @param base DAC peripheral base address.
 * @return Current value of the FIFO read pointer.
 */

static inline uint32_t DAC_GetFIFOReadPointer(DAC_Type *base)
{
    return (DAC_FPR_FIFO_RPT_MASK & base->FPR) >> DAC_FPR_FIFO_RPT_SHIFT;
}

/*!
 * @brief Do software trigger to FIFO when in software mode.
 *
 * @param base DAC peripheral base address.
 */

static inline void DAC_DoSoftwareTriggerFIFO(DAC_Type *base)
{
    base->TCR = DAC_TCR_SWTRG_MASK;
}

/* @} */

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _FSL_DAC12_H_ */
