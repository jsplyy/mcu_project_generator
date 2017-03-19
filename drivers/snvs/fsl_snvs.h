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
#ifndef _FSL_SNVS_H_
#define _FSL_SNVS_H_

#include "fsl_common.h"

/*!
 * @addtogroup snvs
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
#define FSL_SNVS_DRIVER_VERSION (MAKE_VERSION(2, 0, 0)) /*!< Version 2.0.0 */
/*@}*/

/*! @brief List of SNVS interrupts */
typedef enum _snvs_interrupt_enable
{
    kSNVS_RTC_PeriodicInterruptEnable = 1U, /*!< RTC periodic interrupt.*/
    kSNVS_RTC_AlarmInterruptEnable = 2U,    /*!< RTC time alarm.*/
    kSNVS_SRTC_AlarmInterruptEnable = 4U,   /*!< SRTC time alarm.*/
} snvs_interrupt_enable_t;

/*! @brief List of SNVS flags */
typedef enum _snvs_status_flags
{
    kSNVS_RTC_PeriodicInterruptFlag = 1U, /*!< RTC periodic interrupt flag */
    kSNVS_RTC_AlarmInterruptFlag = 2U,    /*!< RTC time alarm flag */
    kSNVS_SRTC_AlarmInterruptFlag = 4U,   /*!< SRTC time alarm flag*/
} snvs_status_flags_t;

/*! @brief List of SNVS external tampers */
typedef enum _snvs_external_tamper
{
    kSNVS_ExternalTamper1 = 1U,
    kSNVS_ExternalTamper2 = 2U,
    kSNVS_ExternalTamper3 = 3U,
    kSNVS_ExternalTamper4 = 4U,
    kSNVS_ExternalTamper5 = 5U,
    kSNVS_ExternalTamper6 = 6U,
    kSNVS_ExternalTamper7 = 7U,
    kSNVS_ExternalTamper8 = 8U,
    kSNVS_ExternalTamper9 = 9U,
    kSNVS_ExternalTamper10 = 10U,
} snvs_external_tamper_t;

/*! @brief List of SNVS external tampers */
typedef enum _snvs_external_tamper_status
{
    kSNVS_TamperNotDetected = 0U,
    kSNVS_TamperDetected = 1U
} snvs_external_tamper_status_t;

/*! @brief SNVS external tamper polarity */
typedef enum _snvs_external_tamper_polarity
{
    kSNVS_ExternalTamperActiveLow = 0U,
    kSNVS_ExternalTamperActiveHigh = 1U,
} snvs_external_tamper_polarity_t;

/*! @brief Structure is used to hold the date and time */
typedef struct _snvs_datetime
{
    uint16_t year;  /*!< Range from 1970 to 2099.*/
    uint8_t month;  /*!< Range from 1 to 12.*/
    uint8_t day;    /*!< Range from 1 to 31 (depending on month).*/
    uint8_t hour;   /*!< Range from 0 to 23.*/
    uint8_t minute; /*!< Range from 0 to 59.*/
    uint8_t second; /*!< Range from 0 to 59.*/
} snvs_datetime_t;

/*!
 * @brief SNVS config structure
 *
 * This structure holds the configuration settings for the SNVS peripheral. To initialize this
 * structure to reasonable defaults, call the SNVS_GetDefaultConfig() function and pass a
 * pointer to your config structure instance.
 *
 * The config struct can be made const so it resides in flash
 */
typedef struct _snvs_config
{
    bool rtcCalEnable;              /*!< true: RTC calibration mechanism is enabled;
                                         false:No calibration is used */
    uint32_t rtcCalValue;           /*!< Defines signed calibration value for nonsecure RTC;
                                         This is a 5-bit 2's complement value, range from -16 to +15 */
    bool srtcCalEnable;             /*!< true: SRTC calibration mechanism is enabled;
                                         false:No calibration is used */
    uint32_t srtcCalValue;          /*!< Defines signed calibration value for SRTC;
                                         This is a 5-bit 2's complement value, range from -16 to +15 */
    uint32_t periodicInterruptFreq; /*!< Defines frequency of the periodic interrupt;
                                         Range from 0 to 15 */
} snvs_config_t;

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
 * @brief Ungates the SNVS clock and configures the peripheral for basic operation.
 *
 * @note This API should be called at the beginning of the application using the SNVS driver.
 *
 * @param base   SNVS peripheral base address
 * @param config Pointer to the user's SNVS configuration structure.
 */
void SNVS_Init(SNVS_Type *base, const snvs_config_t *config);

/*!
 * @brief Stops the RTC and SRTC timers.
 *
 * @param base SNVS peripheral base address
 */
static inline void SNVS_Deinit(SNVS_Type *base)
{
    base->HPCR &= ~SNVS_HPCR_RTC_EN_MASK;
    base->LPCR &= ~SNVS_LPCR_SRTC_ENV_MASK;
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    /* Gate the module clock */
    CLOCK_DisableClock(kCLOCK_SnvsHp0);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
}

/*!
 * @brief Fills in the SNVS config struct with the default settings.
 *
 * The default values are as follows.
 * @code
 *    config->rtccalenable = false;
 *    config->rtccalvalue = 0U;
 *    config->srtccalenable = false;
 *    config->srtccalvalue = 0U;
 *    config->PIFreq = 0U;
 * @endcode
 * @param config Pointer to the user's SNVS configuration structure.
 */
void SNVS_GetDefaultConfig(snvs_config_t *config);

/*! @}*/

/*!
 * @name Non secure RTC current Time & Alarm
 * @{
 */

/*!
 * @brief Sets the SNVS RTC date and time according to the given time structure.
 *
 * @param base     SNVS peripheral base address
 * @param datetime Pointer to the structure where the date and time details are stored.
 *
 * @return kStatus_Success: Success in setting the time and starting the SNVS RTC
 *         kStatus_InvalidArgument: Error because the datetime format is incorrect
 */
status_t SNVS_RTC_SetDatetime(SNVS_Type *base, const snvs_datetime_t *datetime);

/*!
 * @brief Gets the SNVS RTC time and stores it in the given time structure.
 *
 * @param base     SNVS peripheral base address
 * @param datetime Pointer to the structure where the date and time details are stored.
 */
void SNVS_RTC_GetDatetime(SNVS_Type *base, snvs_datetime_t *datetime);

/*!
 * @brief Sets the SNVS RTC alarm time.
 *
 * The function sets the RTC alarm. It also checks whether the specified alarm time
 * is greater than the present time. If not, the function does not set the alarm
 * and returns an error.
 *
 * @param base      SNVS peripheral base address
 * @param alarmTime Pointer to the structure where the alarm time is stored.
 *
 * @return kStatus_Success: success in setting the SNVS RTC alarm
 *         kStatus_InvalidArgument: Error because the alarm datetime format is incorrect
 *         kStatus_Fail: Error because the alarm time has already passed
 */
status_t SNVS_RTC_SetAlarm(SNVS_Type *base, const snvs_datetime_t *alarmTime);

/*!
 * @brief Returns the SNVS RTC alarm time.
 *
 * @param base     SNVS peripheral base address
 * @param datetime Pointer to the structure where the alarm date and time details are stored.
 */
void SNVS_RTC_GetAlarm(SNVS_Type *base, snvs_datetime_t *datetime);

/*!
 * @brief The function synchronizes RTC counter value with SRTC.
 *
 * @param base SNVS peripheral base address
 */
void SNVS_RTC_TimeSyncronize(SNVS_Type *base);

/*! @}*/

/*!
 * @name Secure RTC (SRTC) current Time & Alarm
 * @{
 */

/*!
 * @brief Sets the SNVS SRTC date and time according to the given time structure.
 *
 * @param base     SNVS peripheral base address
 * @param datetime Pointer to the structure where the date and time details are stored.
 *
 * @return kStatus_Success: Success in setting the time and starting the SNVS SRTC
 *         kStatus_InvalidArgument: Error because the datetime format is incorrect
 */
status_t SNVS_SRTC_SetDatetime(SNVS_Type *base, const snvs_datetime_t *datetime);

/*!
 * @brief Gets the SNVS SRTC time and stores it in the given time structure.
 *
 * @param base     SNVS peripheral base address
 * @param datetime Pointer to the structure where the date and time details are stored.
 */
void SNVS_SRTC_GetDatetime(SNVS_Type *base, snvs_datetime_t *datetime);

/*!
 * @brief Sets the SNVS SRTC alarm time.
 *
 * The function sets the SRTC alarm. It also checks whether the specified alarm
 * time is greater than the present time. If not, the function does not set the alarm
 * and returns an error.
 * Please note, that SRTC alarm has limited resolution because only 32 most
 * significant bits of SRTC counter are compared to SRTC Alarm register.
 * If the alarm time is beyond SRTC resolution, the function does not set the alarm
 * and returns an error.
 *
 * @param base      SNVS peripheral base address
 * @param alarmTime Pointer to the structure where the alarm time is stored.
 *
 * @return kStatus_Success: success in setting the SNVS SRTC alarm
 *         kStatus_InvalidArgument: Error because the alarm datetime format is incorrect
 *         kStatus_Fail: Error because the alarm time has already passed or is beyond resolution
 */
status_t SNVS_SRTC_SetAlarm(SNVS_Type *base, const snvs_datetime_t *alarmTime);

/*!
 * @brief Returns the SNVS SRTC alarm time.
 *
 * @param base     SNVS peripheral base address
 * @param datetime Pointer to the structure where the alarm date and time details are stored.
 */
void SNVS_SRTC_GetAlarm(SNVS_Type *base, snvs_datetime_t *datetime);

/*! @}*/

/*!
 * @name Interrupt Interface
 * @{
 */

/*!
 * @brief Enables the selected SNVS interrupts.
 *
 * @param base SNVS peripheral base address
 * @param mask The interrupts to enable. This is a logical OR of members of the
 *             enumeration ::snvs_interrupt_enable_t
 */
void SNVS_EnableInterrupts(SNVS_Type *base, uint32_t mask);

/*!
 * @brief Disables the selected SNVS interrupts.
 *
 * @param base SNVS peripheral base address
 * @param mask The interrupts to enable. This is a logical OR of members of the
 *             enumeration ::snvs_interrupt_enable_t
 */
void SNVS_DisableInterrupts(SNVS_Type *base, uint32_t mask);

/*!
 * @brief Gets the enabled SNVS interrupts.
 *
 * @param base SNVS peripheral base address
 *
 * @return The enabled interrupts. This is the logical OR of members of the
 *         enumeration ::snvs_interrupt_enable_t
 */
uint32_t SNVS_GetEnabledInterrupts(SNVS_Type *base);

/*! @}*/

/*!
 * @name Status Interface
 * @{
 */

/*!
 * @brief Gets the SNVS status flags.
 *
 * @param base SNVS peripheral base address
 *
 * @return The status flags. This is the logical OR of members of the
 *         enumeration ::snvs_status_flags_t
 */
uint32_t SNVS_GetStatusFlags(SNVS_Type *base);

/*!
 * @brief  Clears the SNVS status flags.
 *
 * @param base SNVS peripheral base address
 * @param mask The status flags to clear. This is a logical OR of members of the
 *             enumeration ::snvs_status_flags_t
 */
void SNVS_ClearStatusFlags(SNVS_Type *base, uint32_t mask);

/*! @}*/

/*!
 * @name Timer Start and Stop
 * @{
 */

/*!
 * @brief Starts the SNVS RTC time counter.
 *
 * @param base SNVS peripheral base address
 */
static inline void SNVS_RTC_StartTimer(SNVS_Type *base)
{
    base->HPCR |= SNVS_HPCR_RTC_EN_MASK;
}

/*!
 * @brief Stops the SNVS RTC time counter.
 *
 * @param base SNVS peripheral base address
 */
static inline void SNVS_RTC_StopTimer(SNVS_Type *base)
{
    base->HPCR &= ~SNVS_HPCR_RTC_EN_MASK;
}

/*!
 * @brief Starts the SNVS SRTC time counter.
 *
 * @param base SNVS peripheral base address
 */
static inline void SNVS_SRTC_StartTimer(SNVS_Type *base)
{
    base->LPCR |= SNVS_LPCR_SRTC_ENV_MASK;
}

/*!
 * @brief Stops the SNVS SRTC time counter.
 *
 * @param base SNVS peripheral base address
 */
static inline void SNVS_SRTC_StopTimer(SNVS_Type *base)
{
    base->LPCR &= ~SNVS_LPCR_SRTC_ENV_MASK;
}

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* _FSL_SNVS_H_ */
