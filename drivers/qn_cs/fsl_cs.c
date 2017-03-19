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

#include "fsl_cs.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define CS_ACTIVE_CHANNEL_ENABLE (0U)
#define CS_ACTIVE_CLOCK_DIVIDER (159U)
#define CS_ACTIVE_DETECT_PERIOD (1000U)
#define CS_ACTIVE_IDLE_PERIOD (50U)
#define CS_ACTIVE_OSC_FREQ (10U)

#define CS_LOWPOWER_CHANNEL_NUM (0U)
#define CS_LOWPOWER_THRESHOLD (600U)
#define CS_LOWPOWER_DEBONCE_NUM (3U)
#define CS_LOWPOWER_IDLE_PERIOD (50U)
#define CS_LOWPOWER_OSC_FREQ (10U)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Gets the instance from the base address
 *
 * @param base CS peripheral base address
 *
 * @return The CS instance
 */
static uint32_t CS_GetInstance(CS_Type *base);

/*!
 * @brief CS active mode configure
 *
 * @param base CS peripheral base address
 * @param config Pointer to configuration structure, see to cs_config_t.
 */
static void CS_ActiveModeConfig(CS_Type *base, const cs_config_t *config);

/*!
 * @brief CS low power mode configure
 *
 * @param base CS peripheral base address
 * @param config Pointer to configuration structure, see to cs_config_t.
 */
static void CS_LowPowerModeConfig(CS_Type *base, const cs_config_t *config);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Pointers to CS bases for each instance. */
static CS_Type *const s_csBases[] = CS_BASE_PTRS;

/*! @brief Pointers to CS clocks for each instance. */
static const clock_ip_name_t s_csClocks[] = CS_CLOCKS;

/*! @brief Pointers to CS resets for each instance. */
static const reset_ip_name_t s_csResets[] = CS_RSTS;

/*******************************************************************************
 * Code
 ******************************************************************************/
static uint32_t CS_GetInstance(CS_Type *base)
{
    uint32_t instance;

    /* Find the instance index from base address mappings. */
    for (instance = 0; instance < FSL_FEATURE_SOC_CS_COUNT; instance++)
    {
        if (s_csBases[instance] == base)
        {
            break;
        }
    }

    assert(instance < FSL_FEATURE_SOC_CS_COUNT);

    return instance;
}

static void CS_ActiveModeConfig(CS_Type *base, const cs_config_t *config)
{
    uint32_t mask, tmp32;

    /* Configure clock divider and OSC frequency */
    mask = CS_CTRL0_CLK_DIV_MASK | CS_CTRL0_OSC_FREQ_MASK;
    tmp32 = CS_CTRL0_CLK_DIV(config->activeClockDivider) | CS_CTRL0_OSC_FREQ(config->activeOscFreq);
    base->CTRL0 = (base->CTRL0 & ~mask) | tmp32;

    /* Configure period and channel enable */
    mask = CS_CTRL1_PERIOD_MASK | CS_CTRL1_CH_MASK;
    tmp32 = CS_CTRL1_PERIOD(config->activeDetectPeriod) | CS_CTRL1_CH(config->activeChannelEnable);
    base->CTRL1 = (base->CTRL0 & ~mask) | tmp32;

    /* Configure the idle period */
    base->IDLE_PERIOD =
        (base->IDLE_PERIOD & ~CS_IDLE_PERIOD_IDLE_PERIOD_MASK) | CS_IDLE_PERIOD_IDLE_PERIOD(config->activeIdlePeriod);

    /* Disable sleep interrupt */
    base->LP_INTEN &= ~CS_LP_INTEN_LP_INTEN_MASK;
    /* Disable CS low power */
    base->LP_CTRL &= ~CS_LP_CTRL_LP_EN_MASK;
}

static void CS_LowPowerModeConfig(CS_Type *base, const cs_config_t *config)
{
    uint32_t mask, tmp32;

    base->CTRL0 = (base->CTRL0 & ~CS_CTRL0_OSC_FREQ_MASK) | CS_CTRL0_OSC_FREQ(config->lowPowerOscFreq);

    mask = CS_LP_CTRL_DEBONCE_NUM_MASK | CS_LP_CTRL_LP_CH_MASK | CS_LP_CTRL_THR_MASK;
    tmp32 = CS_LP_CTRL_DEBONCE_NUM(config->lowPowerDebonceNum) | CS_LP_CTRL_LP_CH(config->lowPowerChannelNum) |
            CS_LP_CTRL_THR(config->lowPowerThreshold);
    base->LP_CTRL = (base->LP_CTRL & ~mask) | tmp32;

    base->IDLE_PERIOD =
        (base->IDLE_PERIOD & ~CS_IDLE_PERIOD_IDLE_PERIOD_MASK) | CS_IDLE_PERIOD_IDLE_PERIOD(config->lowPowerIdlePeriod);

    /* Enable sleep interrupt */
    base->LP_INTEN |= CS_LP_INTEN_LP_INTEN_MASK;
    /* Enable CS low power */
    base->LP_CTRL |= CS_LP_CTRL_LP_EN_MASK;
}

void CS_Init(CS_Type *base, const cs_config_t *config)
{
    (void)config;

    /* Enable CS clock */
    CLOCK_EnableClock(s_csClocks[CS_GetInstance(base)]);

    /* Reset CS */
    RESET_PeripheralReset(s_csResets[CS_GetInstance(base)]);
}

void CS_DeInit(CS_Type *base)
{
    /* Disable CS clock */
    CLOCK_DisableClock(s_csClocks[CS_GetInstance(base)]);
}

void CS_GetDefaultConfig(cs_config_t *config)
{
    /* For acitve mode */
    config->activeChannelEnable = CS_ACTIVE_CHANNEL_ENABLE;
    config->activeClockDivider = CS_ACTIVE_CLOCK_DIVIDER;
    config->activeDetectPeriod = CS_ACTIVE_DETECT_PERIOD;
    config->activeIdlePeriod = CS_ACTIVE_IDLE_PERIOD;
    config->activeOscFreq = CS_ACTIVE_OSC_FREQ;

    /* For low power mode */
    config->lowPowerChannelNum = CS_LOWPOWER_CHANNEL_NUM;
    config->lowPowerThreshold = CS_LOWPOWER_THRESHOLD;
    config->lowPowerDebonceNum = CS_LOWPOWER_DEBONCE_NUM;
    config->lowPowerIdlePeriod = CS_LOWPOWER_IDLE_PERIOD;
    config->lowPowerOscFreq = CS_LOWPOWER_OSC_FREQ;
}

void CS_Enable(CS_Type *base, const cs_config_t *config, cs_mode_t mode)
{
    if (mode == kCS_ActiveMode)
    {
        CS_ActiveModeConfig(base, config);
    }
    else
    {
        CS_LowPowerModeConfig(base, config);
    }

    /* Enable CS */
    base->CTRL0 |= CS_CTRL0_ENABLE_MASK;
}
