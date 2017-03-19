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
#ifndef _FSL_CACHE_H_
#define _FSL_CACHE_H_

#include "fsl_common.h"

/*!
 * @addtogroup cache
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief cache driver version 2.0.0. */
#define FSL_CACHE_DRIVER_VERSION (MAKE_VERSION(2, 0, 0))
/*@}*/

#define L1ICACHE_LINESIZE_BYTE (16) /*!< The l1 ICACHE line size is 16B = 128b. */
#define L1DCACHE_LINESIZE_BYTE (16) /*!< The l1 DCACHE line size is 16B = 128b. */

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

#if (FSL_FEATURE_SOC_LMEM_COUNT == 1)
/*!
 * @name cache control for L1 cache(local memory controller)
 *@{
 */

/*!
 * @brief Enables the processor code bus cache.
 *
 */
void L1CACHE_EnableICache(void);

/*!
 * @brief Disables the processor code bus cache.
 *
 */
void L1CACHE_DisableICache(void);

/*!
 * @brief Invalidates the processor code bus cache.
 *
 */
void L1CACHE_InvalidateICache(void);

/*!
 * @brief Invalidates instruction cache by range.
 *
 * @param address The physical address of cache.
 * @param size_byte size of the memory to be invalidated.
 * @note Address and size are recommended be aligned to "LMEM_CACHE_LINE_SIZE"
 *  16-Bytes or else it may generate unpredictable result.
 */
void L1CACHE_InvalidateICacheByRange(uint32_t address, uint32_t size_byte);

/*!
 * @brief Cleans the processor code bus cache.
 *
 */
void L1CACHE_CleanICache(void);

/*!
 * @brief Clean instruction cache by range.
 *
 * @param address The physical address of cache.
 * @param size_byte size of the memory to be clean.
 * @note Address and size are recommended be aligned to "LMEM_CACHE_LINE_SIZE"
 *  16-Bytes or else it may generate unpredictable result.
 */
void L1CACHE_CleanICacheByRange(uint32_t address, uint32_t size_byte);

/*!
 * @brief Cleans and invalidates the processor code bus cache.
 *
 */
void L1CACHE_CleanInvalidateICache(void);

/*!
 * @brief Clean and invalidate instruction cache by range.
 *
 * @param address The physical address of cache.
 * @param size_byte size of the memory to be CleanInvalidateed.
 * @note Address and size are recommended be aligned to "LMEM_CACHE_LINE_SIZE"
 *  16-Bytes or else it may generate unpredictable result.
 */
void L1CACHE_CleanInvalidateICacheByRange(uint32_t address, uint32_t size_byte);

/*!
 * @brief Enables/disables the processor code bus write buffer.
 *
 * @param enable The enable or disable flag.
 *       true  - enable the code bus write buffer.
 *       false - disable the code bus write buffer.
 */
static inline void L1CACHE_EnableICacheWriteBuffer(bool enable)
{
    if (enable)
    {
        LMEM->PCCCR |= LMEM_PCCCR_ENWRBUF_MASK;
    }
    else
    {
        LMEM->PCCCR &= ~LMEM_PCCCR_ENWRBUF_MASK;
    }
}

#if defined(FSL_FEATURE_LMEM_HAS_SYSTEMBUS_CACHE) && FSL_FEATURE_LMEM_HAS_SYSTEMBUS_CACHE
/*!
 * @brief Enables the processor system bus cache.
 *
 */
void L1CACHE_EnableDCache(void);

/*!
 * @brief Disables the processor system bus cache.
 *
 */
void L1CACHE_DisableDCache(void);

/*!
 * @brief Invalidates the processor system bus cache.
 *
 */
void L1CACHE_InvalidateDCache(void);

/*!
 * @brief Invalidates data cache by range.
 *
 * @param address The physical address of cache.
 * @param size_byte size of the memory to be invalidated.
 * @note Address and size are recommended be aligned to "LMEM_CACHE_LINE_SIZE"
 *  16-Bytes or else it may generate unpredictable result.
 */
void L1CACHE_InvalidateDCacheByRange(uint32_t address, uint32_t size_byte);

/*!
 * @brief Cleans the processor system bus cache.
 *
 */
void L1CACHE_CleanDCache(void);

/*!
 * @brief Clean data cache by range.
 *
 * @param address The physical address of cache.
 * @param size_byte size of the memory to be clean.
 * @note Address and size are recommended be aligned to "LMEM_CACHE_LINE_SIZE"
 *  16-Bytes or else it may generate unpredictable result.
 */
void L1CACHE_CleanDCacheByRange(uint32_t address, uint32_t size_byte);

/*!
 * @brief Cleans and invalidates the processor system bus cache.
 *
 */
void L1CACHE_CleanInvalidateDCache(void);

/*!
 * @brief Clean and Invalidate data cache by range.
 *
 * @param address The physical address of cache.
 * @param size_byte size of the memory to be CleanInvalidateed.
 * @note Address and size are recommended be aligned to "LMEM_CACHE_LINE_SIZE"
 *  16-Bytes or else it may generate unpredictable result.
 */
void L1CACHE_CleanInvalidateDCacheByRange(uint32_t address, uint32_t size_byte);

/*!
 * @brief Enables/disables the processor system bus write buffer.
 *
 * @param enable The enable or disable flag.
 *       true  - enable the code bus write buffer.
 *       false - disable the code bus write buffer.
 */
static inline void L1CACHE_EnableDCacheWriteBuffer(bool enable)
{
    if (enable)
    {
        LMEM->PSCCR |= LMEM_PSCCR_ENWRBUF_MASK;
    }
    else
    {
        LMEM->PSCCR &= ~LMEM_PSCCR_ENWRBUF_MASK;
    }
}
/*@}*/ 
#endif /* FSL_FEATURE_LMEM_HAS_SYSTEMBUS_CACHE */
#endif /* FSL_FEATURE_SOC_LMEM_COUNT == 1 */
 
/*!
 * @name Unified Cache Control for all caches
 *@{
 */

/*!
 * @brief Invalidates instruction cache by range.
 *
 * @param address The physical address of cache.
 * @param size_byte size of the memory to be invalidated.
 * @note Address and size are recommended be aligned to "LMEM_CACHE_LINE_SIZE"
 *  16-Bytes or else it may generate unpredictable result.
 */
static inline void ICACHE_InvalidateByRange(uint32_t address, uint32_t size_byte)
{
    L1CACHE_InvalidateICacheByRange(address, size_byte);
}

/*!
 * @brief Invalidates data cache by range.
 *
 * @param address The physical address of cache.
 * @param size_byte size of the memory to be invalidated.
 * @note Address and size are recommended be aligned to "LMEM_CACHE_LINE_SIZE"
 *  16-Bytes or else it may generate unpredictable result.
 */
static inline void DCACHE_InvalidateByRange(uint32_t address, uint32_t size_byte)
{
    L1CACHE_InvalidateDCacheByRange(address, size_byte);
}

/*!
 * @brief Clean data cache by range.
 *
 * @param address The physical address of cache.
 * @param size_byte size of the memory to be clean.
 * @note Address and size are recommended be aligned to "LMEM_CACHE_LINE_SIZE"
 *  16-Bytes or else it may generate unpredictable result.
 */
static inline void DCACHE_CleanByRange(uint32_t address, uint32_t size_byte)
{
    L1CACHE_CleanDCacheByRange(address, size_byte);
}

/*!
 * @brief Clean and Invalidate data cache by range.
 *
 * @param address The physical address of cache.
 * @param size_byte size of the memory to be CleanInvalidateed.
 * @note Address and size are recommended be aligned to "LMEM_CACHE_LINE_SIZE"
 *  16-Bytes or else it may generate unpredictable result.
 */
static inline void DCACHE_CleanInvalidateByRange(uint32_t address, uint32_t size_byte)
{
    L1CACHE_CleanInvalidateDCacheByRange(address, size_byte);
}

/*@}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* _FSL_CACHE_H_*/
