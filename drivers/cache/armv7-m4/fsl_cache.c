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

#include "fsl_cache.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define L1CACHE_ONEWAYSIZE_BYTE (4096U) /*!< Cache size is 4K-bytes one way. */
/*******************************************************************************
 * Code
 ******************************************************************************/
#if (FSL_FEATURE_SOC_LMEM_COUNT == 1)
void L1CACHE_EnableICache(void)
{
    /* First, invalidate the entire cache. */
    L1CACHE_InvalidateICache();

    /* Now enable the cache. */
    LMEM->PCCCR |= LMEM_PCCCR_ENCACHE_MASK;
}

void L1CACHE_DisableICache(void)
{
    /* First, push any modified contents. */
    L1CACHE_CleanICache();

    /* Now disable the cache. */
    LMEM->PCCCR &= ~LMEM_PCCCR_ENCACHE_MASK;
}

void L1CACHE_InvalidateICache(void)
{
    /* Enables the processor code bus to invalidate all lines in both ways.
    and Initiate the processor code bus code cache command. */
    LMEM->PCCCR |= LMEM_PCCCR_INVW0_MASK | LMEM_PCCCR_INVW1_MASK | LMEM_PCCCR_GO_MASK;

    /* Wait until the cache command completes. */
    while (LMEM->PCCCR & LMEM_PCCCR_GO_MASK)
    {
    }

    /* As a precaution clear the bits to avoid inadvertently re-running this command. */
    LMEM->PCCCR &= ~(LMEM_PCCCR_INVW0_MASK | LMEM_PCCCR_INVW1_MASK);
}

void L1CACHE_InvalidateICacheByRange(uint32_t address, uint32_t size_byte)
{
    uint32_t endAddr = address + size_byte;
    uint32_t pccReg = 0;

    /* Align address to cache line size. */
    address = address & ~(L1ICACHE_LINESIZE_BYTE - 1U);
    /* If the size_byte exceeds 4KB, invalidate all. */
    if (size_byte >= L1CACHE_ONEWAYSIZE_BYTE)
    {
        L1CACHE_InvalidateICache();
    }
    else
    { /* Proceed with multi-line invalidate. */
        while (address < endAddr)
        {
            /* Set the invalidate by line command and use the physical address. */
            pccReg = (LMEM->PCCLCR & ~LMEM_PCCLCR_LCMD_MASK) | LMEM_PCCLCR_LCMD(1) | LMEM_PCCLCR_LADSEL_MASK;
            LMEM->PCCLCR = pccReg;

            /* Set the address and initiate the command. */
            LMEM->PCCSAR = (address & LMEM_PCCSAR_PHYADDR_MASK) | LMEM_PCCSAR_LGO_MASK;

            /* Wait until the cache command completes. */
            while (LMEM->PCCSAR & LMEM_PCCSAR_LGO_MASK)
            {
            }
            address = address + L1ICACHE_LINESIZE_BYTE;
        }
    }
}

void L1CACHE_CleanICache(void)
{
    /* Enable the processor code bus to push all modified lines. */
    LMEM->PCCCR |= LMEM_PCCCR_PUSHW0_MASK | LMEM_PCCCR_PUSHW1_MASK | LMEM_PCCCR_GO_MASK;

    /* Wait until the cache command completes. */
    while (LMEM->PCCCR & LMEM_PCCCR_GO_MASK)
    {
    }

    /* As a precaution clear the bits to avoid inadvertently re-running this command. */
    LMEM->PCCCR &= ~(LMEM_PCCCR_PUSHW0_MASK | LMEM_PCCCR_PUSHW1_MASK);
}

void L1CACHE_CleanICacheByRange(uint32_t address, uint32_t size_byte)
{
    uint32_t endAddr = address + size_byte;
    uint32_t pccReg = 0;
    /* Align address to cache line size. */
    address = address & ~(L1ICACHE_LINESIZE_BYTE - 1U);

    /* If the size_byte exceeds 4KB, push all. */
    if (size_byte >= L1CACHE_ONEWAYSIZE_BYTE)
    {
        L1CACHE_CleanICache();
    }
    else
    { /* Proceed with multi-line push. */
        while (address < endAddr)
        {
            /* Set the push by line command. */
            pccReg = (LMEM->PCCLCR & ~LMEM_PCCLCR_LCMD_MASK) | LMEM_PCCLCR_LCMD(2) | LMEM_PCCLCR_LADSEL_MASK;
            LMEM->PCCLCR = pccReg;

            /* Set the address and initiate the command. */
            LMEM->PCCSAR = (address & LMEM_PCCSAR_PHYADDR_MASK) | LMEM_PCCSAR_LGO_MASK;

            /* Wait until the cache command completes. */
            while (LMEM->PCCSAR & LMEM_PCCSAR_LGO_MASK)
            {
            }
            address = address + L1ICACHE_LINESIZE_BYTE;
        }
    }
}

void L1CACHE_CleanInvalidateICache(void)
{
    /* Push and invalidate all. */
    LMEM->PCCCR |= LMEM_PCCCR_PUSHW0_MASK | LMEM_PCCCR_PUSHW1_MASK | LMEM_PCCCR_INVW0_MASK | LMEM_PCCCR_INVW1_MASK |
                   LMEM_PCCCR_GO_MASK;

    /* Wait until the cache command completes. */
    while (LMEM->PCCCR & LMEM_PCCCR_GO_MASK)
    {
    }

    /* As a precaution clear the bits to avoid inadvertently re-running this command. */
    LMEM->PCCCR &= ~(LMEM_PCCCR_PUSHW0_MASK | LMEM_PCCCR_PUSHW1_MASK | LMEM_PCCCR_INVW0_MASK | LMEM_PCCCR_INVW1_MASK);
}

void L1CACHE_CleanInvalidateICacheByRange(uint32_t address, uint32_t size_byte)
{
    uint32_t endAddr = address + size_byte;
    uint32_t pccReg = 0;

    /* Align address to cache line size. */
    address = address & ~(L1ICACHE_LINESIZE_BYTE - 1U);

    /* If the size_byte exceeds 4KB, clear all. */
    if (size_byte >= L1CACHE_ONEWAYSIZE_BYTE)
    {
        L1CACHE_CleanInvalidateICache();
    }
    else /* Proceed with multi-line clear. */
    {
        while (address < endAddr)
        {
            /* Set the push by line command. */
            pccReg = (LMEM->PCCLCR & ~LMEM_PCCLCR_LCMD_MASK) | LMEM_PCCLCR_LCMD(3) | LMEM_PCCLCR_LADSEL_MASK;
            LMEM->PCCLCR = pccReg;

            /* Set the address and initiate the command. */
            LMEM->PCCSAR = (address & LMEM_PCCSAR_PHYADDR_MASK) | LMEM_PCCSAR_LGO_MASK;

            /* Wait until the cache command completes. */
            while (LMEM->PCCSAR & LMEM_PCCSAR_LGO_MASK)
            {
            }
            address = address + L1ICACHE_LINESIZE_BYTE;
        }
    }
}

#if defined(FSL_FEATURE_LMEM_HAS_SYSTEMBUS_CACHE) && FSL_FEATURE_LMEM_HAS_SYSTEMBUS_CACHE
void L1CACHE_EnableDCache(void)
{
    /* First, invalidate the entire cache. */
    L1CACHE_InvalidateDCache();

    /* Now enable the cache. */
    LMEM->PSCCR |= LMEM_PSCCR_ENCACHE_MASK;
}

void L1CACHE_DisableDCache(void)
{
    /* First, push any modified contents. */
    L1CACHE_CleanDCache();

    /* Now disable the cache. */
    LMEM->PSCCR &= ~LMEM_PSCCR_ENCACHE_MASK;
}

void L1CACHE_InvalidateDCache(void)
{
    /* Enables the processor system bus to invalidate all lines in both ways.
    and Initiate the processor system bus cache command. */
    LMEM->PSCCR |= LMEM_PSCCR_INVW0_MASK | LMEM_PSCCR_INVW1_MASK | LMEM_PSCCR_GO_MASK;

    /* Wait until the cache command completes */
    while (LMEM->PSCCR & LMEM_PSCCR_GO_MASK)
    {
    }

    /* As a precaution clear the bits to avoid inadvertently re-running this command. */
    LMEM->PSCCR &= ~(LMEM_PSCCR_INVW0_MASK | LMEM_PSCCR_INVW1_MASK);
}

void L1CACHE_InvalidateDCacheByRange(uint32_t address, uint32_t size_byte)
{
    uint32_t endAddr = address + size_byte;
    uint32_t pscReg = 0;
    address = address & ~(L1DCACHE_LINESIZE_BYTE - 1U); /* Align address to cache line size */

    /* If the size_byte exceeds 4KB, invalidate all. */
    if (size_byte >= L1CACHE_ONEWAYSIZE_BYTE)
    {
        L1CACHE_InvalidateDCache();
    }
    else /* Proceed with multi-line invalidate. */
    {
        while (address < endAddr)
        {
            /* Set the invalidate by line command and use the physical address. */
            pscReg = (LMEM->PSCLCR & ~LMEM_PSCLCR_LCMD_MASK) | LMEM_PSCLCR_LCMD(1) | LMEM_PSCLCR_LADSEL_MASK;
            LMEM->PSCLCR = pscReg;

            /* Set the address and initiate the command. */
            LMEM->PSCSAR = (address & LMEM_PSCSAR_PHYADDR_MASK) | LMEM_PSCSAR_LGO_MASK;

            /* Wait until the cache command completes. */
            while (LMEM->PSCSAR & LMEM_PSCSAR_LGO_MASK)
            {
            }
            address = address + L1DCACHE_LINESIZE_BYTE;
        }
    }
}

void L1CACHE_CleanDCache(void)
{
    /* Enable the processor system bus to push all modified lines. */
    LMEM->PSCCR |= LMEM_PSCCR_PUSHW0_MASK | LMEM_PSCCR_PUSHW1_MASK | LMEM_PSCCR_GO_MASK;

    /* Wait until the cache command completes. */
    while (LMEM->PSCCR & LMEM_PSCCR_GO_MASK)
    {
    }

    /* As a precaution clear the bits to avoid inadvertently re-running this command. */
    LMEM->PSCCR &= ~(LMEM_PSCCR_PUSHW0_MASK | LMEM_PSCCR_PUSHW1_MASK);
}

void L1CACHE_CleanDCacheByRange(uint32_t address, uint32_t size_byte)
{
    uint32_t endAddr = address + size_byte;
    uint32_t pscReg = 0;
    address = address & ~(L1DCACHE_LINESIZE_BYTE - 1U); /* Align address to cache line size. */

    /* If the size_byte exceeds 4KB, push all. */
    if (size_byte >= L1CACHE_ONEWAYSIZE_BYTE)
    {
        L1CACHE_CleanDCache();
    }
    else
    { /* Proceed with multi-line push. */
        while (address < endAddr)
        {
            /* Set the push by line command. */
            pscReg = (LMEM->PSCLCR & ~LMEM_PSCLCR_LCMD_MASK) | LMEM_PSCLCR_LCMD(2) | LMEM_PSCLCR_LADSEL_MASK;
            LMEM->PSCLCR = pscReg;

            /* Set the address and initiate the command. */
            LMEM->PSCSAR = (address & LMEM_PSCSAR_PHYADDR_MASK) | LMEM_PSCSAR_LGO_MASK;

            /* Wait until the cache command completes. */
            while (LMEM->PSCSAR & LMEM_PSCSAR_LGO_MASK)
            {
            }
            address = address + L1DCACHE_LINESIZE_BYTE;
        }
    }
}

void L1CACHE_CleanInvalidateDCache(void)
{
    /* Push and invalidate all. */
    LMEM->PSCCR |= LMEM_PSCCR_PUSHW0_MASK | LMEM_PSCCR_PUSHW1_MASK | LMEM_PSCCR_INVW0_MASK | LMEM_PSCCR_INVW1_MASK |
                   LMEM_PSCCR_GO_MASK;

    /* Wait until the cache command completes. */
    while (LMEM->PSCCR & LMEM_PSCCR_GO_MASK)
    {
    }

    /* As a precaution clear the bits to avoid inadvertently re-running this command. */
    LMEM->PSCCR &= ~(LMEM_PSCCR_PUSHW0_MASK | LMEM_PSCCR_PUSHW1_MASK | LMEM_PSCCR_INVW0_MASK | LMEM_PSCCR_INVW1_MASK);
}

void L1CACHE_CleanInvalidateDCacheByRange(uint32_t address, uint32_t size_byte)
{
    uint32_t endAddr = address + size_byte;
    uint32_t pscReg = 0;
    address = address & ~(L1DCACHE_LINESIZE_BYTE - 1U); /* Align address to cache line size. */

    /* If the size_byte exceeds 4KB, clear all. */
    if (size_byte >= L1CACHE_ONEWAYSIZE_BYTE)
    {
        L1CACHE_CleanInvalidateDCache();
    }
    else /* Proceed with multi-line clear. */
    {
        while (address < endAddr)
        {
            /* Set the push by line command. */
            pscReg = (LMEM->PSCLCR & ~LMEM_PSCLCR_LCMD_MASK) | LMEM_PSCLCR_LCMD(3) | LMEM_PSCLCR_LADSEL_MASK;
            LMEM->PSCLCR = pscReg;

            /* Set the address and initiate the command. */
            LMEM->PSCSAR = (address & LMEM_PSCSAR_PHYADDR_MASK) | LMEM_PSCSAR_LGO_MASK;

            /* Wait until the cache command completes. */
            while (LMEM->PSCSAR & LMEM_PSCSAR_LGO_MASK)
            {
            }
            address = address + L1DCACHE_LINESIZE_BYTE;
        }
    }
}

#endif /* FSL_FEATURE_LMEM_HAS_SYSTEMBUS_CACHE */
#endif /* FSL_FEATURE_SOC_LMEM_COUNT == 1 */
