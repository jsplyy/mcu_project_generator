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

#include "fsl_flexspi_edma.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*<! Structure definition for flexspi_edma_private_handle_t. The structure is private. */
typedef struct _flexspi_edma_private_handle
{
    FLEXSPI_Type *base;
    flexspi_edma_handle_t *handle;
} flexspi_edma_private_handle_t;

/* FLEXSPI EDMA transfer handle. */
enum _flexspi_edma_tansfer_states
{
    kFLEXSPI_Idle, /* FLEXSPI Bus idle. */
    kFLEXSPI_Busy  /* FLEXSPI Bus busy. */
};

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*<! Private handle only used for internally. */
static flexspi_edma_private_handle_t s_edmaPrivateHandle[FSL_FEATURE_SOC_FLEXSPI_COUNT];

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * @brief FLEXSPI EDMA transfer finished callback function.
 *
 * This function is called when FLEXSPI EDMA transfer finished. It disables the FLEXSPI
 * TX/RX EDMA request and sends status to FLEXSPI callback.
 *
 * @param handle The EDMA handle.
 * @param param Callback function parameter.
 */
static void FLEXSPI_TransferEDMACallback(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds);

/*!
 * @brief Get the FLEXSPI instance from peripheral base address.
 *
 * @param base FLEXSPI peripheral base address.
 * @return FLEXSPI instance.
 */
extern uint32_t FLEXSPI_GetInstance(FLEXSPI_Type *base);

/*******************************************************************************
 * Code
 ******************************************************************************/

static void FLEXSPI_TransferEDMACallback(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
    flexspi_edma_private_handle_t *flexspiPrivateHandle = (flexspi_edma_private_handle_t *)param;

    /* Avoid warning for unused parameters. */
    handle = handle;
    tcds = tcds;

    if (transferDone)
    {
        /* Disable transfer. */
        FLEXSPI_TransferAbortEDMA(flexspiPrivateHandle->base, flexspiPrivateHandle->handle);

        if (flexspiPrivateHandle->handle->completionCallback)
        {
            flexspiPrivateHandle->handle->completionCallback(flexspiPrivateHandle->base, flexspiPrivateHandle->handle,
                                                             kStatus_FLEXSPI_Idle,
                                                             flexspiPrivateHandle->handle->userData);
        }
    }
}

void FLEXSPI_TransferCreateHandleEDMA(FLEXSPI_Type *base,
                                      flexspi_edma_handle_t *handle,
                                      flexspi_edma_callback_t callback,
                                      void *userData,
                                      edma_handle_t *txDmaHandle,
                                      edma_handle_t *rxDmaHandle)
{
    assert(handle);

    uint32_t instance = FLEXSPI_GetInstance(base);

    s_edmaPrivateHandle[instance].base = base;
    s_edmaPrivateHandle[instance].handle = handle;

    memset(handle, 0, sizeof(*handle));

    handle->state = kFLEXSPI_Idle;
    handle->txDmaHandle = txDmaHandle;
    handle->rxDmaHandle = rxDmaHandle;

    handle->completionCallback = callback;
    handle->userData = userData;
}

status_t FLEXSPI_TransferEDMA(FLEXSPI_Type *base, flexspi_edma_handle_t *handle, flexspi_transfer_t *xfer)
{
    uint32_t configValue = 0;
    status_t result = kStatus_Success;
    edma_transfer_config_t xferConfig;
    uint32_t instance = FLEXSPI_GetInstance(base);

    assert(handle);
    assert(xfer);

    /* Check if the FLEXSPI bus is idle - if not return busy status. */
    if (handle->state != kFLEXSPI_Idle)
    {
        result = kStatus_FLEXSPI_Busy;
    }
    else
    {
        handle->transferSize = xfer->dataSize;
        handle->state = kFLEXSPI_Busy;

        /* Clear sequence pointer before sending data to external devices. */
        base->FLSHCR2[xfer->port] |= FLEXSPI_FLSHCR2_CLRINSTRPTR_MASK;

        /* Clear former pending status before start this tranfer. */
        base->INTR |= FLEXSPI_INTR_AHBCMDERR_MASK | FLEXSPI_INTR_IPCMDERR_MASK | FLEXSPI_INTR_AHBCMDGE_MASK |
                      FLEXSPI_INTR_IPCMDGE_MASK;

        /* Configure base addresss. */
        base->IPCR0 = xfer->deviceAddress;

        /* Reset fifos. */
        base->IPTXFCR |= FLEXSPI_IPTXFCR_CLRIPTXF_MASK;
        base->IPRXFCR |= FLEXSPI_IPRXFCR_CLRIPRXF_MASK;

        /* Configure data size. */
        if ((xfer->cmdType == kFLEXSPI_Read) || (xfer->cmdType == kFLEXSPI_Write))
        {
            configValue = FLEXSPI_IPCR1_IDATSZ(xfer->dataSize);
        }

        /* Configure sequence ID. */
        configValue |= FLEXSPI_IPCR1_ISEQID(xfer->seqIndex) | FLEXSPI_IPCR1_ISEQNUM(xfer->SeqNumber - 1);
        base->IPCR1 = configValue;

        /* Start Transfer. */
        base->IPCMD |= FLEXSPI_IPCMD_TRG_MASK;
    }

    if (xfer->cmdType == kFLEXSPI_Write)
    {
        handle->count = (base->IPTXFCR & FLEXSPI_IPTXFCR_TXWMRK_MASK) >> FLEXSPI_IPTXFCR_TXWMRK_SHIFT + 1;

        /* Store the initially configured eDMA minor byte transfer count into the FLEXSPI handle */
        handle->nbytes = (8 * handle->count);

        /* Prepare transfer. */
        EDMA_PrepareTransfer(&xferConfig, xfer->data, sizeof(uint32_t), (void *)FLEXSPI_GetTxFifoAddress(base),
                             sizeof(uint32_t), handle->nbytes, xfer->dataSize, kEDMA_MemoryToPeripheral);

        /* Submit transfer. */
        EDMA_SubmitTransfer(handle->txDmaHandle, &xferConfig);
        EDMA_SetCallback(handle->txDmaHandle, FLEXSPI_TransferEDMACallback,
                         &s_edmaPrivateHandle[FLEXSPI_GetInstance(base)]);
        EDMA_StartTransfer(handle->txDmaHandle);

        /* Enable FLEXSPI TX EDMA. */
        FLEXSPI_EnableTxDMA(base, true);
    }
    else if (xfer->cmdType == kFLEXSPI_Read)
    {
        handle->count = (base->IPRXFCR & FLEXSPI_IPRXFCR_RXWMRK_MASK) >> FLEXSPI_IPRXFCR_RXWMRK_SHIFT + 1;

        /* Store the initially configured eDMA minor byte transfer count into the FLEXSPI handle */
        handle->nbytes = (8 * handle->count);

        /* Prepare transfer. */
        EDMA_PrepareTransfer(&xferConfig, (void *)FLEXSPI_GetRxFifoAddress(base), sizeof(uint32_t), xfer->data,
                             sizeof(uint32_t), handle->nbytes, xfer->dataSize, kEDMA_PeripheralToMemory);

        /* Submit transfer. */
        EDMA_SubmitTransfer(handle->rxDmaHandle, &xferConfig);
        EDMA_SetCallback(handle->rxDmaHandle, FLEXSPI_TransferEDMACallback, &s_edmaPrivateHandle[instance]);
        EDMA_StartTransfer(handle->rxDmaHandle);

        /* Enable FLEXSPI RX EDMA. */
        FLEXSPI_EnableRxDMA(base, true);
    }
    else
    {
    }

    return result;
}

void FLEXSPI_TransferAbortEDMA(FLEXSPI_Type *base, flexspi_edma_handle_t *handle)
{
    assert(handle);

    if (base->IPTXFCR & FLEXSPI_IPTXFCR_TXDMAEN_MASK)
    {
        FLEXSPI_EnableTxDMA(base, false);
        EDMA_AbortTransfer(handle->txDmaHandle);
    }

    if (base->IPRXFCR & FLEXSPI_IPRXFCR_RXDMAEN_MASK)
    {
        FLEXSPI_EnableRxDMA(base, false);
        EDMA_AbortTransfer(handle->rxDmaHandle);
    }

    handle->state = kFLEXSPI_Idle;
}

status_t FLEXSPI_TransferGetTransferCountEDMA(FLEXSPI_Type *base, flexspi_edma_handle_t *handle, size_t *count)
{
    assert(handle);
    assert(count);

    status_t result = kStatus_Success;

    if (handle->state != kFLEXSPI_Busy)
    {
        result = kStatus_NoTransferInProgress;
    }
    else
    {
        if (base->IPRXFCR & FLEXSPI_IPRXFCR_RXDMAEN_MASK)
        {
            *count = (handle->transferSize -
                      (uint32_t)handle->nbytes *
                          EDMA_GetRemainingMajorLoopCount(handle->rxDmaHandle->base, handle->rxDmaHandle->channel));
        }
        else if (base->IPTXFCR & FLEXSPI_IPTXFCR_TXDMAEN_MASK)
        {
            *count = (handle->transferSize -
                      (uint32_t)handle->nbytes *
                          EDMA_GetRemainingMajorLoopCount(handle->txDmaHandle->base, handle->txDmaHandle->channel));
        }
        else
        {
        }
    }

    return result;
}
