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

#ifndef _FSL_FLEXSPI_EDMA_H_
#define _FSL_FLEXSPI_EDMA_H_

#include "fsl_flexspi.h"
#include "fsl_edma.h"

/*!
* @addtogroup flexspi_edma
* @{
*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

typedef struct _flexspi_edma_handle flexspi_edma_handle_t;

/*! @brief FLEXSPI eDMA transfer callback function for finish and error */
typedef void (*flexspi_edma_callback_t)(FLEXSPI_Type *base,
                                        flexspi_edma_handle_t *handle,
                                        status_t status,
                                        void *userData);

/*! @brief FLEXSPI DMA transfer handle, users should not touch the content of the handle.*/
struct _flexspi_edma_handle
{
    edma_handle_t *txDmaHandle;                 /*!< eDMA handler for FLEXSPI Tx. */
    edma_handle_t *rxDmaHandle;                 /*!< eDMA handler for FLEXSPI Rx. */
    size_t transferSize;                        /*!< Bytes need to transfer. */
    uint8_t nbytes;                             /*!< eDMA minor byte transfer count initially configured. */
    uint8_t count;                              /*!< The transfer data count in a DMA request. */
    uint32_t state;                             /*!< Internal state for FLEXSPI eDMA transfer. */
    flexspi_edma_callback_t completionCallback; /*!< A callback function called after the eDMA transfer is finished. */
    void *userData;                             /*!< User callback parameter */
};

/*******************************************************************************
 * APIs
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name FLEXSPI eDMA Transactional
 * @{
 */

/*!
 * @brief Initializes the FLEXSPI handle for transfer which is used in transactional functions and set the callback.
 *
 * @param base FLEXSPI peripheral base address
 * @param handle Pointer to flexspi_edma_handle_t structure
 * @param callback FLEXSPI callback, NULL means no callback.
 * @param userData User callback function data.
 * @param txDmaHandle User requested DMA handle for TX DMA transfer.
 * @param rxDmaHandle User requested DMA handle for RX DMA transfer.
 */
void FLEXSPI_TransferCreateHandleEDMA(FLEXSPI_Type *base,
                                      flexspi_edma_handle_t *handle,
                                      flexspi_edma_callback_t callback,
                                      void *userData,
                                      edma_handle_t *txDmaHandle,
                                      edma_handle_t *rxDmaHandle);

/*!
 * @brief Transfers FLEXSPI data using an eDMA non-blocking method.
 *
 * This function writes/receives data to/from the FLEXSPI transmit/receive FIFO. This function is non-blocking.
 * @param base FLEXSPI peripheral base address.
 * @param handle Pointer to flexspi_edma_handle_t structure
 * @param xfer FLEXSPI transfer structure.
 */
status_t FLEXSPI_TransferEDMA(FLEXSPI_Type *base, flexspi_edma_handle_t *handle, flexspi_transfer_t *xfer);

/*!
 * @brief Aborts the transfer data using eDMA.
 *
 * This function aborts the transfer data using eDMA.
 *
 * @param base FLEXSPI peripheral base address.
 * @param handle Pointer to flexspi_edma_handle_t structure
 */
void FLEXSPI_TransferAbortEDMA(FLEXSPI_Type *base, flexspi_edma_handle_t *handle);

/*!
 * @brief Gets the transferred counts of transfer.
 *
 * @param base FLEXSPI peripheral base address.
 * @param handle Pointer to flexspi_edma_handle_t structure.
 * @param count Bytes transfer.
 * @retval kStatus_Success Succeed get the transfer count.
 * @retval kStatus_NoTransferInProgress There is not a non-blocking transaction currently in progress.
 */
status_t FLEXSPI_TransferGetCountEDMA(FLEXSPI_Type *base, flexspi_edma_handle_t *handle, size_t *count);

/* @} */

#if defined(__cplusplus)
}
#endif

/* @} */

#endif /* _FSL_FLEXSPI_EDMA_H_ */
