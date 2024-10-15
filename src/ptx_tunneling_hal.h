/*
    ---------------------------------------------------------------
    SPDX-License-Identifier: BSD-3-Clause

    Copyright (c) 2024, Renesas Electronics Corporation and/or its affiliates


    Redistribution and use in source and binary forms, with or without modification,
    are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice, this list of
       conditions and the following disclaimer in the documentation and/or other
       materials provided with the distribution.

    3. Neither the name of Renesas nor the names of its
       contributors may be used to endorse or promote products derived from this
       software without specific prior written permission.



    THIS SOFTWARE IS PROVIDED BY Renesas "AS IS" AND ANY EXPRESS
    OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
    OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL RENESAS OR CONTRIBUTORS BE
    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
    GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
    HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
    OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    ---------------------------------------------------------------

    Project     : PtxTunneling
    Module      : HAL
    File        : ptx_tunneling_hal.h

    Description : Interface for tunneling HAL
*/


#ifndef __PTX_TUNNELING_HAL_H__
#define __PTX_TUNNELING_HAL_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* including general status and hal stucture */
#include "ptx_tunneling.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Time difference type.
     */
    typedef uint64_t ptxTimeDiff_t;

    /**
     * @brief Returns state of IRQ pin.
     *
     * @param[in] context user defined structure given to @ref ptxTunneling_poll.
     * @return true PTX100x chip interrupt pin is asserted.
     * @return false There is no pending interrupt.
     */
    bool ptxTunneling_GPIO_IsIrqPinAsserted(ptxHal_t *context);

    /**
     * @brief Tells if there is data in receive buffer to read.
     *
     * @param[in] context user defined structure given to @ref ptxTunneling_poll.
     * @return number of bytes to be read.
     */
    int ptxTunneling_UART_rxLength(ptxHal_t *context);

    /**
     * @brief Read data from incoming buffer
     *
     * @param[in] context user defined structure given to @ref ptxTunneling_poll.
     * @param[in] buf destination buffer.
     * @param[in] len maximal data length to be read.
     * @return amount of bytes put into buffer.
     */
    int ptxTunneling_UART_read(ptxHal_t *context, uint8_t *buf, unsigned int len);

    /**
     * @brief Send data via UART.
     *
     * @param[in] context user defined structure given to @ref ptxTunneling_poll.
     * @param[in] buf data to be sent.
     * @param[in] len number of bytes to be sent.
     * @return Status, indicating whether the operation was successful (=0) or an error code.
     */
    int ptxTunneling_UART_write(ptxHal_t *context, const uint8_t *buf, unsigned int len);

    /**
     * @brief Start a time measurement or get a time value.
     *
     * @param[in] context user defined structure given to @ref ptxTunneling_poll.
     * @param[in,out] startVal pointer to a variable that will receive the start value.
     *
     * @return Status, indicating whether the operation was successful .
     *
     */
    void ptxTunneling_Timer_stopwatchStart(ptxHal_t *context, ptxTimeDiff_t *startVal);

    /**
     * @brief End a time measurement.
     *
     * @param[in] context user defined structure given to @ref ptxTunneling_poll.
     * @param[in,out] startStopVal pointer to a variable that provides the start time and that
     * will receive the time span. The time unit is microseconds.
     *
     * @return Status, indicating whether the operation was successful. See ptxStatus_t.
     *
     */
    void ptxTunneling_Timer_stopwatchStop(ptxHal_t *context, ptxTimeDiff_t *startStopVal);

    /**
     * @brief Sleep for a number of ms.
     *
     * @param[in] context user defined structure given to @ref ptxTunneling_poll.
     * @param[in] msSleep number of ms to suspend the current thread. If 0, a Yield
     * operation is performed.
     *
     */
    void ptxTunneling_Timer_ThreadSleep(ptxHal_t *context, uint32_t msSleep);

    /**
     * @brief Exchange content of multiple buffers via the SPI Interface, adding SOF if required.
     *
     * Sends the content of multiple buffers and prepends SOF, if set in the transmission parameter
     * field of the component. In an implementation-specific way, the function concatenates sending
     * of content of multiple buffers. The implementation is hardware-specific, but the interface is
     * done such that the underlying code can be tweaked for maximum performance (e.g. Gather-DMA on
     * some embedded systems, of just multi-send on others).
     *
     * @param[in] context      user defined structure given to @ref ptxTunneling_poll.
     * @param[in] txBuf        array of pointers to buffers with data to write.
     * @param[in] txLen        array of Number of bytes in the respective buffer.
     * @param[in] numBuffers   number of buffers whose data to transmit.
     * @param[in,out] rxBuf    pointer to a buffer with data to write.
     * @param[in,out] rxLen    pointer to a variable providing the maximum number of bytes to
     * read and receiving the actual number of bytes read.
     *
     * @note If txBuf or txLen are NULL, then only receive operation is performed (if rx buffer and
     * length are not NULL). For transmitting: individual buffers may be NULL or the length of
     * individual buffers may be zero. It is required however that the total number of bytes to
     * transfer is greater than zero and within the range the @b length @b field of the PTX-specific
     * frame can transfer.
     *
     * @return Status, indicating whether the operation was successful (=0) or an error code.
     *
     */
    int ptxTunneling_SPI_trx(ptxHal_t *context, uint8_t *const txBuf[], const size_t txLen[],
        size_t numBuffers, uint8_t *rxBuf, size_t *rxLen);

    /**
     * @brief Disable interrupts
     *
     */
    void ptxTunneling_NVIC_disableInterrupts();

    /**
     * @brief Enable interrupts
     *
     */
    void ptxTunneling_NVIC_enableInterrupts();

#ifdef __cplusplus
}
#endif

#endif
