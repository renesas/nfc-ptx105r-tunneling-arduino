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
    Module      : Tunneling
    File        : ptx_tunneling.h

    Description : Interface for tunneling library
*/

#ifndef __PTX_TUNNELING_H__
#define __PTX_TUNNELING_H__

#ifdef __cplusplus
extern "C"
{
#endif
    struct ptxHal;
    typedef struct ptxHal ptxHal_t;

    /**
     * @brief Possible status codes
     *
     */
    typedef enum
    {
        ptxStatus_Success,
        ptxStatus_InvalidParameter,
    } ptxStatus_t;

    /**
     * @brief Initialize the internal tunneling logic.
     *
     * This function can be called anytime to set up or reset the internal state
     * machines and variables.
     */
    void ptxTunneling_init();

    /**
     * @brief Main function of PTX Tunneling function.
     *
     * This function process pending tasks and returns, it must be called repeatedly
     * from the main loop to keep the communication working.
     *
     * The used resources (SPI, UART, etc) must already be initialized and the function
     * @ref ptxTunneling_init must be called before calling this function.
     *
     * @param context application context defined and allocated by the user in advance
     */
    void ptxTunneling_poll(ptxHal_t *context);

#ifdef __cplusplus
}
#endif

#endif
