/** \file
    ---------------------------------------------------------------
    SPDX-License-Identifier: BSD-3-Clause

    Copyright (c) 2024, Renesas Electronics Corporation and/or its affiliates


    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of Renesas nor the names of its
       contributors may be used to endorse or promote products derived from this
       software without specific prior written permission.



   THIS SOFTWARE IS PROVIDED BY Renesas "AS IS" AND ANY EXPRESS
   OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
   OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL RENESAS OR CONTRIBUTORS BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    ---------------------------------------------------------------

    Project     : PTX105R Tunneling
    Module      : PTX105R Tunneling
    File        : sketch.cpp

    Description : PTX Tunneling sketch.
*/

/*
  Tunneling for Renesas PTX105R
  Language: Arduino

  This program provides a UART interface for an embedded PTX105R NFC chip using
  the SPI protocol to enable the usage with the "NFC IoT Tuner" PC application.

  The sketch translates incoming UART (from USB) frames and forwards them to the
  PTX105R chip using the SPI protocol. The responses get similarly translated
  back to UART frames and sent to the host.

  created 20.09.2023 by Renesas Electronics Corporation

  This program is designed to work with boards:
  - Arduino® UNO R4 Minima
  - Arduino® UNO R4 WiFi
*/

#include <SPI.h>

#include "ptx_tunneling.h"

void setup(void) {
  Serial.begin(115200U, SERIAL_8N1);
  SPI.begin();

  pinMode(PIN_SPI_CS, PinMode::OUTPUT);
  digitalWrite(PIN_SPI_CS, PinStatus::HIGH);

  pinMode(PIN_D9, PinMode::INPUT_PULLDOWN);  // PIN_D9 is used for PTX IRQ pin

  while (!Serial)  // wait for USB-serial to be ready/connected
    delay(33);

  ptxTunneling_init();
}

void loop(void) { ptxTunneling_poll(NULL); }