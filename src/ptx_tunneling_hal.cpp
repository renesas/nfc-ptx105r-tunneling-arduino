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
    File        : ptx_tunneling_hal.cpp

    Description : Implementation of PTX105R Tunneling hardware abstraction
                  layer.
*/

#include "ptx_tunneling_hal.h"

#include <Arduino.h>
#include <SPI.h>
#include <Serial.h>

#include <array>

#include "ptx_tunneling.h"

namespace {
const uint8_t OFFSET_CMD_LENGTH_BYTE = 1U;
const uint8_t COMMS_HEADER_SIZE = 2U;
const uint8_t CMD_CODE_TUNNELING_MSG = 0x55U;
const uint16_t COMMS_MAX_MESSAGE_LENGTH = 280U;

std::array<uint8_t, 260>
    spiBuffer;  // temporary buffer to send and receive data on SPI
std::array<uint8_t, 2048> commandBuffer;  // received commands are stored here
uint16_t readPos = 0;
uint16_t writePos = 0;
std::array<uint8_t, COMMS_MAX_MESSAGE_LENGTH> rx;
uint16_t rxi = 0;
}  // namespace

bool ptxTunneling_GPIO_IsIrqPinAsserted(ptxHal_t *) {
  return digitalRead(PIN_D9) == PinStatus::HIGH;
}

int ptxTunneling_UART_rxLength(ptxHal_t *) {
  noInterrupts();
  while (Serial.available()) {
    // receive the header and the length byte
    rx[rxi] = static_cast<uint8_t>(Serial.read());
    if (rxi != 0 ||
        rx[rxi] == CMD_CODE_TUNNELING_MSG) {  // if there are some invalid data
                                              // received while waiting for sync
                                              // byte, just
      // discard them
      rxi++;
    }

    if (rxi >= COMMS_HEADER_SIZE) {
      const auto packLen = static_cast<uint16_t>(
          COMMS_HEADER_SIZE +
          (rx[OFFSET_CMD_LENGTH_BYTE] == 0 ? 256 : rx[OFFSET_CMD_LENGTH_BYTE]));
      if (rxi >= packLen) {
        // whole packet has been received, append to command buffer
        memcpy(commandBuffer.data() + writePos, rx.data(), rxi);
        writePos = static_cast<uint16_t>(writePos + rxi);
        rxi = 0;
      }
    }
  }
  const auto count = static_cast<uint16_t>(writePos - readPos);
  interrupts();
  return count;
}

int ptxTunneling_UART_read(ptxHal_t *, uint8_t *buf, unsigned int len) {
  assert(len <= commandBuffer.size());
  noInterrupts();
  auto readCount = static_cast<size_t>(writePos - readPos);
  if (readCount > len) readCount = len;

  memcpy(buf, commandBuffer.data() + readPos, (size_t)readCount);

  readPos = static_cast<uint16_t>(readPos + readCount);
  if (readPos == writePos) {  // buffer got empty
    readPos = 0;
    writePos = 0;
  }
  interrupts();
  return readCount;
}

int ptxTunneling_UART_write(ptxHal_t *, const uint8_t *buf, unsigned int len) {
  if (len) {
    Serial.write(buf, len);
  }
  return 0;
}

void ptxTunneling_Timer_stopwatchStart(ptxHal_t *, ptxTimeDiff_t *startVal) {
  *startVal = millis();
}

void ptxTunneling_Timer_stopwatchStop(ptxHal_t *, ptxTimeDiff_t *startStopVal) {
  *startStopVal = millis() - *startStopVal;
}

void ptxTunneling_Timer_ThreadSleep(ptxHal_t *, uint32_t msSleep) {
  delay(msSleep);
}

int ptxTunneling_SPI_trx(ptxHal_t *, uint8_t *const txBuf[],
                         const size_t txLen[], size_t numBuffers,
                         uint8_t *rxBuf, size_t *rxLen) {
  bool status = true;
  if (txBuf && txLen) {
    /* At this point the SPI transfer operation is triggered */
    digitalWrite(PIN_SPI_CS, PinStatus::LOW);
    uint8_t index = 0;
    while (status && (index < numBuffers)) {
      if (txBuf[index] && (txLen[index] > 0) && rxBuf) {
        memcpy(spiBuffer.data(), txBuf[index], txLen[index]);
        SPI.transfer(spiBuffer.data(), txLen[index]);
        memcpy(rxBuf, spiBuffer.data(), *rxLen);
      } else {
        status = false;
      }
      index++;
    }
    /* Tx part of the overall transaction. */
  } else {
    /* Let's see if there is something to read. */
    if (rxBuf && rxLen && *rxLen) {
      SPI.transfer(rxBuf, *rxLen);
    } else {
      status = false;
    }
  }
  /* In any case, at this point the SPI transfer operation is finished */
  digitalWrite(PIN_SPI_CS, PinStatus::HIGH);
  return (!status);
}

void ptxTunneling_NVIC_disableInterrupts() {
  // disabling interrupts not needed in this sample, since the SPI/UART
  // communication uses neither interrupts nor multithreading, therefore no race
  // conditions can occur during queue operations
}

void ptxTunneling_NVIC_enableInterrupts() {
  // enabling interrupts not needed in this sample, since the SPI/UART
  // communication uses neither interrupts nor multithreading, therefore no race
  // conditions can occur during queue operations
}
