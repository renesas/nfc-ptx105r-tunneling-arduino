Tunneling for Renesas PTX105R

# Overview

This Library provides a UART interface for an embedded PTX105R NFC chip using
the SPI protocol to enable the usage with the "NFC IoT Tuner" PC application.

The tunneling library translates incoming UART (from USB) frames and forwards them to the
PTX105R chip using the SPI protocol. The responses get similarly translated
back to UART frames and sent to the host.

# Example

The library contains the `PTX105R_Tunneling` example sketch, that sets up the required communication interfaces and invokes the API functions from the precompiled tunneling library. The auxiliary file `ptx_tunneling_hal.cpp` implements the low-level accessors to the actual hardware. Should the peripherals/pins in a custom project differ from the default, this file can be adapted to the new requirements.

# Hardware Compatibility

This program is designed to work with the Renesas UNO boards:
- Arduino® UNO R4 Minima
- Arduino® UNO R4 WiFi

Compatible shield:
- WS004-NFCShield-PoCZ PTX105R Arduino Shield Rev A

## PTX105R Arduino Shield

The shield must be configured for SPI using the pin-caps, as indicated on the top side.

# Required Software

- Arduino IDE
- [NFC IoT Tuner](https://www.renesas.com/en/products/wireless-connectivity/nfc/ptx105r-mid-power-multi-protocol-nfc-forum-compliant-reader/WS031-NFCShld-REFZ-PTX105R-Arduino-Shield-Reference-Design)
