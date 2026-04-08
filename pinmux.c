//*****************************************************************************
// pinmux.c
//
// configure the device pins for different peripheral signals
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

//*****************************************************************************
// Pin assignments -- Anti-Theft Backpack Guardian
//
//  I2C (BMA222 accel 0x18 + BH1750 light 0x23)
//    PIN_01 : I2C0 SCL  (mode 1)
//    PIN_02 : I2C0 SDA  (mode 1)
//
//  OLED SSD1351 over GSPI
//    PIN_05 : GSPI CLK   (mode 7)
//    PIN_07 : GSPI MOSI  (mode 7)
//    PIN_62 : OLED CS    (GPIOA0 bit 0x80) -- GPIO out
//    PIN_63 : OLED RST   (GPIOA1 bit 0x01) -- GPIO out
//    PIN_64 : OLED DC    (GPIOA1 bit 0x02) -- GPIO out
//             (Adafruit_OLED.c writes to GPIOA1_BASE 0x2 for DC)
//
//  IR receiver
//    PIN_18 : GPIO input (GPIOA3 bit 0x10)
//
//  Buzzer (active-LOW)
//    PIN_15 : GPIO output (GPIOA2 bit 0x40)
//
//  GPS (UART1, 9600 baud)
//    PIN_58 : UART1 TX -> GPS RXD  (mode 6)
//    PIN_59 : UART1 RX <- GPS TXD  (mode 6)
//
//  Debug UART (UART0, 115200 baud)
//    PIN_55 : UART0 TX  (mode 3)
//    PIN_57 : UART0 RX  (mode 3)
//
//  NOTE: JTAG pins 16,17,19,20 must never be touched.
//*****************************************************************************

#include "pinmux.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_gpio.h"
#include "pin.h"
#include "rom.h"
#include "rom_map.h"
#include "gpio.h"
#include "prcm.h"

//*****************************************************************************
// Debug flags -- set any to 0 to disable that peripheral's pin config.
// Useful for isolating hardware issues during bring-up.
// All default to 1 (fully enabled) for normal operation.
//*****************************************************************************
#define PINMUX_ENABLE_I2C     1   // BMA222 accel + BH1750 light (PIN_01, PIN_02)
#define PINMUX_ENABLE_OLED    1   // SSD1351 OLED SPI + GPIO     (PIN_05,07,62,63,64)
#define PINMUX_ENABLE_IR      1   // IR receiver input            (PIN_18)
#define PINMUX_ENABLE_BUZZER  1   // Buzzer output                (PIN_15)
#define PINMUX_ENABLE_GPS     1   // GPS UART1                    (PIN_58, PIN_59)

//*****************************************************************************
void
PinMuxConfig(void)
{
    //
    // Enable peripheral clocks.
    // Always enable all GPIO banks -- they are shared across features.
    // Avoid enabling clocks for peripherals that are disabled below.
    //
    PRCMPeripheralClkEnable(PRCM_GPIOA0, PRCM_RUN_MODE_CLK);  // OLED CS  (PIN_62)
    PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_RUN_MODE_CLK);  // OLED RST/DC (PIN_63, PIN_64)
    PRCMPeripheralClkEnable(PRCM_GPIOA2, PRCM_RUN_MODE_CLK);  // Buzzer   (PIN_15)
    PRCMPeripheralClkEnable(PRCM_GPIOA3, PRCM_RUN_MODE_CLK);  // IR input (PIN_18)
    PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK);  // Debug UART0

#if PINMUX_ENABLE_GPS
    PRCMPeripheralClkEnable(PRCM_UARTA1, PRCM_RUN_MODE_CLK);  // GPS UART1
#endif

#if PINMUX_ENABLE_I2C
    PRCMPeripheralClkEnable(PRCM_I2CA0,  PRCM_RUN_MODE_CLK);  // I2C accel + light
#endif

#if PINMUX_ENABLE_OLED
    PRCMPeripheralClkEnable(PRCM_GSPI,   PRCM_RUN_MODE_CLK);  // GSPI for OLED
#endif

    //-------------------------------------------------------------------------
    // UART0 -- debug terminal (always on, needed for Report/UART_PRINT)
    //-------------------------------------------------------------------------
    PinTypeUART(PIN_55, PIN_MODE_3);  // UART0 TX
    PinTypeUART(PIN_57, PIN_MODE_3);  // UART0 RX

    //-------------------------------------------------------------------------
    // I2C -- BMA222 accel (0x18) + BH1750 light sensor (0x23)
    //-------------------------------------------------------------------------
#if PINMUX_ENABLE_I2C
    PinTypeI2C(PIN_01, PIN_MODE_1);   // I2C SCL
    PinTypeI2C(PIN_02, PIN_MODE_1);   // I2C SDA
#endif

    //-------------------------------------------------------------------------
    // OLED SSD1351 -- GSPI + 3 GPIO control lines
    //
    // CS  : PIN_62 -> GPIOA0 bit 0x80
    // RST : PIN_63 -> GPIOA1 bit 0x01
    // DC  : PIN_64 -> GPIOA1 bit 0x02  (Adafruit_OLED.c writes GPIOA1_BASE 0x2)
    //-------------------------------------------------------------------------
#if PINMUX_ENABLE_OLED
    PinTypeSPI(PIN_05, PIN_MODE_7);             // GSPI CLK
    PinTypeSPI(PIN_07, PIN_MODE_7);             // GSPI MOSI

    PinTypeGPIO(PIN_62, PIN_MODE_0, false);     // OLED CS
    GPIODirModeSet(GPIOA0_BASE, 0x80, GPIO_DIR_MODE_OUT);

    PinTypeGPIO(PIN_63, PIN_MODE_0, false);     // OLED RST
    GPIODirModeSet(GPIOA1_BASE, 0x01, GPIO_DIR_MODE_OUT);

    PinTypeGPIO(PIN_45, PIN_MODE_0, false);
    GPIODirModeSet(GPIOA3_BASE, 0x80, GPIO_DIR_MODE_OUT);
#endif

    //-------------------------------------------------------------------------
    // IR receiver -- GPIO input on PIN_18 -> GPIOA3 bit 0x10
    // InitIRGPIO() in main.c registers the interrupt -- pinmux just sets dir.
    //-------------------------------------------------------------------------
#if PINMUX_ENABLE_IR
    PinTypeGPIO(PIN_18, PIN_MODE_0, false);
    GPIODirModeSet(GPIOA3_BASE, 0x10, GPIO_DIR_MODE_IN);
#endif

    //-------------------------------------------------------------------------
    // Buzzer -- GPIO output on PIN_15 -> GPIOA2 bit 0x40 (active-LOW)
    // main.c drives HIGH immediately after PinMuxConfig() to prevent boot buzz.
    //-------------------------------------------------------------------------
#if PINMUX_ENABLE_BUZZER
    PinTypeGPIO(PIN_15, PIN_MODE_0, false);
    GPIODirModeSet(GPIOA2_BASE, 0x40, GPIO_DIR_MODE_OUT);
#endif

    //-------------------------------------------------------------------------
    // GPS -- UART1 on PIN_58 (TX) / PIN_59 (RX) at 9600 baud
    // main.c also calls ConfigureGpsPins() which re-applies these -- safe.
    //-------------------------------------------------------------------------
#if PINMUX_ENABLE_GPS
    PinTypeUART(PIN_58, PIN_MODE_6);  // UART1 TX -> GPS RXD
    PinTypeUART(PIN_59, PIN_MODE_6);  // UART1 RX <- GPS TXD
#endif
}
