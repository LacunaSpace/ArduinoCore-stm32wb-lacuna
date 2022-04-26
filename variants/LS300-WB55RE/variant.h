/*
 * Copyright (c) 2020 Thomas Roell.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Thomas Roell, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#ifndef _VARIANT_LS300_STM32WB55RE_
#define _VARIANT_LS300_STM32WB55RE_

// The definitions here needs a STM32WB core >=1.6.6
#define ARDUINO_STM32WB_VARIANT_COMPLIANCE 10606

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

#define STM32WB_CONFIG_LSECLK             32768
#define STM32WB_CONFIG_HSECLK             32000000
#define STM32WB_CONFIG_SYSOPT             (STM32WB_SYSTEM_OPTION_SMPS_INDUCTOR_10uH | STM32WB_SYSTEM_OPTION_SMPS_CURRENT_220mA)

#define STM32WB_CONFIG_PIN_VBUS           STM32WB_GPIO_PIN_NONE
#define STM32WB_CONFIG_PIN_BUTTON         STM32WB_GPIO_PIN_PH3

// #define STM32WB_CONFIG_PIN_VBAT_SWITCH    STM32WB_GPIO_PIN_PB0
#define STM32WB_CONFIG_VBAT_SENSE_CHANNEL STM32WB_ADC_CHANNEL_VBAT
#define STM32WB_CONFIG_VBAT_SENSE_DELAY   10000
#define STM32WB_CONFIG_VBAT_SENSE_PERIOD  STM32WB_ADC_VBAT_PERIOD
#define STM32WB_CONFIG_VBAT_SENSE_SCALE   (1.27 * 3.0)

#define USBCON

/** Master clock frequency */
#define VARIANT_MCK			  F_CPU

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
#include "USBAPI.h"
#include "Uart.h"
#endif // __cplusplus

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#define PINS_COUNT           (38u)
#define NUM_DIGITAL_PINS     (16u)
#define NUM_ANALOG_INPUTS    (7u)
#define NUM_ANALOG_OUTPUTS   (0u)

// LEDs
#define LED_BUILTIN          (2u)

/*
 * Analog pins
 */
#define PIN_A0               (16ul)
#define PIN_A1               (17ul)
#define PIN_A2               (21ul)
#define PIN_A3               (21ul)
#define PIN_A4               (20ul)
#define PIN_A5               (32ul)
#define PIN_A6               (33ul)	

static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t A2  = PIN_A2;
static const uint8_t A3  = PIN_A3;
static const uint8_t A4  = PIN_A4;
static const uint8_t A5  = PIN_A5;
static const uint8_t A6  = PIN_A6;
#define ADC_RESOLUTION		12

#define PIN_BUTTON           (26)
static const uint8_t BUTTON = PIN_BUTTON;

#define BOARD_LS200_V2 (0x02)
#define BOARD_LS200_V3 (0x03)
#define BOARD_LS200_V4 (0x04)
#define BOARD_LS200_V5 (0x05)
#define BOARD_LS300_V1 (0x06)

#if BOARD_VERSION == BOARD_LS300_V1
	#define LS300_V1
	#define LS_BOARD_STRING "LS300 v1"
#endif

#define BME280_WIRE Wire
#define LSM303_WIRE Wire

// LS300 BOARD
// #define BOARD_LS200
#define BOARD_LS300

// LS300 battery (3.3 Volt)
#define LS_ADC_AREF 3.3f
#define LS_BATVOLT_R1 1.0f 
#define LS_BATVOLT_R2 2.0f

// LS300 pins
// Button PC4 
#define LS_USER_BUTTON 26
// LED PB5
#define LS_LED_BLUE 2
#define LS_LED 2

// PC13 Unused
// #define E22_RXEN 7
// PB0 (pb3)
#define E22_BUSY 27
// PA0 (pa0)
#define E22_NRST 16
// PB4 (pb4)
#define E22_DIO1 3
// PA4
#define E22_NSS 4

#define LS_SKY_ANT 28
#define LS_SKY_CSD 29
#define LS_SKY_CTX 30
#define LS_SKY_CPS 31

// PB9
#define SD_ENABLE 9
// PB8
#define SD_SS 8
#define SDCARD_SS_PIN 8

// PC10
#define LS_GPS_PPS 35
// PA3
#define LS_GPS_RX 19
// PA2
#define LS_GPS_TX 18
// PB1
#define LS_GPS_ENABLE 7
#define LS_MODULE_ENABLE 7
// PA15
#define LS_GPS_INT 10
// PE4
#define LS_GPS_V_BCKP 24
// PC0
#define LS_GPS_STANDBY 36

// PC3
#define LS_BATVOLT_PIN 33
// PC1
#define LS_VERSION_MEAS 32
// PC6
#define LS_VERSION_ENABLE 34

// PA1
#define LS_HALL_OUT	17
// PB3
#define LS_HALL_ENABLE 5

// PA8
#define LS_LNA_CONTROL 20
#define LS_SW2_V2 20

// PA9
#define LS_TX 1
// PA10
#define LS_RX 0

// PC5
#define LS_ACC_INT2 37
// PB2
#define LS_ACC_INT1 6
#define LS_INT3 6

// Compatible LS200
#define LS_INT_ACC_1 6
#define LS_INT_ACC_2 37
// PIN_NONE
#define LS_INT_MAG 21

/*
 * Serial interfaces
 */

#define SERIAL_INTERFACES_COUNT 3

#define PIN_SERIAL1_RX       (0ul)
#define PIN_SERIAL1_TX       (1ul)

#define PIN_SERIAL2_RX       (19ul)
#define PIN_SERIAL2_TX       (18ul)

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MOSI         (11u)
#define PIN_SPI_MISO         (12u)
#define PIN_SPI_SCK          (13u)

static const uint8_t SS	  = (4u);
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (14u)
#define PIN_WIRE_SCL         (15u)

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

/*
 * I2S Interfaces
 */
#define I2S_INTERFACES_COUNT 0

// #define PIN_I2S_SD           (2u)
// #define PIN_I2S_WS           (4u)
// #define PIN_I2S_SCK          (5u)
// #define PIN_I2S_MLCK         (3u)

/*
 * PDM Interfaces
 */
#define PDM_INTERFACES_COUNT 1

#define PIN_PDM_CK           (8u)
#define PIN_PDM_DIN          (9u)

#define PWM_INSTANCE_COUNT  3

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
extern CDC  Serial;
extern Uart Serial1;
extern Uart Serial2;
#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL      Serial
#define SERIAL_PORT_MONITOR         Serial
#define SERIAL_PORT_HARDWARE1       Serial1
#define SERIAL_PORT_HARDWARE2       Serial2

// Alias SerialUSB to Serial
#define SerialUSB SERIAL_PORT_USBVIRTUAL

#endif /* _VARIANT_FIREFLY_STM32WB55CG_ */

