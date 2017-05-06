/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __BOARD_H
#define __BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drivers/Power.h>


// PASTED:::::

/** ============================================================================
 *  @file       CC2650_LAUNCHXL.h
 *
 *  @brief      CC2650 LaunchPad Board Specific header file.
 *
 *  NB! This is the board file for CC2650 LaunchPad PCB version 1.1
 *
 *  ============================================================================
 */
#ifndef __QC14_BOARD_H__
#define __QC14_BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

/** ============================================================================
 *  Includes
 *  ==========================================================================*/
#include <ti/drivers/PIN.h>
#include <driverlib/ioc.h>

/** ============================================================================
 *  Externs
 *  ==========================================================================*/
extern const PIN_Config BoardGpioInitTable[];

/** ============================================================================
 *  Defines
 *  ==========================================================================*/

/* Same RF Configuration as 7x7 EM */
#define CC2650EM_7ID

/* Mapping of pins to board signals using general board aliases
 *      <board signal alias>        <pin mapping>
 */

// Physical mating ports:

#define P1_TX       IOID_3
#define P1_RX       IOID_2
#define P2_TX       IOID_0
#define P2_RX       IOID_1
#define P3_TX       IOID_28
#define P3_RX       IOID_29
#define P4_TX       IOID_27
#define P4_RX       IOID_26

// Rocker switch:

#define SW_L        IOID_7
#define SW_R        IOID_9
#define SW_CLICK    IOID_8

// LED controller:

#define LED_GSCLK   IOID_10
#define LED_DIN     IOID_11
#define LED_DOUT    IOID_12
#define LED_CLK     IOID_13
#define LED_LE     IOID_14

// LED multiplexing:

#define MP_CTR_CLK  IOID_16
#define MP0_OUT     IOID_18
#define MP0_CLR     IOID_15
#define MP1_OUT     IOID_17
#define MP1_CLR     IOID_19

// Light sensor:

#define LIGHT       IOID_30

// External flash chip:

#define FLASH_TX    IOID_20
#define FLASH_RX    IOID_24
#define FLASH_CLK   IOID_21
#define FLASH_CS    IOID_23
#define FLASH_NHOLD IOID_22
#define FLASH_NWP   IOID_25

/** ============================================================================
 *  Instance identifiers
 *  ==========================================================================*/
/* Generic I2C instance identifiers */
#define Board_I2C                   QC14BOARD_I2C0
/* Generic SPI instance identifiers */
#define Board_SPI0                  QC14BOARD_SPI0
#define Board_SPI1                  QC14BOARD_SPI1
/* Generic UART instance identifiers */
#define Board_UART                  QC14BOARD_UART0
/* Generic Crypto instance identifiers */
#define Board_CRYPTO                QC14BOARD_CRYPTO0
/* Generic GPTimer instance identifiers */
#define Board_GPTIMER0A             QC14BOARD_GPTIMER0A
#define Board_GPTIMER0B             QC14BOARD_GPTIMER0B
#define Board_GPTIMER1A             QC14BOARD_GPTIMER1A
#define Board_GPTIMER1B             QC14BOARD_GPTIMER1B
#define Board_GPTIMER2A             QC14BOARD_GPTIMER2A
#define Board_GPTIMER2B             QC14BOARD_GPTIMER2B
#define Board_GPTIMER3A             QC14BOARD_GPTIMER3A
#define Board_GPTIMER3B             QC14BOARD_GPTIMER3B
/* Generic PWM instance identifiers */
#define Board_PWM0                  QC14BOARD_PWM0
#define Board_PWM1                  QC14BOARD_PWM1
#define Board_PWM2                  QC14BOARD_PWM2
#define Board_PWM3                  QC14BOARD_PWM3
#define Board_PWM4                  QC14BOARD_PWM4
#define Board_PWM5                  QC14BOARD_PWM5
#define Board_PWM6                  QC14BOARD_PWM6
#define Board_PWM7                  QC14BOARD_PWM7

/** ============================================================================
 *  Number of peripherals and their names
 *  ==========================================================================*/

/*!
 *  @def    QC14BOARD_I2CName
 *  @brief  Enum of I2C names on the badge
 */
typedef enum QC14BOARD_I2CName {
    QC14BOARD_I2C0 = 0,

    QC14BOARD_I2CCOUNT
} QC14BOARD_I2CName;

/*!
 *  @def    QC14BOARD_CryptoName
 *  @brief  Enum of Crypto names on the badge
 */
typedef enum QC14BOARD_CryptoName {
    QC14BOARD_CRYPTO0 = 0,

    QC14BOARD_CRYPTOCOUNT
} QC14BOARD_CryptoName;


/*!
 *  @def    QC14BOARD_SPIName
 *  @brief  Enum of SPI names on the badge
 */
typedef enum QC14BOARD_SPIName {
    QC14BOARD_SPI0 = 0,
    QC14BOARD_SPI1,

    QC14BOARD_SPICOUNT
} QC14BOARD_SPIName;

/*!
 *  @def    QC14BOARD_TRNGName
 *  @brief  Enum of TRNG names on the board
 */
typedef enum QC14BOARD_TRNGName {
    QC14BOARD_TRNG0 = 0,
    QC14BOARD_TRNGCOUNT
} QC14BOARD_TRNGName;

/*!
 *  @def    QC14BOARD_UARTName
 *  @brief  Enum of UARTs on the badge
 */
typedef enum QC14BOARD_UARTName {
    QC14BOARD_UART0 = 0,

    QC14BOARD_UARTCOUNT
} QC14BOARD_UARTName;

/*!
 *  @def    QC14BOARD_UdmaName
 *  @brief  Enum of DMA buffers
 */
typedef enum QC14BOARD_UdmaName {
    QC14BOARD_UDMA0 = 0,

    QC14BOARD_UDMACOUNT
} QC14BOARD_UdmaName;

/*!
 *  @def    QC14BOARD_GPTimerName
 *  @brief  Enum of GPTimer parts
 */
typedef enum QC14BOARD_GPTimerName
{
    QC14BOARD_GPTIMER0A = 0,
    QC14BOARD_GPTIMER0B,
    QC14BOARD_GPTIMER1A,
    QC14BOARD_GPTIMER1B,
    QC14BOARD_GPTIMER2A,
    QC14BOARD_GPTIMER2B,
    QC14BOARD_GPTIMER3A,
    QC14BOARD_GPTIMER3B,
    QC14BOARD_GPTIMERPARTSCOUNT
} QC14BOARD_GPTimerName;

/*!
 *  @def    QC14BOARD_GPTimers
 *  @brief  Enum of GPTimers
 */
typedef enum QC14BOARD_GPTimers
{
    QC14BOARD_GPTIMER0 = 0,
    QC14BOARD_GPTIMER1,
    QC14BOARD_GPTIMER2,
    QC14BOARD_GPTIMER3,
    QC14BOARD_GPTIMERCOUNT
} QC14BOARD_GPTimers;

/*!
 *  @def    QC14BOARD_PWM
 *  @brief  Enum of PWM outputs on the board
 */
typedef enum QC14BOARD_PWM
{
    QC14BOARD_PWM0 = 0,
    QC14BOARD_PWM1,
    QC14BOARD_PWM2,
    QC14BOARD_PWM3,
    QC14BOARD_PWM4,
    QC14BOARD_PWM5,
    QC14BOARD_PWM6,
    QC14BOARD_PWM7,
    QC14BOARD_PWMCOUNT
} QC14BOARD_PWM;

/*!
 *  @def    QC14BOARD_ADCBufName
 *  @brief  Enum of ADCs
 */
typedef enum QC14BOARD_ADCBufName {
    QC14BOARD_ADCBuf0 = 0,
    QC14BOARD_ADCBufCOUNT
} QC14BOARD_ADCBufName;


/*!
 *  @def    QC14BOARD_ADCName
 *  @brief  Enum of ADCs
 */
typedef enum QC14BOARD_ADCName {
    QC14BOARD_ADC0 = 0,
    QC14BOARD_ADC1,
    QC14BOARD_ADC2,
    QC14BOARD_ADC3,
    QC14BOARD_ADC4,
    QC14BOARD_ADC5,
    QC14BOARD_ADC6,
    QC14BOARD_ADC7,
    QC14BOARD_ADCDCOUPL,
    QC14BOARD_ADCVSS,
    QC14BOARD_ADCVDDS,
    QC14BOARD_ADCCOUNT
} QC14BOARD_ADCName;


#ifdef __cplusplus
}
#endif

#endif /* __QC14BOARD_BOARD_H__ */


/* These #defines allow us to reuse TI-RTOS across other device families */

#define     Board_UART0             Board_UART
#define     Board_AES0              Board_AES
#define     Board_WATCHDOG0         Board_WATCHDOG

#define     Board_ADC0              QC14BOARD_ADCVSS
#define     Board_ADC1              QC14BOARD_ADCVDDS

#define     Board_ADCBuf0           QC14BOARD_ADCBuf0
#define     Board_ADCBufChannel0    (0)
#define     Board_ADCBufChannel1    (1)

#define     Board_initGeneral() { \
    Power_init(); \
    if (PIN_init(BoardGpioInitTable) != PIN_SUCCESS) \
        {System_abort("Error with PIN_init\n"); \
    } \
}

#define     Board_initGPIO()
#define     Board_initPWM()         PWM_init()
#define     Board_initSPI()         SPI_init()
#define     Board_initUART()        UART_init()
#define     Board_initWatchdog()    Watchdog_init()
#define     Board_initADCBuf()      ADCBuf_init()
#define     Board_initADC()         ADC_init()
#define     GPIO_toggle(n)
#define     GPIO_write(n,m)

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
