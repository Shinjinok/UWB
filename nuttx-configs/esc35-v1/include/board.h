/************************************************************************************
 * configs/esc35-v1/include/board.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           David Sidrane <david_s5@nscdg.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __CONFIGS_ESC35_V1_INCLUDE_BOARD_H
#define __CONFIGS_ESC35_V1_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif
#include "stm32_rcc.h"
#include "stm32_sdio.h"
#include "stm32.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Clocking *************************************************************************/

/* HSI - 8 MHz RC factory-trimmed
 * LSI - 40 KHz RC (30-60KHz, uncalibrated)
 * HSE - On-board crystal frequency is 8MHz
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        8000000ul

#define STM32_HSI_FREQUENCY     8000000ul
#define STM32_LSI_FREQUENCY     40000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* PLL source is HSE/1, PLL multipler is 9: PLL frequency is 8MHz (XTAL) x 9 = 72MHz */

#define STM32_CFGR_PLLSRC       RCC_CFGR_PLLSRC
#define STM32_CFGR_PLLXTPRE     0
#define STM32_CFGR_PLLMUL       RCC_CFGR_PLLMUL_CLKx9
#define STM32_PLL_FREQUENCY     (9*STM32_BOARD_XTAL)

/* Use the PLL and set the SYSCLK source to be the PLL */

#define STM32_SYSCLK_SW         RCC_CFGR_SW_PLL
#define STM32_SYSCLK_SWS        RCC_CFGR_SWS_PLL
#define STM32_SYSCLK_FREQUENCY  STM32_PLL_FREQUENCY

/* AHB clock (HCLK) is SYSCLK (72MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY    STM32_PLL_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY    /* same as above, to satisfy compiler */

/* APB2 clock (PCLK2) is HCLK (72MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY   STM32_HCLK_FREQUENCY

 /* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,15-17 are on APB2, others on APB1 */

#define STM32_TIM18_FREQUENCY   STM32_HCLK_FREQUENCY
#define STM32_TIM27_FREQUENCY   STM32_HCLK_FREQUENCY

/* APB2 timer 1 will receive PCLK2. */

#define STM32_APB2_TIM1_CLKIN   (STM32_PCLK2_FREQUENCY)

/* APB1 clock (PCLK1) is HCLK/2 (36MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd2
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* APB1 timers 2-4 will be twice PCLK1 (I presume the remaining will receive PCLK1) */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (STM32_PCLK1_FREQUENCY)

/* USB divider -- Divide PLL clock by 1.5 */

#define STM32_CFGR_USBPRE       0

/* Leds *************************************************************************/

/* LED index values for use with board_setled() */

#define BOARD_LED1                0
#define BOARD_LED_RED             BOARD_LED1
#define BOARD_LED2                1
#define BOARD_LED_GREEN           BOARD_LED2
#define BOARD_LED3                2
#define BOARD_LED_BLUE            BOARD_LED3
#define BOARD_NLEDS               3

/* LED bits for use with board_setleds() */

#define BOARD_LED_RED_BIT     (1 << BOARD_LED_RED)
#define BOARD_LED_GREEN_BIT   (1 << BOARD_LED_GREEN)
#define BOARD_LED_BLUE_BIT    (1 << BOARD_LED_BLUE)

/* TODO:define these
 * These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 *
 * defined.  In that case, the usage by the board port is as follows:
 *
 *   SYMBOL                     Meaning                      LED state
 *                                                         Red   Green Blue
 *   ------------------------  --------------------------  ------ ------ ----*/

#define LED_STARTED          0 /* NuttX has been started   OFF    OFF   OFF */
#define LED_HEAPALLOCATE     1 /* Heap has been allocated  OFF    OFF   ON  */
#define LED_IRQSENABLED      2 /* Interrupts enabled       OFF    ON    OFF */
#define LED_STACKCREATED     3 /* Idle stack created       OFF    ON    ON  */
#define LED_INIRQ            4 /* In an interrupt          N/C    GLOW  N/C */
#define LED_SIGNAL           5 /* In a signal handler      N/C    GLOW  N/C */
#define LED_ASSERTION        6 /* An assertion failed      GLOW   GLOW  N/C */
#define LED_PANIC            7 /* The system has crashed   Blk    OFF   N/C */
#define LED_IDLE             8  /* MCU is is sleep mode    ON     OFF   OFF */

/*
 * Thus if the blue is statically on, NuttX has successfully booted and is,
 * apparently, running normally.  If the Red LED is flashing at
 * approximately 2Hz, then a fatal error has been detected and the system
 * has halted.
 *
 */


/* Alternate function pin selections ************************************************/

/* UARTs */

#define GPIO_USART2_RX         GPIO_USART2_RX_3
#define GPIO_USART2_TX         GPIO_USART2_TX_3

/* CAN
 *
 * CAN1 is routed to the onboard transceiver.
 * CAN2 is routed to the onboard transceiver.
 */

#define GPIO_CAN1_RX     GPIO_CAN_RX_3
#define GPIO_CAN1_TX     GPIO_CAN_TX_3
//#define GPIO_CAN1_RX     GPIO_CAN1_RX_2
//#define GPIO_CAN1_TX     GPIO_CAN1_TX_2
//#define GPIO_CAN2_RX     GPIO_CAN2_RX_2
//#define GPIO_CAN2_TX     GPIO_CAN2_TX_2

/* TIMERS */

#define GPIO_TIM3_CH2OUT        GPIO_TIM3_CH2OUT_3
#define GPIO_TIM3_CH3OUT        GPIO_TIM3_CH3OUT_2
#define GPIO_TIM3_CH4OUT        GPIO_TIM3_CH4OUT_2

/* Probes unused */
#define PROBE_INIT(mask)
#define PROBE(n,s)
#define PROBE_MARK(n)

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/
/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void stm32_boardinitialize(void);

/************************************************************************************
 * Name: stm32_ledinit, stm32_setled, and stm32_setleds
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board LEDs.  If
 *   CONFIG_ARCH_LEDS is not defined, then the following interfaces are available to
 *   control the LEDs from user applications.
 *
 ************************************************************************************/

#ifndef CONFIG_ARCH_LEDS
void stm32_led_initialize(void);
void stm32_setled(int led, bool ledon);
void stm32_setleds(uint8_t ledset);
#endif

#if  !defined(CONFIG_NSH_LIBRARY)
int app_archinitialize(void);
#else
#define app_archinitialize()  (-ENOSYS)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_ESC35_V1_INCLUDE_BOARD_H */
