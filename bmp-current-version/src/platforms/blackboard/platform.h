/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements the platform specific functions for the STM32
 * implementation.
 */
#ifndef __PLATFORM_H
#define __PLATFORM_H

#include "gpio.h"
#include "timing.h"
#include "timing_stm32.h"

#include <setjmp.h>



#define PLATFORM_HAS_TRACESWO
#define PLATFORM_IDENT "(BlackBoard) "

/* Important pin mappings for STM32 implementation:
 *
 * nTRST =    PC1
 * nRST_OUT = PC8
 * TDI =      PC2
 * TMS =      PC4 (input for SWDP)
 * TCK =      PC5/SWCLK
 * TDO =      PC6 (input for TRACESWO
 * nRST =     PC8
 *
 * Force DFU mode button: PA0
 */
/* Hardware definitions... */
#define JTAG_PORT 	GPIOC
#define TDI_PORT	JTAG_PORT
#define TMS_PORT	JTAG_PORT
#define TCK_PORT	JTAG_PORT
#define TDO_PORT	GPIOC
#define TDI_PIN		GPIO7
#define TMS_PIN		GPIO4
#define TCK_PIN		GPIO5
#define TDO_PIN		GPIO6

#define SWDIO_PORT 	JTAG_PORT
#define SWCLK_PORT 	JTAG_PORT
#define SWDIO_PIN	TMS_PIN
#define SWCLK_PIN	TCK_PIN

#define TRST_PORT	GPIOC
#define TRST_PIN	GPIO1
#define NRST_PORT	GPIOC
#define NRST_PIN	GPIO8

/*
#define PWR_BR_PORT GPIOB
#define PWR_BR_PIN GPIO8
*/
#define I2CLED_PORT GPIOC
#define LED_I2C GPIO13
#define LED_PORT GPIOA
#define LED_PORT_UART GPIOC
#define LED_UART GPIO13
#define LED_IDLE_RUN GPIO6
#define LED_ERROR GPIO7
#define LED_BOOTLOADER GPIO13
/*
#define USBUSART USART1
#define USBUSART_CR1 USART1_CR1
#define USBUSART_DR USART1_DR
#define USBUSART_IRQ NVIC_USART1_IRQ
#define USBUSART_CLK RCC_USART1
#define USBUSART_PORT GPIOA
#define USBUSART_TX_PIN GPIO9
#define USBUSART_RX_PIN GPIO10
#define USBUSART_ISR(x) usart1_isr(x)
#define USBUSART_DMA_BUS DMA2
#define USBUSART_DMA_CLK RCC_DMA2
#define USBUSART_DMA_TX_CHAN DMA_STREAM7
#define USBUSART_DMA_TX_IRQ NVIC_DMA2_STREAM7_IRQ
#define USBUSART_DMA_TX_ISR(x) dma2_stream7_isr(x)
#define USBUSART_DMA_RX_CHAN DMA_STREAM5
#define USBUSART_DMA_RX_IRQ NVIC_DMA2_STREAM5_IRQ
#define USBUSART_DMA_RX_ISR(x) dma2_stream5_isr(x)
// For STM32F4 DMA trigger source must be specified 
#define USBUSART_DMA_TRG DMA_SxCR_CHSEL_4
*/
#define USBUSART USART3
#define USBUSART_CR1 USART3_CR1
#define USBUSART_DR USART3_DR
#define USBUSART_IRQ NVIC_USART3_IRQ
#define USBUSART_CLK RCC_USART3
#define USBUSART_PORT GPIOD
#define USBUSART_TX_PIN GPIO8
#define USBUSART_RX_PIN GPIO9
#define USBUSART_ISR(x) usart3_isr(x)
#define USBUSART_DMA_BUS DMA1
#define USBUSART_DMA_CLK RCC_DMA1
#define USBUSART_DMA_TX_CHAN DMA_STREAM3
#define USBUSART_DMA_TX_IRQ NVIC_DMA1_STREAM3_IRQ
#define USBUSART_DMA_TX_ISR(x) dma1_stream3_isr(x)
#define USBUSART_DMA_RX_CHAN DMA_STREAM1
#define USBUSART_DMA_RX_IRQ NVIC_DMA1_STREAM1_IRQ
#define USBUSART_DMA_RX_ISR(x) dma1_stream1_isr(x)
/* For STM32F4 DMA trigger source must be specified */
#define USBUSART_DMA_TRG DMA_SxCR_CHSEL_4

#define PLATFORM_HAS_SLCAN		1
#define CAN_APB_FREQUENCY  (54*1000*1000L)
#define CAN_RX0_IRQ   NVIC_CAN1_RX0_IRQ
#define CAN_RX0_ISR   can1_rx0_isr
#define CAN_TX_IRQ    NVIC_CAN1_TX_IRQ
#define CAN_TX_ISR    can1_tx_isr

#define BOOTMAGIC0 0xb007da7a
#define BOOTMAGIC1 0xbaadfeed
#define BOOTMAGIC2 0xbaadfeed
#define BOOTMAGIC3 0xb007da7a
#define BOOTMAGIC4 0xbeedfaad
#define BOOTMAGIC5 0xb007da7a
#define BOOTMAGIC6 0xb006faad
#define BOOTMAGIC7 0xb007d07a

#define TMS_SET_MODE() \
	gpio_mode_setup(TMS_PORT, GPIO_MODE_OUTPUT, \
	                GPIO_PUPD_NONE, TMS_PIN);
#define SWDIO_MODE_FLOAT() \
	gpio_mode_setup(SWDIO_PORT, GPIO_MODE_INPUT, \
	                GPIO_PUPD_NONE, SWDIO_PIN);

#define SWDIO_MODE_DRIVE() \
	gpio_mode_setup(SWDIO_PORT, GPIO_MODE_OUTPUT, \
	                GPIO_PUPD_NONE, SWDIO_PIN);
#define UART_PIN_SETUP() do { \
	gpio_mode_setup(USBUSART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, \
	                USBUSART_TX_PIN); \
	gpio_set_output_options(USBUSART_PORT, GPIO_OTYPE_PP, \
					GPIO_OSPEED_100MHZ, USBUSART_TX_PIN); \
	gpio_set_af(USBUSART_PORT, GPIO_AF7, USBUSART_TX_PIN); \
	gpio_mode_setup(USBUSART_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, \
	                USBUSART_RX_PIN); \
	gpio_set_output_options(USBUSART_PORT, GPIO_OTYPE_OD, \
					GPIO_OSPEED_100MHZ, USBUSART_RX_PIN); \
	gpio_set_af(USBUSART_PORT, GPIO_AF7, USBUSART_RX_PIN); \
} while(0)

#define USB_DRIVER      stm32f207_usb_driver
#define USB_IRQ         NVIC_OTG_HS_IRQ
#define USB_ISR(x)      otg_hs_isr(x)
/* Interrupt priorities.  Low numbers are high priority.
 * TIM3 is used for traceswo capture and must be highest priority.
 */
#define IRQ_PRI_USB		(1 << 4)
#define IRQ_PRI_USBUSART	(2 << 4)
#define IRQ_PRI_USBUSART_DMA 	(2 << 4)
#define IRQ_PRI_TRACE		(0 << 4)
#define IRQ_PRI_CAN_RX0			(6 << 4)
#define IRQ_PRI_CAN_TX			(7 << 4)

#define TRACE_TIM TIM3
#define TRACE_TIM_CLK_EN() rcc_periph_clock_enable(RCC_TIM3)
#define TRACE_IRQ   NVIC_TIM3_IRQ
#define TRACE_ISR(x) tim3_isr(x)

#define gpio_set_val(port, pin, val) do {	\
	if(val)					\
		gpio_set((port), (pin));	\
	else					\
		gpio_clear((port), (pin));	\
} while(0)

#define SET_RUN_STATE(state)	{running_status = (state);}
#define SET_IDLE_STATE(state)	{gpio_set_val(LED_PORT, LED_IDLE_RUN, state);}
#define SET_ERROR_STATE(state)	{gpio_set_val(LED_PORT, LED_ERROR, state);}

static inline int platform_hwversion(void)
{
	return 0;
}

void put_status(char *m);
int _write(int file, char *ptr, int len);
int _read(int file, char *ptr, int len);
void platform_request_boot2(void);
void tsirq_pin_init(void);
/*
 * Use newlib provided integer only stdio functions
 */

/* sscanf */
#ifdef sscanf
#undef sscanf
#define sscanf siscanf
#else
#define sscanf siscanf
#endif
/* sprintf */
#ifdef sprintf
#undef sprintf
#define sprintf siprintf
#else
#define sprintf siprintf
#endif
/* vasprintf */
#ifdef vasprintf
#undef vasprintf
#define vasprintf vasiprintf
#else
#define vasprintf vasiprintf
#endif
/* snprintf */
#ifdef snprintf
#undef snprintf
#define snprintf sniprintf
#else
#define snprintf sniprintf
#endif

#endif
