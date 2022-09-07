/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2012  Black Sphere Technologies Ltd.
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
#ifndef __USBUART_H
#define __USBUART_H

#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <librfn/fibre.h>



#define USBUSART USART1
#define USBUSART_CR1 USART1_CR1
#define USBUSART_DR USART1_DR
#define USBUSART_IRQ NVIC_USART1_IRQ
#define USBUSART_CLK RCC_USART1
#define USBUSART_PORT GPIOA
#define USBUSART_TX_PIN GPIO9
#define USBUSART_RX_PIN GPIO10
#define USBUSART2_PORT 
#define USBUSART2_TX_PIN
#define USBUSART2_RX_PIN
#define USBUSART_ISR(x) usart1_isr(x)
#define USBUSART_DMA_BUS DMA2
#define USBUSART_DMA_CLK RCC_DMA2
#define USBUSART_DMA_TX_CHAN DMA_STREAM7
#define USBUSART_DMA_TX_IRQ NVIC_DMA2_STREAM7_IRQ
#define USBUSART_DMA_TX_ISR(x) dma2_stream7_isr(x)
#define USBUSART_DMA_RX_CHAN DMA_STREAM5
#define USBUSART_DMA_RX_IRQ NVIC_DMA2_STREAM5_IRQ
#define USBUSART_DMA_RX_ISR(x) dma2_stream5_isr(x)
/* For STM32F4 DMA trigger source must be specified */
#define USBUSART_DMA_TRG DMA_SxCR_CHSEL_4

#define USB_IRQ         NVIC_OTG_FS_IRQ
#define USB_ISR(x)      otg_fs_isr(x)
/* Interrupt priorities.  Low numbers are high priority.
 * TIM3 is used for traceswo capture and must be highest priority.
 */
#define IRQ_PRI_USB		(1 << 4)
#define IRQ_PRI_USBUSART	(2 << 4)
#define IRQ_PRI_USBUSART_DMA 	(2 << 4)
#define IRQ_PRI_TRACE		(0 << 4)

void usbuart_init(void);
//static fibre_t fibre = FIBRE_VAR_INIT(usbuart_fibre);
int cdcacm_get_config(void);
void usbuart_send_stdout(const uint8_t *data, uint32_t len);
void usbuart_set_line_coding(struct usb_cdc_line_coding *coding);
extern void usbuart_usb_out_cb(usbd_device *dev, uint8_t ep);
extern void usbuart_usb_in_cb(usbd_device *dev, uint8_t ep);

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

