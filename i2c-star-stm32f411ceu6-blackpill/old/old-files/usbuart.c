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

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scs.h>

#include "general.h"
#include "cdcacm.h"
#include "usbuart.h"

#define USBUART_TIMER_FREQ_HZ 1000000U /* 1us per tick */
#define USBUART_RUN_FREQ_HZ 5000U /* 200us (or 100 characters at 2Mbps) */

//#define CDCACM_PACKET_SIZE 64U

#define FIFO_SIZE 128

/* RX Fifo buffer */
static uint8_t buf_rx1[128];

/* Fifo in pointer, writes assumed to be atomic, should be only incremented within RX ISR */
static uint8_t buf_rx1_in;

/* Fifo out pointer, writes assumed to be atomic, should be only incremented outside RX ISR */
static uint8_t buf_rx1_out;

static void usbuart_run(int USBUSART_TIM, uint8_t *buf_rx_out, uint8_t *buf_rx_in, uint8_t *buf_rx);

void usbuart_init(void)
{

	/* Setup timer for running deferred FIFO processing */
	rcc_periph_clock_enable(RCC_TIM2);

//	timer_reset(TIM2);

	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);


	timer_set_prescaler(TIM2,
			rcc_apb2_frequency / USBUART_TIMER_FREQ_HZ * 2 - 1);


	timer_set_period(TIM2,
			USBUART_TIMER_FREQ_HZ / USBUART_RUN_FREQ_HZ - 1);


	/* Setup update interrupt in NVIC */
	nvic_set_priority(NVIC_TIM2_IRQ, IRQ_PRI_USBUSART_TIM);

	nvic_enable_irq(NVIC_TIM2_IRQ);


	/* turn the timer on */
	timer_enable_counter(TIM2);

}

/*
 * Runs deferred processing for usb uart rx, draining RX FIFO by sending
 * characters to host PC via CDCACM.  Allowed to read from FIFO in pointer,
 * but not write to it. Allowed to write to FIFO out pointer.
 */

 
static void usbuart_run(int USBUSART_TIM, uint8_t *buf_rx_out, uint8_t *buf_rx_in, uint8_t *buf_rx)
{
	/* forcibly empty fifo if no USB endpoint */
//	if (cdcacm_get_config() != 1)
//	{
//		*buf_rx_out = *buf_rx_in;
//	}
static usbd_device *usbd_dev;
	/* if fifo empty, nothing further to do */
	if (*buf_rx_in == *buf_rx_out) {
		/* turn off LED, disable IRQ */
		timer_disable_irq(USBUSART_TIM, TIM_DIER_UIE);
		
	}
	else
	{
		uint8_t packet_buf[CDCACM_PACKET_SIZE];
		uint8_t packet_size = 0;
		uint8_t buf_out = *buf_rx_out;

		/* copy from uart FIFO into local usb packet buffer */
		while (*buf_rx_in != buf_out && packet_size < CDCACM_PACKET_SIZE)
		{
			packet_buf[packet_size++] = buf_rx[buf_out++];

			/* wrap out pointer */
			if (buf_out >= FIFO_SIZE)
			{
			usbd_ep_write_packet(usbd_dev, 0x87, packet_buf, packet_size);
				buf_out = 0;
			}

		}
        usbd_ep_write_packet(usbd_dev, 0x87, packet_buf, packet_size);
		/* advance fifo out pointer by amount written */
		*buf_rx_out += usbd_ep_write_packet(usbd_dev,
				87, packet_buf, packet_size);

		*buf_rx_out %= FIFO_SIZE;
	}
}

void usbuart_set_line_coding(struct usb_cdc_line_coding *coding, int USBUSART)
{
	usart_set_baudrate(USBUSART, coding->dwDTERate);

	if (coding->bParityType)
		usart_set_databits(USBUSART, coding->bDataBits + 1);
	else
		usart_set_databits(USBUSART, coding->bDataBits);

	switch(coding->bCharFormat) {
	case 0:
		usart_set_stopbits(USBUSART, USART_STOPBITS_1);
		break;
	case 1:
		usart_set_stopbits(USBUSART, USART_STOPBITS_1_5);
		break;
	case 2:
		usart_set_stopbits(USBUSART, USART_STOPBITS_2);
		break;
	}

	switch(coding->bParityType) {
	case 0:
		usart_set_parity(USBUSART, USART_PARITY_NONE);
		break;
	case 1:
		usart_set_parity(USBUSART, USART_PARITY_ODD);
		break;
	case 2:
		usart_set_parity(USBUSART, USART_PARITY_EVEN);
		break;
	}
}






void usbuart_usb_in_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void) usbd_dev;
	(void) ep;
	usbuart_run(TIM2, &buf_rx1_out, &buf_rx1_in, buf_rx1);
}

/*
 * Read a character from the UART RX and stuff it in a software FIFO.
 * Allowed to read from FIFO out pointer, but not write to it.
 * Allowed to write to FIFO in pointer.
 */
// USBUSART_ISR
void usart_isr(int USBUSART, int USBUSART_TIM, uint8_t *buf_rx_out, uint8_t *buf_rx_in, uint8_t *buf_rx)
{
	uint32_t err = USART_SR(USBUSART);
	char c = usart_recv(USBUSART);
	if (err & (USART_SR_ORE | USART_SR_FE))
		return;

	/* Turn on LED */
//	gpio_set(LED_PORT_UART, LED_UART);

	/* If the next increment of rx_in would put it at the same point
	* as rx_out, the FIFO is considered full.
	*/
	if (((*buf_rx_in + 1) % FIFO_SIZE) != *buf_rx_out)
	{
		/* insert into FIFO */
		buf_rx[(*buf_rx_in)++] = c;

		/* wrap out pointer */
		if (*buf_rx_in >= FIFO_SIZE)
		{
			*buf_rx_in = 0;
		}

		/* enable deferred processing if we put data in the FIFO */
		timer_enable_irq(USBUSART_TIM, TIM_DIER_UIE);
	}
}

void usart1_isr(void)
{
    usart_isr(USART1, TIM2, &buf_rx1_out, &buf_rx1_in, buf_rx1);
}



// USBUSART_TIM_ISR
void tim2_isr(void)
{
	/* need to clear timer update event */
	timer_clear_flag(TIM2, TIM_SR_UIF);

	/* process FIFO */
	usbuart_run(TIM2, &buf_rx1_out, &buf_rx1_in, buf_rx1);
}




