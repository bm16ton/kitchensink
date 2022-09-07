/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/usb/usbd.h>
#include "usart_irq.h"
#include "usbuart.h"
#include <string.h>

#define FIFO_SIZE 128

void usart_clock_setup(void)
{

	/* Enable clocks for GPIO port A (for LED GPIO_USART1_TX) and USART1. */
	rcc_periph_clock_enable(RCC_USART1);
}

void usart_setup(void)
{
	/* Enable the USART1 interrupt. */
	nvic_enable_irq(NVIC_USART1_IRQ);

	/* Setup GPIO pin GPIO_USART1_RE_TX on GPIO port A for transmit. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

	/* Setup GPIO pin GPIO_USART1_RE_RX on GPIO port A for receive. */
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
		      GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

	/* Setup UART parameters. */
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, USART_MODE_TX_RX);

	/* Enable USART1 Receive interrupt. */
	USART_CR1(USART1) |= USART_CR1_RXNEIE;

	/* Finally enable the USART. */
	usart_enable(USART1);
}

static void uart_led_gpio_setup(void)
{
	gpio_set(GPIOA, GPIO5);

	/* Setup GPIO5 (in GPIO port A) for LED use. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_PUSHPULL, GPIO5);
}


static usbd_device *usbd_dev;
/*
void usart1_isr(void)
{
	static uint8_t data = 'A';
//    short unsigned int data;
    
	// Check if we were called because of RXNE. 
	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {

		// Indicate that we got data. 
//		gpio_toggle(GPIOA, GPIO5);

		// Retrieve the data from the peripheral. 
		data = usart_recv(USART1);
//        char buf[64];
//        memcpy(buf, &data, sizeof(data));
//	    int len = sizeof(data);


	
		// Enable transmit interrupt so it sends back the data. 
		USART_CR1(USART1) |= USART_CR1_TXEIE;
	}
//    usart_isr(USART1, TIM2, &buf_rx1_out, &buf_rx1_in, d);
	// Check if we were called because of TXE. 
	if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
	    ((USART_SR(USART1) & USART_SR_TXE) != 0)) {

		// Indicate that we are sending out data. 
		// gpio_toggle(GPIOA, GPIO5);

		// Put data into the transmit register. 
		usart_send(USART1, data);

		// Disable the TXE interrupt as we don't need it anymore. 
		USART_CR1(USART1) &= ~USART_CR1_TXEIE;
	}
}

static void usart_isr(int USBUSART, int USBUSART_TIM, uint8_t *buf_rx_out, uint8_t *buf_rx_in, uint8_t *buf_rx)
{
	uint32_t err = USART_SR(USBUSART);
	char c = usart_recv(USBUSART);
	if (err & (USART_SR_ORE | USART_SR_FE))
		return;

	

	// If the next increment of rx_in would put it at the same point
	// as rx_out, the FIFO is considered full.
	//
	if (((*buf_rx_in + 1) % FIFO_SIZE) != *buf_rx_out)
	{
		// insert into FIFO  
		buf_rx[(*buf_rx_in)++] = c;

		// wrap out pointer  
		if (*buf_rx_in >= FIFO_SIZE)
		{
			*buf_rx_in = 0;
		}

		// enable deferred processing if we put data in the FIFO 
		timer_enable_irq(USBUSART_TIM, TIM_DIER_UIE);
	}
}

void usart1_isr(void)
{
    usart_isr(USART1, TIM2, &buf_rx1_out, &buf_rx1_in, buf_rx1);
}
*/
int usart_main(void)
{
//	SCB_VTOR = (uint32_t) 0x08005000;

	usart_clock_setup();
	uart_led_gpio_setup();
	usart_setup();

	/* Wait forever and do nothing. 
	while (1)
		__asm__("nop");

	return 0;
	*/
	return 0;
}
