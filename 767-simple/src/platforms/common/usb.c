/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2022 1BitSquared <info@1bitsquared.com>
 * Written by Rachel Mant <git@dragonmux.network>
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

#include <libopencm3/cm3/nvic.h>

#include "general.h"
#include "usb.h"

#include "usb_descriptors.h"

#include "usb_serial.h"

#include "usb_dfu_stub.h"

#include "usb_gpio.h"
#include "usb_adc.h"
#include "usb_adc2.h"

#include "serialno.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/usb/dwc/otg_hs.h>
#include <libopencm3/usb/dwc/otg_common.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/cm3/scb.h>
#include "stm32_slcan.h"
//#include "usb_descriptors3.h"
#include "usb_serial2.h"

usbd_device *usbdev = NULL;
uint16_t usb_config;
void delay_setup(void);

/* We need a special large control buffer for this device: */
static uint8_t usbd_control_buffer[256];

void delay_setup(void)
{
	/* set up a microsecond free running timer for ... things... */
	rcc_periph_clock_enable(RCC_TIM6);
	/* microsecond counter */
	timer_set_prescaler(TIM6, rcc_apb1_frequency / 1000000 - 1);
	timer_set_period(TIM6, 0xffff);
	timer_one_shot_mode(TIM6);
}

void blackmagic_usb_init(void)
{
	read_serial_number();


	usbdev = usbd_init(&stm32f207_usb_driver, &dev_desc, &config, usb_strings, sizeof(usb_strings) / sizeof(char *),
		usbd_control_buffer, sizeof(usbd_control_buffer));

//    OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS;

	usbd_register_set_config_callback(usbdev, usb_serial_set_config);

//	usbd_register_set_config_callback(usbdev, usb_set_config);
//    usbd_register_set_config_callback(usbdev, gpio_set_config);
    usbd_register_set_config_callback(usbdev, usbadc_set_config);

    usbd_register_set_config_callback(usbdev, dfu_set_config);


//	delay_setup();
	nvic_set_priority(USB_IRQ, IRQ_PRI_USB);
	nvic_enable_irq(USB_IRQ);

}

uint16_t usb_get_config(void)
{
	return usb_config;
}

void USB_ISR(void)
{
	usbd_poll(usbdev);
}
