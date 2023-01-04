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
#ifndef STLINKV3
#include "usb_descriptors.h"
#endif
#ifndef F103
#include "usb_descriptors2.h"
#include "usb_descriptors3.h"
#endif
#ifdef STLINKV3
#include "stlinkv3.h"
#endif
#ifdef F103
//#include "usb_descriptors3.h"
//#include "usb_descriptors4.h"
#include "usb_descriptors2.h"
#else
#include "usb_descriptors3.h"
#endif
#include "usb_serial.h"

#include "usb_dfu_stub.h"
#include "usb_i2c.h"
#include "usb_gpio.h"
#include "usb_adc.h"
#include "usb_adc2.h"
#ifdef F103
#include "usb-swim.h"

#endif
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

void blackmagic_usb_init(int altusb)
{
	read_serial_number();

    if (altusb == 0) {
    read_serial_number();

	usbdev = usbd_init(&stm32f207_usb_driver, &dev_desc, &config, usb_strings, sizeof(usb_strings) / sizeof(char *),
		usbd_control_buffer, sizeof(usbd_control_buffer));

//    OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS;

	usbd_register_set_config_callback(usbdev, usb_serial_set_config);

//	usbd_register_set_config_callback(usbdev, usb_set_config);
//    usbd_register_set_config_callback(usbdev, gpio_set_config);
//    usbd_register_set_config_callback(usbdev, usbadc_set_config);

    usbd_register_set_config_callback(usbdev, dfu_set_config);


	delay_setup();
	nvic_set_priority(USB_IRQ, IRQ_PRI_USB);
	nvic_enable_irq(USB_IRQ);
	} if (altusb == 1) { 
	read_serial_number();

	usbdev = usbd_init(&stm32f207_usb_driver, &dev2, &config2,
//    usbd_dev = usbd_init(&stm32f107_usb_driver, &dev, &config,
			usb2_strings, sizeof(usb2_strings)/sizeof(char *),
			usbd_control_buffer, sizeof(usbd_control_buffer));

//    OTG_FS_GUSBCFG |= OTG_GUSBCFG_FDMOD | OTG_GUSBCFG_TRDT_MASK;
//	OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS | OTG_GCCFG_PWRDWN;
//	OTG_FS_GCCFG &= ~(OTG_GCCFG_VBUSBSEN | OTG_GCCFG_VBUSASEN);
//	usbd_register_set_config_callback(usbdev, usb_set_config);
////	usbd_register_set_config_callback(usbdev, usb_set_config);
	usbd_register_set_config_callback(usbdev, gpio_set_config);
	usbd_register_set_config_callback(usbdev, usbadc_set_config);
    usbd_register_set_config_callback(usbdev, dfu_set_config2);
	delay_setup();
	nvic_set_priority(USB_IRQ, IRQ_PRI_USB);
	nvic_enable_irq(USB_IRQ);



    } if (altusb == 2) { 

	usbdev = usbd_init(&stm32f207_usb_driver, &dev_desc3, &config3,
//    usbd_dev = usbd_init(&stm32f107_usb_driver, &dev, &config,
			usb_strings3, sizeof(usb_strings3)/sizeof(char *),
			usbd_control_buffer, sizeof(usbd_control_buffer));

//    OTG_FS_GUSBCFG |= OTG_GUSBCFG_FDMOD | OTG_GUSBCFG_TRDT_MASK;
//	OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS | OTG_GCCFG_PWRDWN;
//	OTG_FS_GCCFG &= ~(OTG_GCCFG_VBUSBSEN | OTG_GCCFG_VBUSASEN);
//	usbd_register_set_config_callback(usbdev, usb_set_config);
    usbd_register_set_config_callback(usbdev, usb_serial2_set_config);
//    usbd_register_set_config_callback(usbdev, slcan_set_config);
    usbd_register_set_config_callback(usbdev, dfu_set_config);
	delay_setup();
	nvic_set_priority(USB_IRQ, IRQ_PRI_USB);
	nvic_enable_irq(USB_IRQ);

    }
}

uint16_t usb_get_config(void)
{
	return usb_config;
}

void USB_ISR(void)
{
	usbd_poll(usbdev);
}
