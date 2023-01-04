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

#ifndef USB_H
#define USB_H

#include <stdint.h>
#include <libopencm3/usb/usbd.h>

extern usbd_device *usbdev;
extern uint16_t usb_config;

#if defined(USB_HS)
# define CDCACM_PACKET_SIZE    512
#else
# define CDCACM_PACKET_SIZE     64
#endif


#ifndef STLINKV3
#define CDCACM_GDB_ENDPOINT  1
#define CDCACM_SLCAN_ENDPOINT 1
#define CDCACM_UART_ENDPOINT 3
#define CDCACM3_UART_ENDPOINT 3
#define GDB_IF_NO  0
#define UART_IF_NO 2
#define UART3_IF_NO 2
#define DFU_IF_NO  4
#ifdef PLATFORM_HAS_TRACESWO
#define TRACE_IF_NO      5

#define TOTAL_INTERFACES 5
#else
#define TOTAL_INTERFACES 5
#endif
#endif
#define CDCACM_SLCAN_ENDPOINT 1
#define CDCACM_SLCAN_INTERFACE 0
#define SLCAN_IF_NO  0
void blackmagic_usb_init(int altusb);

/* Returns current usb configuration, or 0 if not configured. */
uint16_t usb_get_config(void);

#endif /*USB_H*/
