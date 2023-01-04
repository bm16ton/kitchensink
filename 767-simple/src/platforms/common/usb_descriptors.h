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

#ifndef USB_DESCRIPTORS_H
#define USB_DESCRIPTORS_H

#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/usb/dfu.h>

#include "usb.h"
#include "serialno.h"
#include "version.h"
#include "usb_adc.h"

#define BOARD_IDENT "Black Magic Probe " PLATFORM_IDENT FIRMWARE_VERSION

/* Top-level device descriptor */
static const struct usb_device_descriptor dev_desc = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0100,
	.bDeviceClass = 0xEF, /* Miscellaneous Device */ // EF
	.bDeviceSubClass = 2, /* Common Class */        //  2
	.bDeviceProtocol = 1, /* Interface Association */ //1
	.bMaxPacketSize0 = 64, /*Fixed for icdi*/
	.idVendor = 0x1D50,
	.idProduct = 0x6018,
	.bcdDevice = 0x0100,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

/* GDB interface descriptors */

/* This notification endpoint isn't implemented. According to CDC spec its
 * optional, but its absence causes a NULL pointer dereference in Linux cdc_acm
 * driver. */
static const struct usb_endpoint_descriptor gdb_comm_endp = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = (CDCACM_GDB_ENDPOINT + 1) | USB_REQ_TYPE_IN,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 64,
	.bInterval = 9,
};

static const struct usb_endpoint_descriptor gdb_data_endp[] = {
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = CDCACM_GDB_ENDPOINT, //1
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = CDCACM_PACKET_SIZE,
		.bInterval = 1,
	},
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = CDCACM_GDB_ENDPOINT | USB_REQ_TYPE_IN,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = CDCACM_PACKET_SIZE,
		.bInterval = 1,
	},
};

static const struct {
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) gdb_cdcacm_functional_descriptors = {
	.header =
		{
			.bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
			.bDescriptorType = CS_INTERFACE,
			.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
			.bcdCDC = 0x0110,
		},
	.call_mgmt =
		{
			.bFunctionLength = sizeof(struct usb_cdc_call_management_descriptor),
			.bDescriptorType = CS_INTERFACE,
			.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
			.bmCapabilities = 0,
			.bDataInterface = GDB_IF_NO + 1,
		},
	.acm =
		{
			.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
			.bDescriptorType = CS_INTERFACE,
			.bDescriptorSubtype = USB_CDC_TYPE_ACM,
			.bmCapabilities = 2, // SET_LINE_CODING supported 
		},
	.cdc_union =
		{
			.bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
			.bDescriptorType = CS_INTERFACE,
			.bDescriptorSubtype = USB_CDC_TYPE_UNION,
			.bControlInterface = GDB_IF_NO,
			.bSubordinateInterface0 = GDB_IF_NO + 1,
		},
};

static const struct usb_interface_descriptor gdb_comm_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_CDC,
	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol = USB_CDC_PROTOCOL_NONE,
	.iInterface = 4,

	.endpoint = &gdb_comm_endp,

	.extra = &gdb_cdcacm_functional_descriptors,
	.extralen = sizeof(gdb_cdcacm_functional_descriptors),
};

static const struct usb_interface_descriptor gdb_data_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = GDB_IF_NO + 1, //0
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_DATA,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = gdb_data_endp,
};

static const struct usb_iface_assoc_descriptor gdb_assoc = {
	.bLength = USB_DT_INTERFACE_ASSOCIATION_SIZE,
	.bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,
	.bFirstInterface = GDB_IF_NO,
	.bInterfaceCount = 2,
	.bFunctionClass = USB_CLASS_CDC,
	.bFunctionSubClass = USB_CDC_SUBCLASS_ACM,
	.bFunctionProtocol = USB_CDC_PROTOCOL_NONE,
	.iFunction = 4,
};

/* Phyiscal/debug UART interface */

static const struct usb_endpoint_descriptor uart_comm_endp = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = (CDCACM_UART_ENDPOINT + 1) | USB_REQ_TYPE_IN,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = CDCACM_PACKET_SIZE,
	.bInterval = 11,
};

static const struct usb_endpoint_descriptor uart_data_endp[] = {
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = CDCACM_UART_ENDPOINT, //2
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = CDCACM_PACKET_SIZE,
		.bInterval = 1,
	},
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = CDCACM_UART_ENDPOINT | USB_REQ_TYPE_IN,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = CDCACM_PACKET_SIZE,
		.bInterval = 1,
	},
};

static const struct {
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) uart_cdcacm_functional_descriptors = {
	.header =
		{
			.bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
			.bDescriptorType = CS_INTERFACE,
			.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
			.bcdCDC = 0x0110,
		},
	.call_mgmt =
		{
			.bFunctionLength = sizeof(struct usb_cdc_call_management_descriptor),
			.bDescriptorType = CS_INTERFACE,
			.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
			.bmCapabilities = 0,
			.bDataInterface = UART_IF_NO + 1,
		},
	.acm =
		{
			.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
			.bDescriptorType = CS_INTERFACE,
			.bDescriptorSubtype = USB_CDC_TYPE_ACM,
			.bmCapabilities = 2, // SET_LINE_CODING supported
		},
	.cdc_union =
		{
			.bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
			.bDescriptorType = CS_INTERFACE,
			.bDescriptorSubtype = USB_CDC_TYPE_UNION,
			.bControlInterface = UART_IF_NO,
			.bSubordinateInterface0 = UART_IF_NO + 1,
		},
};

static const struct usb_interface_descriptor uart_comm_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = UART_IF_NO,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_CDC,
	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol = USB_CDC_PROTOCOL_NONE,
	.iInterface = 5,

	.endpoint = &uart_comm_endp,

	.extra = &uart_cdcacm_functional_descriptors,
	.extralen = sizeof(uart_cdcacm_functional_descriptors),
};

static const struct usb_interface_descriptor uart_data_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = UART_IF_NO + 1,   //1
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_DATA,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = uart_data_endp,
};

static const struct usb_iface_assoc_descriptor uart_assoc = {
	.bLength = USB_DT_INTERFACE_ASSOCIATION_SIZE,
	.bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,
	.bFirstInterface = UART_IF_NO, //1
	.bInterfaceCount = 2,
	.bFunctionClass = USB_CLASS_CDC,
	.bFunctionSubClass = USB_CDC_SUBCLASS_ACM,
	.bFunctionProtocol = USB_CDC_PROTOCOL_NONE,
	.iFunction = 5,
};

/* DFU interface */


/*
const struct usb_interface_descriptor i2c_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 2,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = 0,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 7,

};


static const struct usb_iface_assoc_descriptor i2c_assoc = {
	.bLength = USB_DT_INTERFACE_ASSOCIATION_SIZE,
	.bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,
	.bFirstInterface = 3,
	.bInterfaceCount = 1,
	.bFunctionClass = 0xff,
	.bFunctionSubClass = 0,
	.bFunctionProtocol = 0,
	.iFunction = 7,
};

const struct usb_endpoint_descriptor gpio_endpoint = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x83,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 16,
	.bInterval = 0x20,
};

const struct usb_interface_descriptor gpio_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 3,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = 0,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 8,

	.endpoint = &gpio_endpoint,
};
*/
const struct usb_dfu_descriptor dfu_function = {
	.bLength = sizeof(struct usb_dfu_descriptor),
	.bDescriptorType = DFU_FUNCTIONAL,
	.bmAttributes = USB_DFU_CAN_DOWNLOAD | USB_DFU_WILL_DETACH,
	.wDetachTimeout = 255,
	.wTransferSize = 1024,
	.bcdDFUVersion = 0x011A,
};

const struct usb_interface_descriptor dfu_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = DFU_IF_NO,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = 0xFE,
	.bInterfaceSubClass = 1,
	.bInterfaceProtocol = 1,
	.iInterface = 6,

	.extra = &dfu_function,
	.extralen = sizeof(dfu_function),
};

static const struct usb_iface_assoc_descriptor dfu_assoc = {
	.bLength = USB_DT_INTERFACE_ASSOCIATION_SIZE,
	.bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,
	.bFirstInterface = DFU_IF_NO, //2
	.bInterfaceCount = 1,
	.bFunctionClass = 0xFE,
	.bFunctionSubClass = 1,
	.bFunctionProtocol = 1,
	.iFunction = 6,
};
/*
static const struct usb_iface_assoc_descriptor gpio_assoc = {
	.bLength = USB_DT_INTERFACE_ASSOCIATION_SIZE,
	.bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,
	.bFirstInterface = 4,
	.bInterfaceCount = 1,
	.bFunctionClass = 0,
	.bFunctionSubClass = 0,
	.bFunctionProtocol = 0,
	.iFunction = 8,
};
*/

static const struct usb_endpoint_descriptor endp_bulk[] = {
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x04,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = CDCACM_PACKET_SIZE,
		.bInterval = 1,
	},
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x84,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = CDCACM_PACKET_SIZE,
		.bInterval = 1,
	},
};

static const struct usb_interface_descriptor iface_sourcesink[] = {
	{
		.bLength = USB_DT_INTERFACE_SIZE,
		.bDescriptorType = USB_DT_INTERFACE,
		.bInterfaceNumber = 5,
		.bAlternateSetting = 0,
		.bNumEndpoints = 2,
		.bInterfaceClass = USB_CLASS_VENDOR,
		.iInterface = 9,
		.endpoint = endp_bulk,
	}
};

/* Interface and configuration descriptors */

static const struct usb_interface ifaces[] = {
	{
		.num_altsetting = 1,
		.iface_assoc = &gdb_assoc,
		.altsetting = &gdb_comm_iface,
	},
	{
		.num_altsetting = 1,
		.altsetting = &gdb_data_iface,
	},
	{
		.num_altsetting = 1,
		.iface_assoc = &uart_assoc,
		.altsetting = &uart_comm_iface,
	},
	{
		.num_altsetting = 1,
		.altsetting = &uart_data_iface,
	},
	{
		.num_altsetting = 1,
		.iface_assoc = &dfu_assoc,
		.altsetting = &dfu_iface,
	},
	{
		.num_altsetting = 1,
	    .altsetting = iface_sourcesink,
	}, 
};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 6,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char *const usb_strings[] = {
	"Black Magic Debug",
	BOARD_IDENT,
	"367E36723135",
	"Black Magic GDB Server",
	"Black Magic UART Port",
	"@Internal Flash  /0x08000000/04*016Kg,01*064Kg,07*128Kg",
    "i2c-stm32f4-usb",
    "usb-gpio",
    "usb-adc",
};



#endif /*USB_DESCRIPTORS_H*/
