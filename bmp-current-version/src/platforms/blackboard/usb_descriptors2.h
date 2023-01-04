#ifndef USB_DESCRIPTORS2_H
#define USB_DESCRIPTOR2S_H

#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/usb/dfu.h>

#include "usb.h"
#include "serialno.h"
#include "version.h"
#include "usb_adc.h"

#define BOARD_IDENT "Black Magic Probe " PLATFORM_IDENT FIRMWARE_VERSION


static const struct usb_device_descriptor dev2 = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0100,
	.bDeviceClass = 0xEF,
	.bDeviceSubClass = 2,
	.bDeviceProtocol = 1,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0403,
	.idProduct = 0xc631,
	.bcdDevice = 0x0100,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 5,
	.bNumConfigurations = 1,
};

const struct usb_endpoint_descriptor i2i2c_endpoint = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x81,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 4,
	.bInterval = 0x20,
};

const struct usb_interface_descriptor i2i2c_iface2 = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = 0,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 2,

	.endpoint = &i2i2c_endpoint,
};

const struct usb_endpoint_descriptor gpio_endpoint2 = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x82,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 4,
	.bInterval = 9,
};

const struct usb_interface_descriptor gpio_iface2 = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = 0,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 1,

	.endpoint = &gpio_endpoint2,
};

static const struct usb_endpoint_descriptor endp_bulk2[] = {
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x03,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = BULK_EP_MAXPACKET,
		.bInterval = 1,
	},
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = 0x83,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = BULK_EP_MAXPACKET,
		.bInterval = 1,
	},
};

static const struct usb_interface_descriptor iface_sourcesink2[] = {
	{
		.bLength = USB_DT_INTERFACE_SIZE,
		.bDescriptorType = USB_DT_INTERFACE,
		.bInterfaceNumber = 2,
		.bAlternateSetting = 0,
		.bNumEndpoints = 2,
		.bInterfaceClass = USB_CLASS_VENDOR,
		.iInterface = 3,
		.endpoint = endp_bulk2,
	}
};

const struct usb_dfu_descriptor dfu2_function = {
	.bLength = sizeof(struct usb_dfu_descriptor),
	.bDescriptorType = DFU_FUNCTIONAL,
	.bmAttributes = USB_DFU_CAN_DOWNLOAD | USB_DFU_WILL_DETACH,
	.wDetachTimeout = 255,
	.wTransferSize = 1024,
	.bcdDFUVersion = 0x011A,
};

const struct usb_interface_descriptor dfu2_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 3,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = 0xFE,
	.bInterfaceSubClass = 1,
	.bInterfaceProtocol = 1,
	.iInterface = 4,

	.extra = &dfu2_function,
	.extralen = sizeof(dfu2_function),
};

static const struct usb_iface_assoc_descriptor dfu2_assoc = {
	.bLength = USB_DT_INTERFACE_ASSOCIATION_SIZE,
	.bDescriptorType = USB_DT_INTERFACE_ASSOCIATION,
	.bFirstInterface = 3, //2
	.bInterfaceCount = 1,
	.bFunctionClass = 0xFE,
	.bFunctionSubClass = 1,
	.bFunctionProtocol = 1,
	.iFunction = 4,
};


const struct usb_interface ifacespb[] = {{
	.num_altsetting = 1,
	.altsetting = &i2i2c_iface2,
	}, {
	.num_altsetting = 1,
	.altsetting = &gpio_iface2,
	}, {
	.num_altsetting = 1,
	.altsetting = iface_sourcesink2,
	}, {
	.num_altsetting = 1,
	.iface_assoc = &dfu2_assoc,
	.altsetting = &dfu2_iface,
}};

static const struct usb_config_descriptor config2 = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0, /* ?automatically calculated? */
	.bNumInterfaces = 4,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80, /* bus powered */
	.bMaxPower = 0x32,

	.interface = ifacespb,
};

static const char *const usb2_strings[] = {
	"redfelineninja.org.uk",
	"i2c-stm32f4-usb",
	"testbulk",
	"@Internal Flash  /0x08000000/04*016Kg,01*064Kg,07*128Kg",
	"205D338C3156",
};

#endif
