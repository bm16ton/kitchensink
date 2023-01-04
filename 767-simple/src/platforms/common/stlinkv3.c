
#include "general.h"
#include <gdb_if.h>
#include "cdcacm.h"
#if defined(PLATFORM_HAS_SLCAN)
# undef PLATFORM_HAS_TRACESWO
# define PLATFORM_HAS_TRACESWO
//# include "traceswo.h"
#endif
#if defined(PLATFORM_HAS_TRACESWO)
#	include "traceswo.h"
#endif
#include "usbuart.h"
#include "serialno.h"
#include "version.h"
#include "stlinkv3.h"
#include "usb_serial.h"
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/usb/dfu.h>


usbd_device * usbdev;

static int configured;
static int cdcacm_gdb_dtr = 1;

static void cdcacm_set_modem_state(usbd_device *dev, int iface, bool dsr, bool dcd);

static void dfu_detach_complete(usbd_device *dev, struct usb_setup_data *req)
{
	(void)dev;
	(void)req;

	platform_request_boot();

	/* Reset core to enter bootloader */
#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
	scb_reset_core();
#endif
}

static enum usbd_request_return_codes  cdcacm_control_request(usbd_device *dev,
		struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
		void (**complete)(usbd_device *dev, struct usb_setup_data *req))
{
	(void)dev;
	(void)complete;
	(void)buf;
	(void)len;

	switch(req->bRequest) {
	case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
		cdcacm_set_modem_state(dev, req->wIndex, true, true);
		/* Ignore if not for GDB interface */
		if(req->wIndex != GDB_IF_NO)
			return USBD_REQ_HANDLED;

		cdcacm_gdb_dtr = req->wValue & 1;

		return USBD_REQ_HANDLED;
	case USB_CDC_REQ_SET_LINE_CODING:
		if(*len < sizeof(struct usb_cdc_line_coding))
			return USBD_REQ_NOTSUPP;

		switch(req->wIndex) {
		case UART_IF_NO:
			usbuart_set_line_coding((struct usb_cdc_line_coding*)*buf);
			return USBD_REQ_HANDLED;
		case GDB_IF_NO:
#if defined(PLATFORM_HAS_SLCAN)
		case SLCAN_IF_NO:
#endif
			return USBD_REQ_HANDLED; /* Ignore on GDB Port */
		default:
			return USBD_REQ_NOTSUPP;
		}
	case DFU_GETSTATUS:
		if(req->wIndex == DFU_IF_NO) {
			(*buf)[0] = DFU_STATUS_OK;
			(*buf)[1] = 0;
			(*buf)[2] = 0;
			(*buf)[3] = 0;
			(*buf)[4] = STATE_APP_IDLE;
			(*buf)[5] = 0;	/* iString not used here */
			*len = 6;

			return USBD_REQ_HANDLED;
		}
		return USBD_REQ_NOTSUPP;
	case DFU_DETACH:
		if(req->wIndex == DFU_IF_NO) {
			*complete = dfu_detach_complete;
			return USBD_REQ_HANDLED;
		}
		return USBD_REQ_NOTSUPP;
	}
	return USBD_REQ_NOTSUPP;
}

int cdcacm_get_config(void)
{
	return configured;
}

int cdcacm_get_dtr(void)
{
	return cdcacm_gdb_dtr;
}

static void cdcacm_set_modem_state(usbd_device *dev, int iface, bool dsr, bool dcd)
{
	char buf[10];
	struct usb_cdc_notification *notif = (void*)buf;
	/* We echo signals back to host as notification */
	notif->bmRequestType = 0xA1;
	notif->bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
	notif->wValue = 0;
	notif->wIndex = iface;
	notif->wLength = 2;
	buf[8] = (dsr ? 2 : 0) | (dcd ? 1 : 0);
	buf[9] = 0;
	uint8_t cdcacm_cltr_ep = 0x82 + iface;
#if defined(PLATFORM_HAS_SLCAN)
	if (iface == SLCAN_IF_NO)
		cdcacm_cltr_ep = (CDCACM_SLCAN_ENDPOINT2 + 1) | USB_REQ_TYPE_IN;
#endif
	usbd_ep_write_packet(dev, cdcacm_cltr_ep, buf, 10);
}

#if defined(PLATFORM_HAS_SLCAN)
extern void slcan_usb_out_cb(usbd_device *dev, uint8_t ep);
#endif
extern void gdb_usb_out_cb(usbd_device *dev, uint8_t ep);

void cdcacm_set_config(usbd_device *dev, uint16_t wValue)
{
	configured = wValue;

	/* GDB interface */
#if defined(STM32F4) || defined(LM4F) || defined(STM32F7)
	usbd_ep_setup(dev, CDCACM_GDB_ENDPOINT2, USB_ENDPOINT_ATTR_BULK,
	              CDCACM_PACKET_SIZE, gdb_usb_out_cb);
#else
	usbd_ep_setup(dev, CDCACM_GDB_ENDPOINT2, USB_ENDPOINT_ATTR_BULK,
	              CDCACM_PACKET_SIZE, NULL);
#endif
	usbd_ep_setup(dev, CDCACM_GDB_ENDPOINT2 | USB_REQ_TYPE_IN,
				  USB_ENDPOINT_ATTR_BULK, CDCACM_PACKET_SIZE, NULL);
	usbd_ep_setup(dev, (CDCACM_GDB_ENDPOINT2 + 1) | USB_REQ_TYPE_IN,
				  USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

	/* Serial interface */
	usbd_ep_setup(dev, CDCACM_UART_ENDPOINT2, USB_ENDPOINT_ATTR_BULK,
	              CDCACM_PACKET_SIZE / 2, usbuart_usb_out_cb);
	usbd_ep_setup(dev, CDCACM_UART_ENDPOINT2 | USB_REQ_TYPE_IN,
				  USB_ENDPOINT_ATTR_BULK,
	              CDCACM_PACKET_SIZE, usbuart_usb_in_cb);
	usbd_ep_setup(dev, (CDCACM_UART_ENDPOINT2 + 1) | USB_REQ_TYPE_IN,
				  USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

#if defined(PLATFORM_HAS_TRACESWO)
	/* Trace interface */
	usbd_ep_setup(dev, TRACE_ENDPOINT2 | USB_REQ_TYPE_IN, USB_ENDPOINT_ATTR_BULK,
					TRACE_ENDPOINT_SIZE2, trace_buf_drain);
#endif
#if defined(PLATFORM_HAS_SLCAN)
	/* SLCAN interface */
	usbd_ep_setup(dev, CDCACM_SLCAN_ENDPOINT2, USB_ENDPOINT_ATTR_BULK,
				  CDCACM_PACKET_SIZE, slcan_usb_out_cb);
	usbd_ep_setup(dev, CDCACM_SLCAN_ENDPOINT2 | USB_REQ_TYPE_IN, USB_ENDPOINT_ATTR_BULK,
				  CDCACM_PACKET_SIZE, NULL);
	usbd_ep_setup(dev, (CDCACM_SLCAN_ENDPOINT2 + 1) | USB_REQ_TYPE_IN, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);
#endif

	usbd_register_control_callback(dev,
			USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
			USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
			cdcacm_control_request);

	/* Notify the host that DCD is asserted.
	 * Allows the use of /dev/tty* devices on *BSD/MacOS
	 */
	cdcacm_set_modem_state(dev, GDB_IF_NO, true, true);
	cdcacm_set_modem_state(dev, UART_IF_NO, true, true);
#if defined(PLATFORM_HAS_SLCAN)
	cdcacm_set_modem_state(dev, SLCAN_IF_NO, true, true);
#endif
}

/* We need a special large control buffer for this device: */
#if defined(PLATFORM_HAS_SLCAN)
uint8_t usbd_control_buffer[512];
#else
uint8_t usbd_control_buffer[256];
#endif

void cdcacm_init(void)
{
	void exti15_10_isr(void);

//	serial_no_read(serial_no);

}

