/*
 * This file is part of the i2c-star project.
 *
 * Copyright (C) 2014 Daniel Thompson <daniel@redfelineninja.org.uk>
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

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libopencm3/usb/usbd.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <librfn/console.h>
#include <librfn/fibre.h>
#include <librfn/time.h>
#include <librfn/util.h>
#include <librfm3/i2c_ctx.h>
#include <libopencm3/usb/cdc.h>

#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/timer.h>

#include "usbuart.h"
// #include <pwmf1.h>

static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,  //16ton was 0xff
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0403,
	.idProduct = 0xc631,
	.bcdDevice = 0x0205,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 0,
	.bNumConfigurations = 1,
};

const struct usb_endpoint_descriptor i2c_endpoint = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x81,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 4,
	.bInterval = 0x20,
};

const struct usb_interface_descriptor i2c_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = 0,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = &i2c_endpoint,
};

const struct usb_endpoint_descriptor gpio_endpoint = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x83,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 4,
	.bInterval = 0x20,
};

const struct usb_interface_descriptor gpio_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = 0,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 1,

	.endpoint = &gpio_endpoint,
};

static const struct usb_endpoint_descriptor comm_endp[] = {
  {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x85,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 16,
    .bInterval = 255,
  }
};

static const struct usb_endpoint_descriptor data_endp[] = {
  {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x07,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
  },
  {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x87,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
  }
};

static const struct {
  struct usb_cdc_header_descriptor header;
  struct usb_cdc_call_management_descriptor call_mgmt;
  struct usb_cdc_acm_descriptor acm;
  struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
  .header = {
    .bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
    .bDescriptorType = CS_INTERFACE,
    .bDescriptorSubtype = USB_CDC_TYPE_HEADER,
    .bcdCDC = 0x0110,
  },
  .call_mgmt = {
    .bFunctionLength = 
    sizeof(struct usb_cdc_call_management_descriptor),
    .bDescriptorType = CS_INTERFACE,
    .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
    .bmCapabilities = 0,
    .bDataInterface = 3,     //  16ton was 1
  },
  .acm = {
    .bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
    .bDescriptorType = CS_INTERFACE,
    .bDescriptorSubtype = USB_CDC_TYPE_ACM,
    .bmCapabilities = 0,
  },
  .cdc_union = {
    .bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
    .bDescriptorType = CS_INTERFACE,
    .bDescriptorSubtype = USB_CDC_TYPE_UNION,
    .bControlInterface = 2,
    .bSubordinateInterface0 = 3, 
  }
};

static const struct usb_interface_descriptor comm_iface[] = {
  {
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 2,
    .bAlternateSetting = 0,
    .bNumEndpoints = 1,
    .bInterfaceClass = 0x02,
    .bInterfaceSubClass = 0x02,
    .bInterfaceProtocol = 0x02,
    .iInterface = 0,
    .endpoint = comm_endp,
    .extra = &cdcacm_functional_descriptors,
    .extralen = sizeof(cdcacm_functional_descriptors)
  }
};

static const struct usb_interface_descriptor data_iface[] = {
  {
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 3,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = 0x0a,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = 0,
    .endpoint = data_endp,
  }
};


const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &i2c_iface,
	}, {
	.num_altsetting = 1,
	.altsetting = &gpio_iface,
	}, {
    .num_altsetting = 1,
    .altsetting = comm_iface,
    }, {
    .num_altsetting = 1,
    .altsetting = data_iface,
}};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0, /* ?automatically calculated? */
	.bNumInterfaces = 4,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80, /* bus powered */
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char *usb_strings[] = {
	"redfelineninja.org.uk",
	"i2c-stm32f1-usb",
};
static void cdcacm_set_modem_state(usbd_device *dev, int iface, bool dsr, bool dcd);

/*
static enum usbd_request_return_codes cdcacm_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
		uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)buf;
	(void)usbd_dev;

	switch(req->bRequest) {
	case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
		/*
		 * This Linux cdc_acm driver requires this to be implemented
		 * even though it's optional in the CDC spec, and we don't
		 * advertise it in the ACM functional descriptor.
		 
		char local_buf[10];
		struct usb_cdc_notification *notif = (void *)local_buf;

		
		notif->bmRequestType = 0xA1;
		notif->bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
		notif->wValue = 0;
		notif->wIndex = 0;
		notif->wLength = 2;
		local_buf[8] = req->wValue & 3;
		local_buf[9] = 0;
		// usbd_ep_write_packet(0x83, buf, 10);
		return USBD_REQ_HANDLED;
		}
	case USB_CDC_REQ_SET_LINE_CODING: 
		if(*len < sizeof(struct usb_cdc_line_coding))
			return USBD_REQ_NOTSUPP;

		return USBD_REQ_HANDLED;
	}
//	return USBD_REQ_NOTSUPP;
    return 0;
}
*/
static enum usbd_request_return_codes  cdcacm_control_request(usbd_device *usbd_dev,
		struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
		void (**complete)(usbd_device *dev, struct usb_setup_data *req))
{
	(void)dev;
	(void)complete;
	(void)buf;
	(void)len;

	switch(req->bRequest) {
	case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
		cdcacm_set_modem_state(usbd_dev, req->wIndex, true, true);
        switch(req->wIndex) {
			case 2:
			    #ifdef USBUSART_DTR_PIN
				gpio_set_val(USBUSART_PORT, USBUSART_DTR_PIN, !(req->wValue & 1));
				#endif
				#ifdef USBUSART_RTS_PIN
				gpio_set_val(USBUSART_PORT, USBUSART_RTS_PIN, !((req->wValue >> 1) & 1));
				#endif
				return USBD_REQ_HANDLED;

			default:
				return USBD_REQ_NOTSUPP;
		}
	case USB_CDC_REQ_SET_LINE_CODING:
		if(*len < sizeof(struct usb_cdc_line_coding))
			return USBD_REQ_NOTSUPP;

		switch(req->wIndex) {
		case 2:
			usbuart_set_line_coding((struct usb_cdc_line_coding*)*buf);
			return USBD_REQ_HANDLED;

		default:
			return USBD_REQ_NOTSUPP;
		}
}
	return USBD_REQ_NOTSUPP;
	
}

static void cdcacm_set_modem_state(usbd_device *usbd_dev, int iface, bool dsr, bool dcd)
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
	/* FIXME: Remove magic numbers */
	usbd_ep_write_packet(usbd_dev, 0x85 + iface, buf, 10);
}



/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

/* commands from USB, must e.g. match command ids in kernel driver */
#define CMD_ECHO       0
#define CMD_GET_FUNC   1
#define CMD_SET_DELAY  2
#define CMD_GET_STATUS 3

#define CMD_I2C_IO     4
#define CMD_I2C_BEGIN  1  // flag fo I2C_IO
#define CMD_I2C_END    2  // flag fo I2C_IO

/* linux kernel flags */
#define I2C_M_TEN		0x10	/* we have a ten bit chip address */
#define I2C_M_RD		0x01
#define I2C_M_NOSTART		0x4000
#define I2C_M_REV_DIR_ADDR	0x2000
#define I2C_M_IGNORE_NAK	0x1000
#define I2C_M_NO_RD_ACK		0x0800

/* To determine what functionality is present */
#define I2C_FUNC_I2C			0x00000001
#define I2C_FUNC_10BIT_ADDR		0x00000002
#define I2C_FUNC_PROTOCOL_MANGLING	0x00000004 /* I2C_M_{REV_DIR_ADDR,NOSTART,..} */
#define I2C_FUNC_SMBUS_HWPEC_CALC	0x00000008 /* SMBus 2.0 */
#define I2C_FUNC_NOSTART		0x00000010 /* I2C_M_NOSTART */
#define I2C_FUNC_SMBUS_READ_WORD_DATA_PEC  0x00000800 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_WRITE_WORD_DATA_PEC 0x00001000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_PROC_CALL_PEC	0x00002000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_BLOCK_PROC_CALL_PEC 0x00004000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_BLOCK_PROC_CALL	0x00008000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_QUICK		0x00010000
#define I2C_FUNC_SMBUS_READ_BYTE	0x00020000
#define I2C_FUNC_SMBUS_WRITE_BYTE	0x00040000
#define I2C_FUNC_SMBUS_READ_BYTE_DATA	0x00080000
#define I2C_FUNC_SMBUS_WRITE_BYTE_DATA	0x00100000
#define I2C_FUNC_SMBUS_READ_WORD_DATA	0x00200000
#define I2C_FUNC_SMBUS_WRITE_WORD_DATA	0x00400000
#define I2C_FUNC_SMBUS_PROC_CALL	0x00800000
#define I2C_FUNC_SMBUS_READ_BLOCK_DATA	0x01000000
#define I2C_FUNC_SMBUS_WRITE_BLOCK_DATA 0x02000000
#define I2C_FUNC_SMBUS_READ_I2C_BLOCK	0x04000000 /* I2C-like block xfer  */
#define I2C_FUNC_SMBUS_WRITE_I2C_BLOCK	0x08000000 /* w/ 1-byte reg. addr. */
#define I2C_FUNC_SMBUS_READ_I2C_BLOCK_2	 0x10000000 /* I2C-like block xfer  */
#define I2C_FUNC_SMBUS_WRITE_I2C_BLOCK_2 0x20000000 /* w/ 2-byte reg. addr. */
#define I2C_FUNC_SMBUS_READ_BLOCK_DATA_PEC  0x40000000 /* SMBus 2.0 */
#define I2C_FUNC_SMBUS_WRITE_BLOCK_DATA_PEC 0x80000000 /* SMBus 2.0 */

#define I2C_FUNC_SMBUS_BYTE I2C_FUNC_SMBUS_READ_BYTE | \
                            I2C_FUNC_SMBUS_WRITE_BYTE
#define I2C_FUNC_SMBUS_BYTE_DATA I2C_FUNC_SMBUS_READ_BYTE_DATA | \
                                 I2C_FUNC_SMBUS_WRITE_BYTE_DATA
#define I2C_FUNC_SMBUS_WORD_DATA I2C_FUNC_SMBUS_READ_WORD_DATA | \
                                 I2C_FUNC_SMBUS_WRITE_WORD_DATA
#define I2C_FUNC_SMBUS_BLOCK_DATA I2C_FUNC_SMBUS_READ_BLOCK_DATA | \
                                  I2C_FUNC_SMBUS_WRITE_BLOCK_DATA
#define I2C_FUNC_SMBUS_I2C_BLOCK I2C_FUNC_SMBUS_READ_I2C_BLOCK | \
                                  I2C_FUNC_SMBUS_WRITE_I2C_BLOCK

#define I2C_FUNC_SMBUS_EMUL I2C_FUNC_SMBUS_QUICK | \
                            I2C_FUNC_SMBUS_BYTE | \
                            I2C_FUNC_SMBUS_BYTE_DATA | \
                            I2C_FUNC_SMBUS_WORD_DATA | \
                            I2C_FUNC_SMBUS_PROC_CALL | \
                            I2C_FUNC_SMBUS_WRITE_BLOCK_DATA | \
                            I2C_FUNC_SMBUS_WRITE_BLOCK_DATA_PEC | \
                            I2C_FUNC_SMBUS_I2C_BLOCK

#define GPIO1_PORT   GPIOC      //LED
#define GPIO1_PIN    GPIO13     //LED
#define GPIO2_PORT   GPIOB
#define GPIO2_PIN    GPIO12
#define GPIO3_PORT   GPIOB       //BTN
#define GPIO3_PIN    GPIO13       //BTN
#define GPIO4_PORT   GPIOB
#define GPIO4_PIN    GPIO14
#define GPIO5_PORT   GPIOB
#define GPIO5_PIN    GPIO4
#define GPIO6_PORT
#define GPIO6_PIN
#define GPIO7_PORT
#define GPIO7_PIN
#define GPIO8_PORT
#define GPIO8_PIN

#define LED1_PORT
#define LED1_PIN

#define IRQ_TYPE_NONE		0
#define IRQ_TYPE_EDGE_RISING	0x00000001
#define IRQ_TYPE_EDGE_FALLING	0x00000002
#define IRQ_TYPE_EDGE_BOTH	IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING
#define IRQ_TYPE_LEVEL_HIGH	0x00000004
#define IRQ_TYPE_LEVEL_LOW	0x00000008

/* the currently support capability is quite limited */
const unsigned long func = I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_NOSTART;


#define STATUS_IDLE	   0
#define STATUS_ADDRESS_ACK 1
#define STATUS_ADDRESS_NACK 2

static uint8_t status = STATUS_IDLE;

int irqtype = EXTI_TRIGGER_FALLING;

uint32_t i2c = I2C1;

/*!
 * \brief Handle I2C I/O request.
 *
 * \todo There is no bus error checking at all...
 */
 
 static void my_delay_2( void )
{
	for (unsigned i = 0; i < 20000; i++)
	  {
		__asm__("nop");
	  }
}

static void irq_pin_init(void)
{
//    nvic_disable_irq(NVIC_EXTI0_IRQ);
	my_delay_2();
    nvic_enable_irq(NVIC_EXTI4_IRQ);					
	gpio_set_mode(GPIO5_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO5_PIN);

	my_delay_2();
	exti_select_source(EXTI4, GPIO5_PORT);
//	state.falling = false;
//	exti_set_trigger(EXTI0, EXTI_TRIGGER_RISING);
    exti_set_trigger(EXTI4, irqtype);
	exti_enable_request(EXTI4);
}

static void irq_none(void)
{
//    nvic_disable_irq(NVIC_EXTI1_IRQ);
	my_delay_2();				
	exti_disable_request(EXTI4);
	my_delay_2();
}

static void pwm_probe(void)
{
//    pwm_setup_timer(RCC_TIM3, TIM3, 2, 1000);
//    pwm_setup_output_channel(TIM3, TIM_OC1, RCC_APB2ENR, GPIOA, GPIO8);
//    pwm_set_pulse_width(TIM3, 0, 800);
//    pwm_setup_output_channel(TIM3, TIM_OC2, RCC_APB2ENR, GPIOA, GPIO9);
//    pwm_set_pulse_width(TIM3, 1, 400);
//    pwm_setup_output_channel(TIM3, TIM_OC3, RCC_APB2ENR, GPIOA, GPIO10);
//    pwm_set_pulse_width(TIM3, 2, 200);
//    pwm_start_timer(TIM3);
/*    pwm_init();
    pwm_set_frequency(1000);
	pwm_set_dc(PWM_CH1, 0);
	pwm_set_dc(PWM_CH2, 0);
	pwm_set_dc(PWM_CH3, 0);
	pwm_start();
	my_delay_2();
    pwm_set_frequency(1000);
	pwm_set_dc(PWM_CH1, 500);
	pwm_set_dc(PWM_CH2, 500);
	pwm_set_dc(PWM_CH3, 500);
	my_delay_2();
	*/

}	
	
static void pwm_disable(void)
{
/*	timer_set_oc_value(TIM1, TIM_OC2, 0);
	timer_set_oc_value(TIM1, TIM_OC3, 0);
	pwm_set_dc(PWM_CH1, 0);
	pwm_set_dc(PWM_CH2, 0);
	pwm_set_dc(PWM_CH3, 0);
	timer_disable_oc_output(TIM1, TIM_OC1);
	timer_disable_oc_output(TIM1, TIM_OC2);
	timer_disable_oc_output(TIM1, TIM_OC3);
	my_delay_2();
	* */
}	


static int usb_i2c_io(struct usb_setup_data *req, uint8_t *buf, uint16_t *len)
{
	uint32_t reg32 __attribute__((unused));

	/* Interpret the request */
	uint8_t cmd = req->bRequest;
	uint8_t address = req->wIndex;
	uint8_t is_read = req->wValue & I2C_M_RD;
	uint8_t size = req->wLength;

	i2c_ctx_t ctx;

	i2c_ctx_init(&ctx, I2C1);

	if (!(req->wValue & I2C_M_NOSTART)) {
		/* We can ignore CMD_I2C_BEGIN, the hardware will work out which
		 * type of start condition to generate.
		 */
		PT_CALL(&ctx.leaf, i2c_ctx_start(&ctx));
		if (ctx.err)
			goto err;

		/* Send the address */
		PT_CALL(&ctx.leaf,
			i2c_ctx_sendaddr(&ctx, address, (is_read ? size : 0)));
		if (ctx.err)
			goto err;
	}

	/* Perform the transaction */
	for (int i=0; i<size; i++) {
		PT_CALL(&ctx.leaf, is_read ? i2c_ctx_getdata(&ctx, buf + i)
					    : i2c_ctx_senddata(&ctx, buf[i]));
		if (ctx.err)
			goto err;
	}

	/* Stop the transaction if requested and this is a write transaction
	 * (reads are stopped automatically)
	 */
	if (cmd & CMD_I2C_END && !is_read) {
		PT_CALL(&ctx.leaf, i2c_ctx_stop(&ctx));
		if (ctx.err)
			goto err;
	}

	status = STATUS_ADDRESS_ACK;
	*len = (is_read ? size : 0);
	return USBD_REQ_HANDLED;

err:
	i2c_ctx_reset(&ctx);
	status = STATUS_ADDRESS_NACK;
	*len = 0;
	return USBD_REQ_HANDLED;
}

static void my_delay_1( void )
{
   for (unsigned i = 0; i < 800000; i++)
     {
        __asm__("nop");
     }
}


static void usbgpio_output(int gpio)
{
	if (gpio == 1) {
	gpio_set_mode(GPIO1_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO1_PIN);
	gpio_set(GPIO1_PORT, GPIO1_PIN);
    } else if (gpio == 2) {
	gpio_set_mode(GPIO2_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO2_PIN);
	gpio_set(GPIO2_PORT, GPIO2_PIN);
	} else if (gpio == 3) {
	gpio_set_mode(GPIO3_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO3_PIN);
	gpio_set(GPIO3_PORT, GPIO3_PIN);
	} else if (gpio == 4) {
	gpio_set_mode(GPIO4_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO4_PIN);
	gpio_set(GPIO4_PORT, GPIO4_PIN);
	} else if (gpio == 5) {
	gpio_set_mode(GPIO5_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO5_PIN);
	gpio_set(GPIO5_PORT, GPIO5_PIN);
	}
	
    my_delay_1();
}

static void usbgpio_input(int gpio)
{

	if (gpio == 1) {
	gpio_set_mode(GPIO1_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO1_PIN);
	gpio_set(GPIO1_PORT, GPIO1_PIN);
	} else if (gpio == 2) {
	gpio_set_mode(GPIO2_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO2_PIN);
	gpio_set(GPIO2_PORT, GPIO2_PIN);
	} else if (gpio == 3) {
	gpio_set_mode(GPIO3_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO3_PIN);
	gpio_set(GPIO3_PORT, GPIO3_PIN);
	} else if (gpio == 4) {
	gpio_set_mode(GPIO4_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO4_PIN);
	gpio_set(GPIO4_PORT, GPIO4_PIN);
	} else if (gpio == 5) {
	gpio_set_mode(GPIO5_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO5_PIN);
	gpio_set(GPIO5_PORT, GPIO5_PIN);
	}

	my_delay_1();
}

static enum usbd_request_return_codes usb_control_gpio_request(
    usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
    uint16_t *len,
    void (**complete)(usbd_device *, struct usb_setup_data *req))
{
    (void)complete;
	(void)usbd_dev;

   if ((req->bmRequestType & 0x7F) != USB_REQ_TYPE_VENDOR)
     return 0;

   (*len) = 1;
   (*buf)[0] = 1; //success

   if (req->wValue == 1)
     {
        if ( req->wIndex == 0 )
			{
				usbgpio_input(1);
				return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 1 )
			{
				usbgpio_input(2);
				return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 2 )
			{
				usbgpio_input(3);
				return USBD_REQ_HANDLED;
			}	
	    else if ( req->wIndex == 3 )
			{
				usbgpio_input(4);
				return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 4 )
			{
				usbgpio_input(5);
				return USBD_REQ_HANDLED;
			}	
      }
   else if (req->wValue == 2)
     {
     if ( req->wIndex == 0 )
			{
				usbgpio_output(1);
				return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 1 )
			{
				usbgpio_output(2);
				return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 2 )
			{
				usbgpio_output(3);
				return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 3 )
			{
				usbgpio_output(4);
				return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 4 )
			{
				usbgpio_output(5);
				return USBD_REQ_HANDLED;
			}
      }
   else if (req->wValue == 3)
     {
     if ( req->wIndex == 0 )
			{
            if (gpio_get(GPIO1_PORT, GPIO1_PIN)) {
            	(*buf)[0] = 1; 
		        (*buf)[1] = 4;
		        (*buf)[2] = 4;
		        (*buf)[3] = 4;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			} else {
				(*buf)[0] = 1; 
		        (*buf)[1] = 3;
		        (*buf)[2] = 3;
		        (*buf)[3] = 3;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			}
			return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 1 )
			{
            if (gpio_get(GPIO2_PORT, GPIO2_PIN)) {
            	(*buf)[0] = 1; 
		        (*buf)[1] = 4;
		        (*buf)[2] = 4;
		        (*buf)[3] = 4;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			} else {
				(*buf)[0] = 1; 
		        (*buf)[1] = 3;
		        (*buf)[2] = 3;
		        (*buf)[3] = 3;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			}
		    return USBD_REQ_HANDLED;
		   }
	    else if ( req->wIndex == 2 )
			{
            if (gpio_get(GPIO3_PORT, GPIO3_PIN)) {
            	(*buf)[0] = 1; 
		        (*buf)[1] = 4;
		        (*buf)[2] = 4;
		        (*buf)[3] = 4;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			} else {
				(*buf)[0] = 1; 
		        (*buf)[1] = 3;
		        (*buf)[2] = 3;
		        (*buf)[3] = 3;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			}
			return USBD_REQ_HANDLED;
			}
		 else if ( req->wIndex == 3 )
			{
            if (gpio_get(GPIO4_PORT, GPIO4_PIN)) {
            	(*buf)[0] = 1; 
		        (*buf)[1] = 4;
		        (*buf)[2] = 4;
		        (*buf)[3] = 4;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			} else {
				(*buf)[0] = 1; 
		        (*buf)[1] = 3;
		        (*buf)[2] = 3;
		        (*buf)[3] = 3;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			}
			return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 4 )
			{
            if (gpio_get(GPIO5_PORT, GPIO5_PIN)) {
            	(*buf)[0] = 1; 
		        (*buf)[1] = 4;
		        (*buf)[2] = 4;
		        (*buf)[3] = 4;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			} else {
				(*buf)[0] = 1; 
		        (*buf)[1] = 3;
		        (*buf)[2] = 3;
		        (*buf)[3] = 3;
			    *len = sizeof(buf);
			    return USBD_REQ_HANDLED;
			}
			return USBD_REQ_HANDLED;
			}
         }
  else if (req->wValue == 0)
     { 
	 if (req->bRequest == 1)
        {
        if ( req->wIndex == 0 )
			{
				gpio_clear(GPIO1_PORT, GPIO1_PIN);
				return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 1 )
			{
				gpio_set(GPIO2_PORT, GPIO2_PIN);
				return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 2 )
			{
				gpio_set(GPIO3_PORT, GPIO3_PIN);
				return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 3 )
			{
				gpio_set(GPIO4_PORT, GPIO4_PIN);
				return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 4 )
			{
				gpio_set(GPIO5_PORT, GPIO5_PIN);
				return USBD_REQ_HANDLED;
			}
        }
   else if (req->bRequest == 0)
     {
     if (req->wIndex == 0)
			{
				gpio_set(GPIO1_PORT, GPIO1_PIN);
				return USBD_REQ_HANDLED;
			}
	    else if ( req->wIndex == 1 )
			{
				gpio_clear(GPIO2_PORT, GPIO2_PIN);
				return USBD_REQ_HANDLED;
			}	
		else if ( req->wIndex == 2 )
			{
				gpio_clear(GPIO3_PORT, GPIO3_PIN);
				return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 3 )
			{
				gpio_clear(GPIO4_PORT, GPIO4_PIN);
				return USBD_REQ_HANDLED;
			}
		else if ( req->wIndex == 4 )
			{
				gpio_clear(GPIO5_PORT, GPIO5_PIN);
				return USBD_REQ_HANDLED;
			}	
	  }
    }
       else if (req->wValue == 4)
     { 
     if (req->wIndex == 1) {
          pwm_probe();
          return USBD_REQ_HANDLED;
     } else if (req->wIndex == 2) {
          pwm_disable();
          return USBD_REQ_HANDLED;
       }
     }
    else if (req->wValue == 5)
     { 
//    pwm_period(1, req->wIndex);
//    pwm_duty(req->bRequest);
      if ( req->wIndex == 0 ) {
 //       pwm_set_dc(PWM_CH1, req->bRequest*4);
        return USBD_REQ_HANDLED;
      }	    
      else if ( req->wIndex == 104 ) {
//        pwm_set_dc(PWM_CH2, req->bRequest*4);
        return USBD_REQ_HANDLED;
      }
      else if ( req->wIndex == 208 ) {
//        pwm_set_dc(PWM_CH3, req->bRequest*4);
        return USBD_REQ_HANDLED;
      }
//        pwm_set_dc(PWM_CH2, req->bRequest);
        return USBD_REQ_HANDLED;
     }
    else if (req->wValue == 9)
     {
     if ( req->wIndex == 1 ) {
        irqtype = IRQ_TYPE_NONE;
        irq_pin_init();
     return USBD_REQ_HANDLED;
     }
     else if ( req->wIndex == 2 ) {
        irqtype = IRQ_TYPE_LEVEL_HIGH;
     return USBD_REQ_HANDLED;
     }
     else if ( req->wIndex == 3 ) {
        irqtype = IRQ_TYPE_LEVEL_LOW;
     return USBD_REQ_HANDLED;
     }
     else if ( req->wIndex == 4 ) {
//        irqtype = IRQ_TYPE_EDGE_BOTH;
        irqtype = EXTI_TRIGGER_BOTH;   
        irq_pin_init();
     return USBD_REQ_HANDLED;
     }
     else if ( req->wIndex == 5 ) {
//        irqtype = IRQ_TYPE_EDGE_RISING;
        irqtype = EXTI_TRIGGER_RISING;
        irq_pin_init();
     return USBD_REQ_HANDLED;
     }
     else if ( req->wIndex == 6 ) {
//        irqtype = IRQ_TYPE_EDGE_FALLING;
        irqtype = EXTI_TRIGGER_FALLING;
        irq_pin_init();
     return USBD_REQ_HANDLED;
     }
     }
   else
     {
        (*buf)[0] = -1; // FAILURE
     }
 
   return 1;
}

static enum usbd_request_return_codes usb_control_request(
    usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
    uint16_t *len,
    void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	static uint8_t reply_buf[64];

	(void)usbd_dev;
	(void)complete;

	switch (req->bRequest) {
	case CMD_ECHO:
		memcpy(reply_buf, &req->wValue, sizeof(req->wValue));
		*buf = reply_buf;
		*len = sizeof(req->wValue);
		return USBD_REQ_HANDLED;

	case CMD_GET_FUNC:
		/* Report our capabilities */
		memcpy(reply_buf, &func, sizeof(func));
		*buf = reply_buf;
		*len = sizeof(func);
		return USBD_REQ_HANDLED;

	case CMD_SET_DELAY:
		/* This was used in i2c-tiny-usb to choose the clock
		 * frequency by specifying the shortest time between
		 * clock edges.
		 *
		 * This implementation silently ignores delay requests. We
		 * run the hardware as fast as we are permitted.
		 */
		*buf = reply_buf;
		*len = 0;
		return USBD_REQ_HANDLED;

	case CMD_I2C_IO:
	case CMD_I2C_IO | CMD_I2C_BEGIN:
	case CMD_I2C_IO | CMD_I2C_END:
	case CMD_I2C_IO | CMD_I2C_BEGIN | CMD_I2C_END:
		if (req->wValue & I2C_M_RD)
			*buf = reply_buf;
		return usb_i2c_io(req, *buf, len);
		break;

	case CMD_GET_STATUS:
		memcpy(reply_buf, &status, sizeof(status));
		*buf = reply_buf;
		*len = sizeof(status);
		return USBD_REQ_HANDLED;

	default:
		break;

	}

	return USBD_REQ_NEXT_CALLBACK;
}



static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
  (void)ep;
  char buf[64];
  int len = usbd_ep_read_packet(usbd_dev, 0x07, buf, 64);

  if (len) {
    usbd_ep_write_packet(usbd_dev, 0x87, buf, len);
  }
}

static void usb_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, 4, NULL);

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_VENDOR | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				usb_control_request);
	
	usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 9, NULL);

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_VENDOR,
				USB_REQ_TYPE_TYPE,
				usb_control_gpio_request);
				
  usbd_ep_setup(usbd_dev, 0x07, USB_ENDPOINT_ATTR_BULK, 64, usbuart_usb_in_cb);
  usbd_ep_setup(usbd_dev, 0x87, USB_ENDPOINT_ATTR_BULK, 64, usbuart_usb_out_cb);
  usbd_ep_setup(usbd_dev, 0x85, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

  usbd_register_control_callback(usbd_dev,
				 USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				 USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				 cdcacm_control_request);

        char buf[10];
		struct usb_cdc_notification *notif = (void *)buf;

		
		notif->bmRequestType = 0xA1;
		notif->bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
		notif->wValue = 0;
		notif->wIndex = 0;
		notif->wLength = 2;
		buf[8] = 3;
		buf[9] = 0;
		usbd_ep_write_packet(usbd_dev, 0x87, buf, 10);
//		return USBD_REQ_HANDLED;
		
}



static usbd_device *usbd_dev;

static int usb_fibre(fibre_t *fibre)
{
	static uint32_t t;

	PT_BEGIN_FIBRE(fibre);

	rcc_periph_clock_enable(RCC_GPIOA);

	/* lower hotplug and leave enough time for the host to notice */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
		      GPIO11 | GPIO12);
	gpio_clear(GPIOA, GPIO11 | GPIO12);

#ifdef USE_MAPLEMINI
	/* add support for Maplemini board */
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
		      GPIO9);
	gpio_clear(GPIOB, GPIO9);
#endif

	t = time_now() + 10000;
	PT_WAIT_UNTIL(fibre_timeout(t));

	usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config,
			usb_strings, 2,
			usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, usb_set_config);

	while (true) {
		usbd_poll(usbd_dev);
		PT_YIELD();
	}

	PT_END();
}
static fibre_t usb_task = FIBRE_VAR_INIT(usb_fibre);
static console_t uart_console;

void exti4_isr(void)
{
    // char buf2[64] __attribute__ ((aligned(4)));
//    uint8_t buft[4] = {3, 3, 3, 3};
    static uint8_t buft[4];
    buft[0] = 1; 
	buft[1] = 3;
	buft[2] = 3;
	buft[3] = 3;
	exti_reset_request(EXTI4);
//	usbd_ep_write_packet(usbd_device usbd_dev, 0x83, buf2, 64);
    usbd_ep_write_packet(usbd_dev, 0x83, buft, sizeof(buft));
    exti_set_trigger(EXTI4, irqtype);
}

static void i2c_init(void)
{
	/* clocks */
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_I2C1);
	rcc_periph_clock_enable(RCC_AFIO);

	/* initialize the peripheral */
	i2c_ctx_t ctx;
	i2c_ctx_init(&ctx, I2C1);
	i2c_ctx_reset(&ctx);

	/* GPIO for I2C1 */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO6 | GPIO7);
}

static void jump_to_bootloader(void)
{
	char *const marker = (char *)0x20004800; /* RAM@18K */
	const char key[] = "remain-in-loader";

	memcpy(marker, key, sizeof(key));
	scb_reset_system(); /* Will never return. */
}

static pt_state_t do_id(console_t *c)
{
	char serial_no[25];

	desig_get_unique_id_as_string(serial_no, sizeof(serial_no));
	fprintf(c->out, "%s\n", serial_no);

	return PT_EXITED;
}

static pt_state_t do_reboot(console_t *c)
{
	(void)c;
	jump_to_bootloader();
	return PT_EXITED;
}

static const console_cmd_t cmds[] = {
	CONSOLE_CMD_VAR_INIT("id", do_id),
	CONSOLE_CMD_VAR_INIT("reboot", do_reboot),
};

static void gpio_init(void)
{
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO0 | GPIO1);

	gpio_set_mode(GPIO1_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO1_PIN);
	gpio_set_mode(GPIO2_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO2_PIN);
	gpio_set_mode(GPIO3_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO3_PIN);
	gpio_set_mode(GPIO4_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO4_PIN);
	gpio_set_mode(GPIO5_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO5_PIN);

    my_delay_1();
	gpio_clear(GPIO1_PORT, GPIO1_PIN);
	gpio_set(GPIO2_PORT, GPIO2_PIN);
	gpio_set(GPIO4_PORT, GPIO4_PIN);
	gpio_set(GPIO3_PORT, GPIO3_PIN);
	gpio_set(GPIO5_PORT, GPIO5_PIN);
	gpio_set(GPIOC, GPIO1);
    gpio_set(GPIOC, GPIO0);
}

int main(void)
{
	unsigned int i;

	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);

	console_init(&uart_console, stdout);
	for (i=0; i<lengthof(cmds); i++)
		console_register(&cmds[i]);

	i2c_init();
	time_init();
//	tim_setup();
	gpio_init();
	irq_pin_init();
//	exti_setup();
    usbuart_init();
//     cdcacm_init();
	printf("Booted OK\n");

	for (i = 0; i < 0x800000; i++)
		__asm__("nop");

	/* prepare the scheduler */
	fibre_run(&usb_task);

	fibre_scheduler_main_loop();
	return 0;
}
