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
#include "platform.h"
#include <libopencm3/usb/usbd.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/dwc/otg_common.h>
#include <libopencm3/usb/dwc/otg_fs.h>
#include <libopencm3/stm32/timer.h>
#include <librfn/fibre.h>
#include <librfn/time.h>
#include <librfn/util.h>
#include <librfm3/i2c_ctx.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/spi.h>
#include <pwm.h>
#include "usbuart.h"
#include "ws2812_spi.h"
#include "timing_stm32.h"

#define CDCACM_PACKET_SIZE 64
//#include "clock.h"
void clock_setup(void);
void milli_sleep(uint32_t delay);
static volatile uint32_t system_millis;
uint32_t mtime(void);

uint16_t usb_config;
void delay_setup(void);

void neopixel_init(void);
/* Called when systick fires */

//void sys_tick_handler(void)
//{
//	system_millis++;
//}

/* simple sleep for delay milliseconds */
/*
=======
void sys_tick_handler(void)
{
	system_millis++;
}

 */

void milli_sleep(uint32_t delay)
{
	uint32_t wake = system_millis + delay;
	while (wake > system_millis) {
		continue;
	}
}


/* Getter function for the current time */
/*
=======

/* Getter function for the current time */

uint32_t mtime(void)
{
	return system_millis;
}



static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0xef,
	.bDeviceSubClass = 2,
	.bDeviceProtocol = 1,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0403,
	.idProduct = 0xc631,
	.bcdDevice = 0x0205,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 0,
	.bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor uart_comm_endp = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = (CDCACM_UART_ENDPOINT + 1) | USB_REQ_TYPE_IN,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 16,
	.bInterval = 255,
};

static const struct usb_endpoint_descriptor uart_data_endp[] = {
	{
		.bLength = USB_DT_ENDPOINT_SIZE,
		.bDescriptorType = USB_DT_ENDPOINT,
		.bEndpointAddress = CDCACM_UART_ENDPOINT,
		.bmAttributes = USB_ENDPOINT_ATTR_BULK,
		.wMaxPacketSize = CDCACM_PACKET_SIZE / 2,
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
			.bmCapabilities = 2, /* SET_LINE_CODING supported*/
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
	.iInterface = 3,

	.endpoint = &uart_comm_endp,

	.extra = &uart_cdcacm_functional_descriptors,
	.extralen = sizeof(uart_cdcacm_functional_descriptors),
};

static const struct usb_interface_descriptor uart_data_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = UART_IF_NO + 1,
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
	.bFirstInterface = 0,
	.bInterfaceCount = 2,
	.bFunctionClass = USB_CLASS_CDC,
	.bFunctionSubClass = USB_CDC_SUBCLASS_ACM,
	.bFunctionProtocol = USB_CDC_PROTOCOL_NONE,
	.iFunction = 3,
};

const struct usb_endpoint_descriptor i2c_endpoint = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x83,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 4,
	.bInterval = 0x20,
};

const struct usb_interface_descriptor i2c_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 2,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = 0,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 3,

	.endpoint = &i2c_endpoint,
};

const struct usb_endpoint_descriptor gpio_endpoint = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x84,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 4,
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
	.iInterface = 1,

	.endpoint = &gpio_endpoint,
};

const struct usb_interface ifaces[] = {
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
    	.altsetting = &i2c_iface,
	}, {
    	.num_altsetting = 1,
    	.altsetting = &gpio_iface,
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
	"i2c-stm32f4-usb",
	"uart",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[256];

// gpio
#define USB_CMD_WRITE       0
#define USB_CMD_READ        1
#define USB_CMD_GPIO_OUTPUT 2
#define USB_CMD_GPIO_INPUT  3
#define USB_CMD_GPIO_SET    4
#define USB_CMD_GPIO_GET    5

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
#define GPIO2_PORT   GPIOC
#define GPIO2_PIN    GPIO14
#define GPIO3_PORT   GPIOA       //BTN
#define GPIO3_PIN    GPIO0       //BTN
#define GPIO4_PORT   GPIOC
#define GPIO4_PIN    GPIO15
#define GPIO5_PORT   GPIOA
#define GPIO5_PIN    GPIO1
#define GPIO6_PORT
#define GPIO6_PIN
#define GPIO7_PORT
#define GPIO7_PIN
#define GPIO8_PORT
#define GPIO8_PIN


#define PWM0_PORT    GPIOC
#define PWM0_PIN     GPIO6
#define PWM1_PORT    GPIOC
#define PWM1_PIN     GPIO7
#define PWM2_PORT    GPIOC
#define PWM2_PIN     GPIO8

#define PWM3_PORT    GPIOC
#define PWM3_PIN     GPIO9

#define LED1_PORT
#define LED1_PIN

#define IRQ_TYPE_NONE		0
#define IRQ_TYPE_EDGE_RISING	0x00000001
#define IRQ_TYPE_EDGE_FALLING	0x00000002
#define IRQ_TYPE_EDGE_BOTH	IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING
#define IRQ_TYPE_LEVEL_HIGH	0x00000004
#define IRQ_TYPE_LEVEL_LOW	0x00000008
	
/* the currently support capability is quite limited */
const unsigned long func = I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;

static void usb_serial_set_state(usbd_device *dev, uint16_t iface, uint8_t ep);

#define STATUS_IDLE	   0
#define STATUS_ADDRESS_ACK 1
#define STATUS_ADDRESS_NACK 2

static uint8_t status = STATUS_IDLE;

uint32_t i2c = I2C1;

int irqtype = EXTI_TRIGGER_FALLING;
 
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
    nvic_enable_irq(NVIC_EXTI1_IRQ);					
	gpio_mode_setup(GPIO5_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO5_PIN);
	my_delay_2();
	exti_select_source(EXTI1, GPIO5_PORT);
    exti_set_trigger(EXTI1, irqtype);
	exti_enable_request(EXTI1);
}

static void irq_none(void)
{
    nvic_disable_irq(NVIC_EXTI1_IRQ);
	my_delay_2();				
	exti_disable_request(EXTI1);
	my_delay_2();
}

static void pwm_probe(void)
{
    pwm_init();
    pwm_set_frequency(1000000);
	pwm_set_dc(PWM_CH1, 0);
	pwm_set_dc(PWM_CH2, 0);
	pwm_set_dc(PWM_CH3, 0);

//	pwm_set_dc(PWM_CH4, 0);

	pwm_set_dc(PWM_CH4, 0);

	pwm_start();
	my_delay_2();
    pwm_set_frequency(1000000);
	pwm_set_dc(PWM_CH1, 100);
	pwm_set_dc(PWM_CH2, 200);
	pwm_set_dc(PWM_CH3, 300);

//	pwm_set_dc(PWM_CH4, 500);

	pwm_set_dc(PWM_CH4, 500);

	my_delay_2();

}	
	
static void pwm_disable(void)
{
	timer_set_oc_value(TIM8, TIM_OC1, 0);
	timer_set_oc_value(TIM8, TIM_OC2, 0);
	timer_set_oc_value(TIM8, TIM_OC3, 0);

//	timer_set_oc_value(TIM8, TIM_OC4, 0);
	pwm_set_dc(PWM_CH1, 0);
	pwm_set_dc(PWM_CH2, 0);
	pwm_set_dc(PWM_CH3, 0);
//	pwm_set_dc(PWM_CH4, 0);
	timer_disable_oc_output(TIM8, TIM_OC1);
	timer_disable_oc_output(TIM8, TIM_OC2);
	timer_disable_oc_output(TIM8, TIM_OC3);
//	timer_disable_oc_output(TIM8, TIM_OC4);


	my_delay_2();
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
	gpio_mode_setup(GPIO1_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1_PIN);
	gpio_set_output_options(GPIO1_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
							GPIO1_PIN);
	gpio_set(GPIO1_PORT, GPIO1_PIN);
    } else if (gpio == 2) {
	gpio_mode_setup(GPIO2_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO2_PIN);
	gpio_set_output_options(GPIO2_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
							GPIO2_PIN);
	gpio_set(GPIO2_PORT, GPIO2_PIN);
	} else if (gpio == 3) {
	gpio_mode_setup(GPIO3_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3_PIN);
	gpio_set_output_options(GPIO3_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
							GPIO3_PIN);
	gpio_set(GPIO3_PORT, GPIO3_PIN);
	} else if (gpio == 4) {
	gpio_mode_setup(GPIO4_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4_PIN);
	gpio_set_output_options(GPIO4_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
							GPIO4_PIN);
	gpio_set(GPIO4_PORT, GPIO4_PIN);
	} else if (gpio == 5) {
	gpio_mode_setup(GPIO5_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5_PIN);
	gpio_set_output_options(GPIO5_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
							GPIO5_PIN);
	gpio_set(GPIO5_PORT, GPIO5_PIN);
	}
	
    my_delay_1();
}

static void usbgpio_input(int gpio)
{

	if (gpio == 1) {
	gpio_mode_setup(GPIO1_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO1_PIN);
	gpio_set(GPIO1_PORT, GPIO1_PIN);
	} else if (gpio == 2) {
	gpio_mode_setup(GPIO2_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO2_PIN);
	gpio_set(GPIO2_PORT, GPIO2_PIN);
	} else if (gpio == 3) {
	gpio_mode_setup(GPIO3_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO3_PIN);
	gpio_set(GPIO3_PORT, GPIO3_PIN);
	} else if (gpio == 4) {
	gpio_mode_setup(GPIO4_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO4_PIN);
	gpio_set(GPIO4_PORT, GPIO4_PIN);
	} else if (gpio == 5) {
	gpio_mode_setup(GPIO5_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO5_PIN);
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
			    *len = 4;
			    return 1;
			} else {
				(*buf)[0] = 0; 
		        (*buf)[1] = 3;
		        (*buf)[2] = 3;
		        (*buf)[3] = 3;
			    *len = 4;
			    return 1;
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
				gpio_set(GPIO1_PORT, GPIO1_PIN);
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
				gpio_clear(GPIO1_PORT, GPIO1_PIN);
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
      if ( req->wIndex == 0 ) {
        pwm_set_dc(PWM_CH1, req->bRequest*4);
        return USBD_REQ_HANDLED;
      }	    
      else if ( req->wIndex == 104 ) {
        pwm_set_dc(PWM_CH2, req->bRequest*4);
        return USBD_REQ_HANDLED;
      }
      else if ( req->wIndex == 208 ) {
        pwm_set_dc(PWM_CH3, req->bRequest*4);
        return USBD_REQ_HANDLED;
      }
      else if ( req->wIndex == 56 ) {
        pwm_set_dc(PWM_CH4, req->bRequest*4);
        return USBD_REQ_HANDLED;
      }
        return USBD_REQ_HANDLED;
     }
    else if (req->wValue == 9)
     {
     if ( req->wIndex == 1 ) {
        irqtype = IRQ_TYPE_NONE;
        irq_none();
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

static enum usbd_request_return_codes debug_uart_control_request(usbd_device *usbd_dev, struct usb_setup_data *req,
	uint8_t **buf, uint16_t *const len, void (**complete)(usbd_device *dev, struct usb_setup_data *req))
{
	(void)complete;
	/* Is the request for the physical/debug UART interface? */
	if (req->wIndex != UART_IF_NO)
		return USBD_REQ_NEXT_CALLBACK;

	switch (req->bRequest) {
	case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
		usb_serial_set_state(usbd_dev, req->wIndex, CDCACM_UART_ENDPOINT);
#ifdef USBUSART_DTR_PIN
		gpio_set_val(USBUSART_PORT, USBUSART_DTR_PIN, !(req->wValue & 1U));
#endif
#ifdef USBUSART_RTS_PIN
		gpio_set_val(USBUSART_PORT, USBUSART_RTS_PIN, !((req->wValue >> 1U) & 1U));
#endif
		return USBD_REQ_HANDLED;
	case USB_CDC_REQ_SET_LINE_CODING:
		if (*len < sizeof(struct usb_cdc_line_coding))
			return USBD_REQ_NOTSUPP;
		usbuart_set_line_coding((struct usb_cdc_line_coding *)*buf);
		return USBD_REQ_HANDLED;
	}
	return USBD_REQ_NOTSUPP;
}

void usb_serial_set_state(usbd_device *const usbd_dev, const uint16_t iface, const uint8_t ep)
{
	uint8_t buf[10];
	struct usb_cdc_notification *notif = (void*)buf;
	/* We echo signals back to host as notification */
	notif->bmRequestType = 0xA1;
	notif->bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
	notif->wValue = 0;
	notif->wIndex = iface;
	notif->wLength = 2;
	buf[8] = 3U;
	buf[9] = 0U;
	usbd_ep_write_packet(usbd_dev, ep, buf, sizeof(buf));
}

static void usb_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
//	(void)wValue;
    usb_config = wValue;
    
	usbd_ep_setup(usbd_dev, CDCACM_UART_ENDPOINT, USB_ENDPOINT_ATTR_BULK, CDCACM_PACKET_SIZE / 2, usbuart_usb_out_cb);
	usbd_ep_setup(usbd_dev, CDCACM_UART_ENDPOINT | USB_REQ_TYPE_IN, USB_ENDPOINT_ATTR_BULK, CDCACM_PACKET_SIZE, usbuart_usb_in_cb);
	usbd_ep_setup(usbd_dev, (CDCACM_UART_ENDPOINT + 1) | USB_REQ_TYPE_IN, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);
	
	usbd_register_control_callback(usbd_dev, USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
	USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT, debug_uart_control_request);
	
	usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 4, NULL);

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_VENDOR | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				usb_control_request);

	usbd_ep_setup(usbd_dev, 0x84, USB_ENDPOINT_ATTR_INTERRUPT, 9, NULL);

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_VENDOR,
				USB_REQ_TYPE_TYPE,
				usb_control_gpio_request);
				
	usb_serial_set_state(usbd_dev, UART_IF_NO, CDCACM_UART_ENDPOINT);
}

uint16_t usb_get_config(void)
{
	return usb_config;
}

static usbd_device *usbd_dev;

static int usb_fibre(fibre_t *fibre)
{
	PT_BEGIN_FIBRE(fibre);

	rcc_periph_clock_enable(RCC_OTGFS);

	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
			GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);

	GPIOA_OSPEEDR &= 0x3C00000C;
	GPIOA_OSPEEDR |= 0x28000008;

	usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config,
			usb_strings, 2,
			usbd_control_buffer, sizeof(usbd_control_buffer));
	OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS;
//    OTG_FS_GUSBCFG |= OTG_GUSBCFG_FDMOD | OTG_GUSBCFG_TRDT_MASK;
//	OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS | OTG_GCCFG_PWRDWN;
//	OTG_FS_GCCFG &= ~(OTG_GCCFG_VBUSBSEN | OTG_GCCFG_VBUSASEN);
	usbd_register_set_config_callback(usbd_dev, usb_set_config);

	while (true) {
		usbd_poll(usbd_dev);
		PT_YIELD();
	}

	PT_END();
}
static fibre_t usb_task = FIBRE_VAR_INIT(usb_fibre);

void exti1_isr(void)
{
    // char buf2[64] __attribute__ ((aligned(4)));
    uint8_t buft[4] = {3, 3, 3, 3};
	exti_reset_request(EXTI1);
//	usbd_ep_write_packet(usbd_device usbd_dev, 0x83, buf2, 64);
    usbd_ep_write_packet(usbd_dev, 0x83, buft, 4);
    exti_set_trigger(EXTI1, irqtype);
}


static void i2c_init(void)
{
	/* clocks */
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_I2C1);

	/* initialize the peripheral */
	i2c_ctx_t ctx;
	i2c_ctx_init(&ctx, I2C1);
	i2c_ctx_reset(&ctx);

	/* GPIO for I2C1 */
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO6 | GPIO9);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ,
				GPIO6 | GPIO9);
	gpio_set_af(GPIOB, GPIO_AF4, GPIO6 | GPIO9);

}

static void gpio_init(void)
{
	rcc_periph_clock_enable(RCC_GPIOC);
//	nvic_enable_irq(NVIC_EXTI0_IRQ);
	gpio_mode_setup(GPIO1_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1_PIN);
	gpio_set_output_options(GPIO1_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
							GPIO1_PIN);

	gpio_set(GPIO1_PORT, GPIO1_PIN);

//	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO14);
//	gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
//							GPIO14);

	gpio_mode_setup(GPIO2_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO2_PIN);
	
	gpio_mode_setup(GPIO4_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4_PIN);
	gpio_set_output_options(GPIO4_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
							GPIO4_PIN);
		
//	nvic_enable_irq(NVIC_EXTI0_IRQ);					
	gpio_mode_setup(GPIO3_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO3_PIN);
	gpio_mode_setup(GPIO5_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO5_PIN);
	
    my_delay_1();
	gpio_clear(GPIO1_PORT, GPIO1_PIN);
	gpio_set(GPIO2_PORT, GPIO2_PIN);
	gpio_set(GPIO4_PORT, GPIO4_PIN);
	gpio_set(GPIO3_PORT, GPIO3_PIN);
	gpio_set(GPIO5_PORT, GPIO5_PIN);
	

	/* Configure the EXTI subsystem. 
	exti_select_source(EXTI0, GPIO3_PORT);
	state.falling = false;
	exti_set_trigger(EXTI0, EXTI_TRIGGER_RISING);
	exti_enable_request(EXTI0);
	*/
}

void neopixel_init(void)
{
 //   systime_setup(84000);
    systime_setup(84000);
//	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_SPI1);

	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO5);
	gpio_set_af(GPIOB, GPIO_AF5, GPIO5);

	// use PIN B5 as ws2812 open drain output (1K pullup -> +5V)
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO5);
	ws2812_init(SPI1);

//	int h = 0;
//	while (1) {
//		h = (h+1) % 360;
		ws2812_write_rgb(SPI1, 0, 0, 220);
//		ws2812_write_hsv(SPI1, 120, 10, 1);
//		ws2812_write_hsv(SPI1, 120, 10, 1);
		delay_ms(15);
//    }

}

void clock_setup(void)
{
	/* Base board frequency, set to 168Mhz */
//	rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

	/* clock rate / 168000 to get 1mS interrupt rate */

	systick_set_reload(84000);

	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_counter_enable();

	/* this done last */
	systick_interrupt_enable();
}


void delay_setup(void)
{
	/* set up a microsecond free running timer for ... things... */
	rcc_periph_clock_enable(RCC_TIM6);
	/* microsecond counter */
	timer_set_prescaler(TIM6, rcc_apb1_frequency / 1000000 - 1);
	timer_set_period(TIM6, 0xffff);
	timer_one_shot_mode(TIM6);
}

int main(void)
{
	int i;

    rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
    
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);

	
	gpio_init();
	i2c_init();
//	time_init();
//	clock_setup();
    platform_timing_init();
//	irq_pin_init();
//	pwm_probe();
//	gpio_clear(GPIOC, GPIO13);
	usbuart_init();

    neopixel_init();
    
	for (i = 0; i < 0x200000; i++)
		__asm__("nop");

	/* prepare the scheduler */
	fibre_run(&usb_task);

	fibre_scheduler_main_loop();
//	rcc_periph_clock_enable(RCC_OTGFS);
//	rcc_periph_clock_enable(RCC_CRC);

//	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
//			GPIO11 | GPIO12);
//	gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);

//	GPIOA_OSPEEDR &= 0x3C00000C;
//	GPIOA_OSPEEDR |= 0x28000008;

//	usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config,
//			usb_strings, 2,
//			usbd_control_buffer, sizeof(usbd_control_buffer));
//	OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS;
//    OTG_FS_GUSBCFG |= OTG_GUSBCFG_FDMOD | OTG_GUSBCFG_TRDT_MASK;
//	OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS | OTG_GCCFG_PWRDWN;
//	OTG_FS_GCCFG &= ~(OTG_GCCFG_VBUSBSEN | OTG_GCCFG_VBUSASEN);
//	usbd_register_set_config_callback(usbd_dev, usb_set_config);
//    delay_setup();
//	nvic_set_priority(USB_IRQ, IRQ_PRI_USB);
//	nvic_enable_irq(USB_IRQ);

	return 0;
}

void USB_ISR(void)
{
	usbd_poll(usbd_dev);
}

