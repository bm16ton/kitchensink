/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
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

/* This file implements the platform specific functions for the STM32
 * implementation.
 */
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include "general.h"
#include "usb.h"
#include "usbuart.h"
#include "morse.h"
#include <string.h>
#include <ctype.h>

#include <usb_gpio.h>
#include <usb_adc.h>
#include <altusb.h>
#include <setjmp.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/usb/dwc/otg_hs.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/iwdg.h>
#include <librfn/fibre.h>
#include <librfn/time.h>
#include <librfn/util.h>
//#include "systime.h"
#include "xpt2046.h"

#include "timing_stm32.h"
#include "st7789_stm32_spi.h"
#include "fonts/font_fixedsys_mono_24.h"
#include "fonts/pic.h"
#include "fonts/ext-firm.h"
#include "fonts/bitmap_typedefs.h"
#include "ILI9486_Defines.h"
#include "fonts/font_ubuntu_48.h"
#include <libopencm3/stm32/usart.h>


//#include "include/nrf24l01_regs.h"
//#include "include/nrf24l01_hw.h"
//#include "include/nrf24l01.h"



void adcboot(void);

int usbmode;

void adc_init(void);

//void nrf_dump_regs(nrf_regs *r);

//void tsirq_pin_init(void);
//int tsxor = 1;
volatile int seesaw = 0;
static char ret2[] = "0.0V";
int adcrun = 0;
extern char _ebss[];
uint8_t usbd_control_buffer[256];

#define USART_CONSOLE USART1

#define FW_ADDR    0x08080000

void get_buffered_line(void);
static void usart_setup(void);

static void ulpi_pins(uint32_t gpioport, uint16_t gpiopins);
void gpio2_setup(void);

#define IRQ_TYPE_NONE		0
#define IRQ_TYPE_EDGE_RISING	0x00000001
#define IRQ_TYPE_EDGE_FALLING	0x00000002
#define IRQ_TYPE_EDGE_BOTH	IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING
#define IRQ_TYPE_LEVEL_HIGH	0x00000004
#define IRQ_TYPE_LEVEL_LOW	0x00000008


void platform_init(void)
{

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOE);
	rcc_periph_clock_enable(RCC_GPIOD);

	rcc_clock_setup_hse(&rcc_3v3[RCC_CLOCK_3V3_216MHZ], 8);
    gpio2_setup();

	rcc_periph_clock_enable(RCC_CRC);
	rcc_periph_clock_enable(RCC_USART1);

    
    usart_setup();

//	GPIOC_OSPEEDR &= ~0xF30;
//	GPIOC_OSPEEDR |= 0xA20;
printf("b4 jtag pin setup\n");
	gpio_mode_setup(JTAG_PORT, GPIO_MODE_OUTPUT,
					GPIO_PUPD_NONE,
					TCK_PIN | TDI_PIN);
	gpio_mode_setup(JTAG_PORT, GPIO_MODE_INPUT,
					GPIO_PUPD_NONE, TMS_PIN);
	gpio_set_output_options(JTAG_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
							TCK_PIN | TDI_PIN | TMS_PIN);
	gpio_mode_setup(TDO_PORT, GPIO_MODE_INPUT,
					GPIO_PUPD_NONE,
					TDO_PIN);
	gpio_set_output_options(TDO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
							TDO_PIN | TMS_PIN);

    
    systime_setup(216000);
    delay(100);
 //   rcc_periph_clock_enable(RCC_DMA1);
//    printf("b4 tft setup\n");
//    tftdma();
//    st_init();
//    delay(100);
printf("b4 bmp/usb setup\n");
 	blackmagic_usb_init();
 	printf("b4  usbuart setup\n");
	usbuart_init();
//	i2c_dma_start();
//i2c_init();
//    i2c_init();
//	i2c_setup2();
//    i2c2_init();
//	sendi2ctest();
printf("b4 adc setup\n");
    adc_init();
 //   seesawneoint(24);
    
 //    clearseesaw(24);
/*
    st_fill_screen(0xD69A);
    delay(122);

    st_draw_bitmap(0, 192, &bm16ton);
	st_draw_string(10, 20, "white magic probe", ST_COLOR_BLACK, &font_ubuntu_48);

	st_draw_rectangle(385, 95, 80, 25, ILI9486_RED);
	st_fill_rect(385, 95, 80, 25, ILI9486_RED);
	st_draw_string(385, 95, "ADC", ST_COLOR_YELLOW, &font_fixedsys_mono_24);
	st_draw_rectangle(385, 125, 80, 25, ILI9486_RED);
	st_fill_rect(385, 125, 80, 25, ILI9486_RED);
	st_draw_string(385, 125, "BMP", ST_COLOR_PURPLE, &font_fixedsys_mono_24);
	st_draw_rectangle(385, 155, 80, 25, ILI9486_RED);
	st_fill_rect(385, 155, 80, 25, ILI9486_RED);
	st_draw_string(385, 155, "i2c_ex", ST_COLOR_YELLOW, &font_fixedsys_mono_24);
	st_draw_rectangle(385, 185, 80, 25, ILI9486_RED);
	st_fill_rect(385, 185, 80, 25, ILI9486_RED);
	st_draw_string(385, 185, "can bus", ST_COLOR_YELLOW, &font_fixedsys_mono_24);
    
//    spiflash_setup();
    printf("b4 jts irq pin setup\n");
    tsirq_pin_init();

	printf("Booted BMP\n");
*/
//    neoup(0x18, 0xa, 0xff, 0xc);
//    neoeveryother(0x00, 0x09, 0x7D, 0x00, 0x7D, 0x09);
//    neoeveryother(0x00, 0x09, 0x7D, 0x00, 0x7D, 0x09);
//    neoeveryother(0x00, 0x09, 0x7D, 0x00, 0x7D, 0x09);
//    delay(122);
//    neodown(0x18, 0x0, 0x50, 0x0);
//    delay(100);
//    sendi2ctest();
//    delay(200);
//    neotimodd();
//    neoeveryother(0xff, 0x0, 0xff, 0x0, 0xff, 0x0);
// 	while (1) {
// 		usbd_poll(usbdev);
// 	}
}


static void ulpi_pins(uint32_t gpioport, uint16_t gpiopins)
{
	gpio_mode_setup(gpioport, GPIO_MODE_AF, GPIO_PUPD_NONE, gpiopins);
	gpio_set_output_options(gpioport, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, gpiopins);
	gpio_set_af(gpioport, GPIO_AF10, gpiopins);
}

void gpio2_setup(void) {
	ulpi_pins(GPIOA, GPIO3 | GPIO5);
	ulpi_pins(GPIOB, GPIO0 | GPIO1 | GPIO5  | GPIO10 | GPIO11 | GPIO12 | GPIO13);
	ulpi_pins(GPIOC, GPIO0);
	ulpi_pins(GPIOC, GPIO2);
	ulpi_pins(GPIOC, GPIO3);
}


int _write(int file, char *ptr, int len)
{
	int i;

	if (file == STDOUT_FILENO || file == STDERR_FILENO) {
		for (i = 0; i < len; i++) {
			if (ptr[i] == '\n') {
				usart_send_blocking(USART1, '\r');
			}
			usart_send_blocking(USART1, ptr[i]);
		}
		return i;
	}
	errno = EIO;
	return -1;
}
/*
static void i2c_setup2(void) {
  // Set alternate functions for the SCL and SDA pins of I2C1. 
	// GPIO for I2C1 


	rcc_periph_clock_enable(RCC_I2C2);
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO10);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ,
				GPIO10);
	gpio_set_af(GPIOB, GPIO_AF4, GPIO10);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO11);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ,
				GPIO11);
	gpio_set_af(GPIOB, GPIO_AF4, GPIO11);

	i2c_peripheral_disable(I2C2);
	i2c_set_clock_frequency(I2C2, 30);
	i2c_set_ccr(I2C2, 30 * 5);
	i2c_set_trise(I2C2, 30 + 1);
	i2c_enable_ack(I2C2);
	i2c_set_own_7bit_slave_address(I2C2, 0x32);
//	i2c_set_fast_mode(I2C2);
    i2c_set_standard_mode(I2C2);
	i2c_peripheral_enable(I2C2);

  for (uint32_t loop = 0; loop < 1000000; ++loop) {
    __asm__("nop");
  }
}  
*/

static void usart_setup(void)
{
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO9);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, 
	                GPIO10); 
	gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, 
					GPIO_OSPEED_100MHZ, GPIO10); 
	gpio_set_af(GPIOA, GPIO_AF7, GPIO10); 
	
	usart_set_baudrate(USART_CONSOLE, 115200);
	usart_set_databits(USART_CONSOLE, 8);
	usart_set_stopbits(USART_CONSOLE, USART_STOPBITS_1);
	usart_set_mode(USART_CONSOLE, USART_MODE_TX_RX);
	usart_set_parity(USART_CONSOLE, USART_PARITY_NONE);
	usart_set_flow_control(USART_CONSOLE, USART_FLOWCONTROL_NONE);
	usart_enable(USART_CONSOLE);
}

void tsirq_pin_init(void)
{
	delay(100);
    nvic_enable_irq(NVIC_EXTI1_IRQ);					
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO1);
	delay(100);
	exti_select_source(EXTI1, GPIO1);
    exti_set_trigger(EXTI1, IRQ_TYPE_EDGE_FALLING);
	exti_enable_request(EXTI1);
}

void exti1_isr(void)
{

    volatile int xrw;
    volatile int yrw;
    volatile int xraw;
    volatile int yraw;
    volatile int16_t thr;
//    volatile int zraw1;
//    volatile int zraw2;

//    uint32_t *magic = (uint32_t *)&_ebss;
    xrw = ts_get_x();
    yrw = ts_get_y();
    thr = threshholdv();
    xraw = ts_get_x_raw();

    yraw = ts_get_y_raw();
    printf("threshhold =  %d\n", thr);
    printf("xraw =  %d\n", xraw);
    printf("yraw =  %d\n", yraw);
    printf("xrw =  %d\n", xrw);
    printf("yrw =  %d\n", yrw);

    

    if ((xraw >= 700 && xraw <= 800 ) && (yraw >= 400 && yraw <= 550)) {
printf("first button \n");
          
    }
    if ((xraw >= 850 && xraw <= 980 ) && (yraw >= 350 && yraw <= 450)) {
            printf("second button \n");
    }
    
    if ((xraw >= 500 && xraw <= 600 ) && (yraw >= 2000 && yraw <= 2050)) {

printf("third button \n");
 
//    platform_request_boot3();
     }
    exti_reset_request(EXTI1);
    exti_set_trigger(EXTI1, IRQ_TYPE_EDGE_FALLING);
       
}

void platform_nrst_set_val(bool assert) { (void)assert; }
bool platform_nrst_get_val(void) { return false; }


void adc_init(void)
{
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO2);
	rcc_periph_clock_enable(RCC_ADC1);
	adc_power_off(ADC1);
	adc_disable_scan_mode(ADC1);
	adc_set_sample_time(ADC1, ADC_CHANNEL2, ADC_SMPR_SMP_3CYC);
	adc_power_on(ADC1);
}



const char *platform_target_voltage(void)
{
	static char ret[] = "0.0V";
	
	uint8_t channels[] = { ADC_CHANNEL2, };
	unsigned value;

	adc_set_regular_sequence(ADC1, 1, channels);
	adc_start_conversion_regular(ADC1);
	while (!adc_eoc(ADC1));
	value = adc_read_regular(ADC1);

	value *= 3379; /* 3.3 * 1024 == 3379.2 */
	value += 104858; /* round, 0.05V * 2 ^ 21 == 104857.6 */
	ret[0] = (value >> 21) + '0';
	value &= (1 << 21) - 1;
	value *= 10;
	ret[2] = (value >> 21) + '0';

   strcpy(ret2, ret);

	return ret;
}


void platform_request_boot(void)
{
	uint32_t *magic = (uint32_t *)&_ebss;
	st_fill_screen(ILI9486_BLACK);
	st_draw_string_withbg(90, 110, "DFU FIRMWARE UPGRADE", ST_COLOR_RED, ST_COLOR_WHITE, &font_fixedsys_mono_24);
	usart_disable(USART_CONSOLE);
	rcc_periph_clock_disable(RCC_USART1);
//	gpio_clear(GPIOA, GPIO9);
	GPIOA_MODER |= (0x00000000);
	delay(122);
	magic[0] = BOOTMAGIC0;
	magic[1] = BOOTMAGIC1;
	scb_reset_system();
}


