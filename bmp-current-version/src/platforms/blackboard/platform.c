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
#include <usb_i2c.h>
#include <usb_gpio.h>
#include <usb_adc.h>
#include <altusb.h>
#include <u8x8.h>
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
#include <libopencm3/usb/dwc/otg_fs.h>
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

#include "ws2812_spi.h"
#include "timing_stm32.h"
#include "st7789_stm32_spi.h"
#include "fonts/font_fixedsys_mono_24.h"
#include "fonts/pic.h"
//#include "fonts/badger.h"
//#include "fonts/img_flag.h"
#include "fonts/bitmap_typedefs.h"
#include "ILI9486_Defines.h"
#include "fonts/font_ubuntu_48.h"
#include <libopencm3/stm32/usart.h>

#define IRQ_TYPE_NONE		0
#define IRQ_TYPE_EDGE_RISING	0x00000001
#define IRQ_TYPE_EDGE_FALLING	0x00000002
#define IRQ_TYPE_EDGE_BOTH	IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING
#define IRQ_TYPE_LEVEL_HIGH	0x00000004
#define IRQ_TYPE_LEVEL_LOW	0x00000008

void adcboot(void);
void platform_request_boot2(void);
int usbmode;
void neopixel_init(void);
void adc_init(void);
static void tsirq_pin_init(void);
//int tsxor = 1;

static char ret2[] = "0.0V";
int adcrun = 0;
jmp_buf fatal_error_jmpbuf;
extern char _ebss[];
uint8_t usbd_control_buffer[256];

#define USART_CONSOLE USART1

int _write(int file, char *ptr, int len);
int _read(int file, char *ptr, int len);
void get_buffered_line(void);
static void usart_setup(void);

void platform_init(void)
{
	volatile uint32_t *magic = (uint32_t *)_ebss;
	/* Enable GPIO peripherals */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOE);
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT,
					GPIO_PUPD_PULLUP, GPIO0);
	gpio_mode_setup(GPIOE, GPIO_MODE_INPUT,
					GPIO_PUPD_PULLUP, GPIO4);
	/* Check the USER button*/
	if (!gpio_get(GPIOA, GPIO0) ||
		((magic[0] == BOOTMAGIC0) && (magic[1] == BOOTMAGIC1)))
	{
		magic[0] = 0;
		magic[1] = 0;
		/* Assert blue LED as indicator we are in the bootloader */
		gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT,
						GPIO_PUPD_NONE, LED_BOOTLOADER);
		gpio_set(LED_PORT, LED_BOOTLOADER);
		/* Jump to the built in bootloader by mapping System flash.
		   As we just come out of reset, no other deinit is needed!*/
		rcc_periph_clock_enable(RCC_SYSCFG);
		SYSCFG_MEMRM &= ~3;
		SYSCFG_MEMRM |= 1;
		scb_reset_core();
	}
/*	
	if ((magic[0] == BOOTMAGIC4) && (magic[1] == BOOTMAGIC5))
	{

//	delay(122);
 //   rcc_peripheral_reset(&RCC_APB1ENR, RCC_APB1RSTR);
 //   rcc_peripheral_reset(&RCC_AHB2ENR, RCC_APB2RSTR);
	magic[0] = BOOTMAGIC2;
	magic[1] = BOOTMAGIC3;
//	reset_handler();
	scb_reset_system();
	
	}
*/	
	if (!gpio_get(GPIOE, GPIO4) || ((magic[0] == BOOTMAGIC2) && (magic[1] == BOOTMAGIC3))) 
	{
		magic[0] = 0;
		magic[1] = 0;
	rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT,
			GPIO_PUPD_NONE, GPIO6);
	/* Enable peripherals */
    SCB_VTOR = (uint32_t) 0x08000000;
	rcc_periph_clock_enable(RCC_OTGFS);
	rcc_periph_clock_enable(RCC_CRC);
	rcc_periph_clock_enable(RCC_USART1);
//    rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
			GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);

    usbmode = 1;
	/* Set up USB Pins and alternate function*/
	
	usart_setup();
    i2c_init();
    systime_setup(168000);
    
	usbgpio_init();
    blackmagic_usb_init(1);
	
//	time_init();
    
//	clock_setup();
   
//	adc_init();

//	irq_pin_init();
//	pwm_probe();
//	gpio_clear(GPIOC, GPIO13);
    gpio_clear(GPIOA, GPIO6);


    
    adc_start();
    neopixel_init();
	/* prepare the scheduler */
//	fibre_run(&usb_task);

//	fibre_scheduler_main_loop();
//    ulib8run2();
    st_init();
          
    st_fill_screen(ST_COLOR_YELLOW);

	st_draw_string_withbg(10, 5, "16ton presents", ST_COLOR_RED, ST_COLOR_PURPLE, &font_ubuntu_48);
//	st_draw_string(10, 50, "white magic probe", ST_COLOR_NAVY, &font_fixedsys_mono_24);
    st_draw_string(10, 69, "usb adc i2c", ST_COLOR_RED, &font_ubuntu_48);
    st_draw_bitmap(352, 192, &bm16ton);
    st_draw_rectangle(385, 95, 80, 25, ILI9486_RED);
	st_fill_rect(385, 95, 80, 25, ILI9486_RED);
	st_draw_string(385, 95, "ADC", ST_COLOR_PURPLE, &font_fixedsys_mono_24);
	st_draw_rectangle(385, 125, 80, 25, ILI9486_RED);
	st_fill_rect(385, 125, 80, 25, ILI9486_RED);
	st_draw_string(385, 125, "BMP", ST_COLOR_YELLOW, &font_fixedsys_mono_24);
	OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS | OTG_GCCFG_PWRDWN;
	OTG_FS_GCCFG &= ~(OTG_GCCFG_VBUSBSEN | OTG_GCCFG_VBUSASEN);
    tsirq_pin_init();
//    		st_draw_string(10, 120, rcc_get_spi_clk_freq(SPI3), 0xffff, &font_fixedsys_mono_24);
// 	OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS | OTG_GCCFG_PWRDWN;
//	OTG_FS_GCCFG &= ~(OTG_GCCFG_VBUSBSEN | OTG_GCCFG_VBUSASEN);
	printf("Booted BMP\n");
//	return;
	} else {
	rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
   int sspeed;
    int sspeed2;
	/* Enable peripherals */
	rcc_periph_clock_enable(RCC_OTGFS);
	rcc_periph_clock_enable(RCC_CRC);
	rcc_periph_clock_enable(RCC_USART1);
    usbmode = 2;
    usart_setup();
	/* Set up USB Pins and alternate function*/
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);

	GPIOC_OSPEEDR &= ~0xF30;
	GPIOC_OSPEEDR |= 0xA20;

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
/*
	gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT,
					GPIO_PUPD_NONE,
					LED_IDLE_RUN | LED_ERROR | LED_BOOTLOADER);

	gpio_mode_setup(LED_PORT_UART, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_UART);

 */ 
//    i2c_init();
//	platform_timing_init();
//    usbgpio_init();

    systime_setup(168000);
 	blackmagic_usb_init(0);
	usbuart_init();
	
    adc_init();
//    adc_start();
    neopixel_init();
//    ulib8run();
    st_init();
    delay(10);
//    tsirq_pin_init();
    st_fill_screen(0xD69A);
    delay(122);
//    char test;
//    ret2[0] = (char)rcc_get_spi_clk_freq(SPI3);
//    st_set_address_window(0, 0, TFT_WIDTH - 1, TFT_HEIGHT - 1);
//	st_draw_string_withbg(10, 2, "16ton presents", ST_COLOR_YELLOW, ST_COLOR_BLACK, &font_fixedsys_mono_24);
    st_draw_bitmap(352, 192, &bm16ton);
	st_draw_string(10, 20, "white magic probe", ST_COLOR_BLACK, &font_ubuntu_48);
//		st_draw_string(10, 120, ret2, 0xffff, &font_fixedsys_mono_24);
	// left lines lft/right updwn then rightvlines lft/rght up/dwn 
	st_draw_rectangle(385, 95, 80, 25, ILI9486_RED);
	st_fill_rect(385, 95, 80, 25, ILI9486_RED);
	st_draw_string(385, 95, "ADC", ST_COLOR_YELLOW, &font_fixedsys_mono_24);
	st_draw_rectangle(385, 125, 80, 25, ILI9486_RED);
	st_fill_rect(385, 125, 80, 25, ILI9486_RED);
	st_draw_string(385, 125, "BMP", ST_COLOR_PURPLE, &font_fixedsys_mono_24);
//	st_fill_screen(ST_COLOR_YELLOW);
    tsirq_pin_init();
//	sspeed = rcc_get_spi_clk_freq(SPI1);
//	sspeed2 = rcc_get_spi_clk_freq(SPI2);
	OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS | OTG_GCCFG_PWRDWN;
	OTG_FS_GCCFG &= ~(OTG_GCCFG_VBUSBSEN | OTG_GCCFG_VBUSASEN);
	printf("Booted BMP\n");
//	printf("spi2 clock speed %d\n", sspeed2);
	}
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

	/* Finally enable the USART. */
	usart_enable(USART_CONSOLE);
}

static void tsirq_pin_init(void)
{
	delay(100);
    nvic_enable_irq(NVIC_EXTI1_IRQ);					
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO1);
	delay(100);
	exti_select_source(EXTI1, GPIO1);
    exti_set_trigger(EXTI1, IRQ_TYPE_EDGE_FALLING);
	exti_enable_request(EXTI1);
}

void wdg(void) {
            uint32_t *magic = (uint32_t *)&_ebss;
            magic[0] = BOOTMAGIC2;
	        magic[1] = BOOTMAGIC3;
//	        iwdg_set_period_ms(2000);
	        iwdg_start();
}

void exti1_isr(void)
{
//	exti_reset_request(EXTI1);
    volatile int xrw;
    volatile int yrw;
    volatile int xraw;
    volatile int yraw;
    volatile int16_t thr;
//    volatile int zraw1;
//    volatile int zraw2;

    uint32_t *magic = (uint32_t *)&_ebss;
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
//    zraw1 = ts_get_z1_raw();
//    zraw2 = ts_get_z2_raw();
    

    if ((xraw >= 700 && xraw <= 800 ) && (yraw >= 400 && yraw <= 550)) {
//            exti_disable_request(EXTI1);
//            usart_disable(USART_CONSOLE);
//        	rcc_periph_clock_disable(RCC_USART1);
//	gpio_clear(GPIOA, GPIO9);
//        	GPIOA_MODER |= (0x00000000);
         
//            exti_reset_request(EXTI1);
//           nvic_disable_irq(NVIC_EXTI1_IRQ);
//	cm_disable_interrupts();
	
//	usart_disable(USART_CONSOLE);
//	rcc_periph_clock_disable(RCC_USART1);
//	gpio_clear(GPIOA, GPIO9);
//	GPIOA_MODER |= (0x00000000);
           magic[0] = BOOTMAGIC2;
           magic[1] = BOOTMAGIC3;
//	        setpwoff();
//	        return;
//	        rcc_periph_clock_disable(RCC_USART1);
//	        rcc_periph_clock_disable(SPI2_BASE);
	        GPIOA_MODER |= (0x00000000);
//	        GPIOB_MODER |= (0x00000000);
            scb_reset_system();
            scb_reset_core();
//            platform_request_boot2();
           
 //           wdg();
            return;
          
    }
    if ((xraw >= 850 && xraw <= 980 ) && (yraw >= 350 && yraw <= 450)) {
            scb_reset_system();
            platform_request_boot2();
    }
//    printf("x coord =  %d\n", xrw);
//    printf("y coord =  %d\n", yrw);
//    printf("xraw =  %d\n", xraw);
//    printf("yraw =  %d\n", yraw);
   
 

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
	/* On the stlinkv3, the target input voltage is divided by two.
	 * The ADC is sampling at 12 bit resolution.
	 * Vref+ input is assumed to be 3.3 volts. */
	
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

   st_fill_rect(1, 110, 74, 60, ILI9486_LIGHTGREY);
   st_draw_string(10, 65, "Target IO", ILI9486_DARKGREEN, &font_ubuntu_48);
	st_draw_string(10, 110, ret2, ST_COLOR_RED, &font_ubuntu_48);
	return ret;
}


void platform_request_boot2(void)
{
	uint32_t *magic = (uint32_t *)&_ebss;
	st_fill_screen(ILI9486_GREEN);
//	st_draw_string_withbg(90, 110, "DFU FIRMWARE UPGRADE", ST_COLOR_RED, ST_COLOR_WHITE, &font_fixedsys_mono_24);
	usart_disable(USART_CONSOLE);
	rcc_periph_clock_disable(RCC_USART1);
	rcc_periph_clock_disable(RCC_USART3);
    cm_disable_interrupts();
	usart_disable(USART_CONSOLE);
//	rcc_periph_clock_disable(SPI2);
//	gpio_clear(GPIOA, GPIO9);
	GPIOA_MODER |= (0x00000000);
	GPIOB_MODER |= (0x00000000);
	GPIOC_MODER |= (0x00000000);
	GPIOD_MODER |= (0x00000000);
	delay(122);
	magic[0] = BOOTMAGIC2;
	magic[1] = BOOTMAGIC3;
	scb_reset_system();
//	POO = (0x5FA << 16) |
//	SCB_AIRCR_SYSRESETREQ;
//	    while (1==1);
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

void neopixel_init(void)
{
 //   systime_setup(84000);
//    systime_setup(84000);
//	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_SPI1);

	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO5);
	gpio_set_af(GPIOB, GPIO_AF5, GPIO5);

	// use PIN B5 as ws2812 open drain output (1K pullup -> +5V)
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO5);
	ws2812_init(SPI1);
    delay_ms(15);
//	int h = 0;
//	while (1) {
//		h = (h+1) % 360;
        if (usbmode == 1) {
		    ws2812_write_rgb(SPI1, 220, 0, 220);
		}
		if (usbmode == 2) {
		    ws2812_write_rgb(SPI1, 0, 220, 220);
		}
//		ws2812_write_hsv(SPI1, 120, 10, 1);
//		ws2812_write_hsv(SPI1, 120, 10, 1);
		delay_ms(15);
//    }

}

