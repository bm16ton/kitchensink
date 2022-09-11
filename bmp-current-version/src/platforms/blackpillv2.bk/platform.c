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

#include "general.h"
#include "usb.h"
#include "usbuart.h"
#include "morse.h"
#include <usb_i2c.h>
#include <usb_gpio.h>
#include <usb_adc.h>
#include <altusb.h>
#include <u8x8.h>

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
#include <librfn/fibre.h>
#include <librfn/time.h>
#include <librfn/util.h>
//#include "systime.h"
#include "ws2812_spi.h"
#include "timing_stm32.h"
#include "st7789_stm32_spi.h"
#include "fonts/font_fixedsys_mono_24.h"

int usbmode;
void neopixel_init(void);
void adc_init(void);
// void u8log_Init(u8log_t *u8log, uint8_t width, uint8_t height, uint8_t *buf);
//static uint8_t u8x8_gpio_and_delay_cm3(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
//static uint8_t u8x8_byte_hw_i2c_cm3(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
//static void i2c_setup2(void);
//void ulib8run(void);
//void ulib8run2(void);
static char ret2[] = "0.0V";
int adcrun = 0;
jmp_buf fatal_error_jmpbuf;
extern char _ebss[];
uint8_t usbd_control_buffer[256];
u8x8_t u8x8_i, *u8x8 = &u8x8_i;
void clock_setup(void);
/*
static int usb_fibre(fibre_t *fibre)
{
	PT_BEGIN_FIBRE(fibre);
    SCB_VTOR = (uint32_t) 0x08000000;
	rcc_periph_clock_enable(RCC_OTGFS);

	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
			GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);

	usbdev = usbd_init(&otgfs_usb_driver, &dev, &config2,
//    usbd_dev = usbd_init(&stm32f107_usb_driver, &dev, &config,
			usb2_strings, sizeof(usb2_strings)/sizeof(char *),
			usbd_control_buffer, sizeof(usbd_control_buffer));
	OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS;
//    OTG_FS_GUSBCFG |= OTG_GUSBCFG_FDMOD | OTG_GUSBCFG_TRDT_MASK;
//	OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS | OTG_GCCFG_PWRDWN;
//	OTG_FS_GCCFG &= ~(OTG_GCCFG_VBUSBSEN | OTG_GCCFG_VBUSASEN);
//	usbd_register_set_config_callback(usbdev, usb_set_config);
	usbd_register_set_config_callback(usbdev, usb_set_config);
	usbd_register_set_config_callback(usbdev, gpio_set_config);
	usbd_register_set_config_callback(usbdev, usbadc_set_config);
//    usbuart_init();

	while (true) {
		usbd_poll(usbdev);
		PT_YIELD();
	}

	PT_END();
}
static fibre_t usb_task = FIBRE_VAR_INIT(usb_fibre);
*/

void platform_init(void)
{
	volatile uint32_t *magic = (uint32_t *)_ebss;
	/* Enable GPIO peripherals */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOB);
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT,
					GPIO_PUPD_PULLUP, GPIO0);
	gpio_mode_setup(GPIOC, GPIO_MODE_INPUT,
					GPIO_PUPD_PULLUP, GPIO15);
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
	
		if (!gpio_get(GPIOC, GPIO15))
	{
	
	rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
    
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT,
			GPIO_PUPD_NONE, GPIO13);
	/* Enable peripherals */
    SCB_VTOR = (uint32_t) 0x08000000;
	rcc_periph_clock_enable(RCC_OTGFS);
	rcc_periph_clock_enable(RCC_CRC);
//    rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
			GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);

    usbmode = 1;
	/* Set up USB Pins and alternate function*/
	
	
    i2c_init();
    systime_setup(84000);
    
	usbgpio_init();
    blackmagic_usb_init(1);
	
//	time_init();
    
//	clock_setup();
   
//	adc_init();

//	irq_pin_init();
//	pwm_probe();
//	gpio_clear(GPIOC, GPIO13);
    gpio_clear(GPIOC, GPIO13);


    
    adc_start();
    neopixel_init();
	/* prepare the scheduler */
//	fibre_run(&usb_task);

//	fibre_scheduler_main_loop();
//    ulib8run2();
    st_init();
          
    st_fill_screen(ST_COLOR_YELLOW);

	st_draw_string_withbg(10, 5, "16ton presents", ST_COLOR_RED, ST_COLOR_PURPLE, &font_fixedsys_mono_24);
	st_draw_string(10, 50, "white magic probe", ST_COLOR_NAVY, &font_fixedsys_mono_24);
    st_draw_string(10, 100, "usb adc i2c", ST_COLOR_RED, &font_fixedsys_mono_24);
 	OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS | OTG_GCCFG_PWRDWN;
	OTG_FS_GCCFG &= ~(OTG_GCCFG_VBUSBSEN | OTG_GCCFG_VBUSASEN);
	
//	return;
	} else {
	rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
    
	/* Enable peripherals */
	rcc_periph_clock_enable(RCC_OTGFS);
	rcc_periph_clock_enable(RCC_CRC);
    usbmode = 2;
	/* Set up USB Pins and alternate function*/
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);

	GPIOA_OSPEEDR &= 0x3C00000C;
	GPIOA_OSPEEDR |= 0x28000008;

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

	gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT,
					GPIO_PUPD_NONE,
					LED_IDLE_RUN | LED_ERROR | LED_BOOTLOADER);

	gpio_mode_setup(LED_PORT_UART, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_UART);

    
//    i2c_init();
//	platform_timing_init();
//    usbgpio_init();
    systime_setup(84000);
	blackmagic_usb_init(0);
	usbuart_init();
	
    adc_init();
//    adc_start();
    neopixel_init();
//    ulib8run();
    st_init();
    
    st_fill_screen(ST_COLOR_NAVY);

	st_draw_string_withbg(10, 2, "16ton presents", ST_COLOR_YELLOW, ST_COLOR_BLACK, &font_fixedsys_mono_24);
	st_draw_string(10, 100, "white magic probe", 0xffff, &font_fixedsys_mono_24);
	// left lines lft/right updwn then rightvlines lft/rght up/dwn 
//	st_draw_rectangle(240, 15, 80, 25, ST_COLOR_YELLOW);
	
	// https://github.com/libopencm3/libopencm3/pull/1256#issuecomment-779424001
	OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS | OTG_GCCFG_PWRDWN;
	OTG_FS_GCCFG &= ~(OTG_GCCFG_VBUSBSEN | OTG_GCCFG_VBUSASEN);
	}
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

void platform_nrst_set_val(bool assert) { (void)assert; }
bool platform_nrst_get_val(void) { return false; }

/*
static uint8_t u8x8_gpio_and_delay_cm3(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
	switch(msg) {
	case U8X8_MSG_GPIO_AND_DELAY_INIT:
		i2c_setup2();  // Init I2C communication 
		break;

	default:
		u8x8_SetGPIOResult(u8x8, 1);
		break;
	}

	return 1;
}
*/
/*
// I2C hardware transfer based on u8x8_byte.c implementation 
static uint8_t u8x8_byte_hw_i2c_cm3(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
	static uint8_t buffer[32];   // u8g2/u8x8 will never send more than 32 bytes 
	static uint8_t buf_idx;
	uint8_t *data;

	switch(msg) {
	case U8X8_MSG_BYTE_SEND:
		data = (uint8_t *)arg_ptr;
		while(arg_int > 0) {
			buffer[buf_idx++] = *data;
			data++;
			arg_int--;
		}
		break;
	case U8X8_MSG_BYTE_INIT:
		break;
	case U8X8_MSG_BYTE_SET_DC:
		break;
	case U8X8_MSG_BYTE_START_TRANSFER:
		buf_idx = 0;
		break;
	case U8X8_MSG_BYTE_END_TRANSFER:
		i2c_transfer7(I2C2, 0x3C, buffer, buf_idx, NULL, 0);
		break;
	default:
		return 0;
	}
	return 1;
}
*/
/*
static void i2c_setup2(void) {
  // Set alternate functions for the SCL and SDA pins of I2C1. 
	// GPIO for I2C1 
//	u8x8_t u8x8_i, *u8x8 = &u8x8_i;
	rcc_periph_clock_enable(RCC_GPIOB);

	rcc_periph_clock_enable(RCC_I2C2);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO10 | GPIO3);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ,
				GPIO10 | GPIO3);
	gpio_set_af(GPIOB, GPIO_AF4, GPIO10);
    gpio_set_af(GPIOB, GPIO_AF9, GPIO3);

	i2c_peripheral_disable(I2C2);
	i2c_set_clock_frequency(I2C2, 30);
	i2c_set_ccr(I2C2, 30 * 5);
	i2c_set_trise(I2C2, 30 + 1);
	i2c_enable_ack(I2C2);
	i2c_set_own_7bit_slave_address(I2C2, 0x32);
	i2c_set_fast_mode(I2C2);
//    i2c_set_standard_mode(I2C1);
	i2c_peripheral_enable(I2C2);

  for (uint32_t loop = 0; loop < 1000000; ++loop) {
    __asm__("nop");
  }
}  

void ulib8run(void) {

	u8x8_Setup(u8x8, u8x8_d_sh1106_128x64_noname, u8x8_cad_ssd13xx_fast_i2c, u8x8_byte_hw_i2c_cm3, u8x8_gpio_and_delay_cm3);
    i2c_setup2();
      for (uint32_t loop = 0; loop < 800000; ++loop) {
    __asm__("nop");
    }
	u8x8_InitDisplay(u8x8);
	u8x8_SetPowerSave(u8x8,0);
	u8x8_SetFont(u8x8, u8x8_font_7x14B_1x2_f);
//    u8x8_SetFont(u8x8, u8x8_font_open_iconic_embedded_2x2);
	u8x8_ClearDisplay(u8x8);
	u8x8_DrawString(u8x8, 1,1, "16ton presents");
//	u8x8_Draw2x2Glyph(u8x8, 0,0, 'H');
	u8x8_SetInverseFont(u8x8, 1);
	u8x8_DrawString(u8x8, 0,5, "BlackMagic");
	u8x8_SetInverseFont(u8x8, 0);
    u8x8_SetFont(u8x8, u8x8_font_7x14B_1x2_f); 
//	u8x8_SetFont(u8x8, u8x8_font_open_iconic_embedded_2x2);
//	u8x8_DrawGlyph(u8x8, 11,1, 65); // Bell 
 //   u8x8_ClearDisplay(u8x8);
}




void ulib8run2(void) {

	u8x8_Setup(u8x8, u8x8_d_sh1106_128x64_noname, u8x8_cad_ssd13xx_fast_i2c, u8x8_byte_hw_i2c_cm3, u8x8_gpio_and_delay_cm3);
	delay_ms(15);
    i2c_setup2();
      for (uint32_t loop = 0; loop < 800000; ++loop) {
    __asm__("nop");
    }
	u8x8_InitDisplay(u8x8); 

	u8x8_SetPowerSave(u8x8,0);

	u8x8_SetFont(u8x8, u8x8_font_7x14B_1x2_f);
//    u8x8_SetFont(u8x8, u8x8_font_open_iconic_embedded_2x2);
	u8x8_ClearDisplay(u8x8);
	u8x8_DrawString(u8x8, 1,1, "16ton presents");
//	u8x8_Draw2x2Glyph(u8x8, 0,0, 'H');
	u8x8_SetInverseFont(u8x8, 1);
	u8x8_DrawString(u8x8, 0,5, "USB ADC");
	u8x8_SetInverseFont(u8x8, 0);
    u8x8_SetFont(u8x8, u8x8_font_7x14B_1x2_f); 
//	u8x8_SetFont(u8x8, u8x8_font_open_iconic_embedded_2x2);
//	u8x8_DrawGlyph(u8x8, 11,1, 65); // Bell 
//    u8x8_ClearDisplay(u8x8);
//    printBits(1, "poooop");
}
*/
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
//   adcrun = 1;
//    u8x8_ClearDisplay(u8x8);
   
//	u8x8_DrawString(u8x8, 1,1, ret2);
	
	
	return ret;
}

int yline = 1;
//static char banner1[] = " USB to ADC              ";
//static char banner2[] = " Blackmagic              ";

/*
void lcdshow(void) {
//if (adcrun == 1) {
//    u8x8_t u8x8_i, *u8x8 = &u8x8_i;
//    u8x8_Setup(u8x8, u8x8_d_sh1106_128x64_noname, u8x8_cad_ssd13xx_fast_i2c, u8x8_byte_hw_i2c_cm3, u8x8_gpio_and_delay_cm3);
//    u8x8_InitDisplay(u8x8);
//	u8x8_SetPowerSave(u8x8,0);
    yline = yline + 1;
    if (yline == 16)
         yline = 1;
//	u8x8_SetFont(u8x8, u8x8_font_7x14B_1x2_f);
    if (usbmode == 1) {
    u8x8_ClearDisplay(u8x8);
	u8x8_DrawString(u8x8, yline,3, banner1);
    }

    if (usbmode == 2) {   
	u8x8_ClearDisplay(u8x8);
	u8x8_DrawString(u8x8, yline,3, banner2);
//	adcrun = 0;
    }
}
*/
void platform_request_boot(void)
{
	uint32_t *magic = (uint32_t *)&_ebss;
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

