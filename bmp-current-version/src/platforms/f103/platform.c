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

/* This file implements the platform specific functions for the ST-Link
 * implementation.
 */

#include "general.h"
#include "usb.h"
#include "usbuart.h"
//#include "list.h"
//#include "messageq.h"
//#include "util.h"
#include "st7789_stm32_spi.h"
#include "fonts/font_fixedsys_mono_24.h"
/*
#include <librfn/fibre.h>
#include <librfn/time.h>
#include <librfn/util.h>
#include <librfm3/i2c_ctx.h>
*/
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/scs.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/stm32/adc.h>
#include <usb_i2c.h>
#include <usb_gpio.h>
#include <usb_adc.h>

uint16_t led_idle_run;
uint16_t nrst_pin;
static uint32_t rev;
static void adc_init(void);

int platform_hwversion(void)
{
	return rev;
}
/*
static int usb_fibre(fibre_t *fibre)
{
	static uint32_t t;

	PT_BEGIN_FIBRE(fibre);

	rcc_periph_clock_enable(RCC_GPIOA);



	while (true) {
		usbd_poll(usbdev);
//		adc_once();
		PT_YIELD();
	}

	PT_END();
}
static fibre_t usb_task = FIBRE_VAR_INIT(usb_fibre);
*/
void platform_init(void)
{
//	rev = detect_rev();
	SCS_DEMCR |= SCS_DEMCR_VC_MON_EN;
	rcc_periph_clock_enable(RCC_GPIOC);
#ifdef ENABLE_DEBUG
	void initialise_monitor_handles(void);
	initialise_monitor_handles();
#endif
	gpio_set_mode(GPIOC, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_FLOAT, GPIO2);
//    GPIO_CNF_INPUT_PULL_UPDOWN, GPIO2);
		gpio_clear(GPIOC, GPIO2);
//    gpio_set(GPIOC, GPIO2);
	if (!gpio_get(GPIOC, GPIO2)) {
	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_CRC);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    systime_setup(72000);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO0); 
	gpio_set(GPIOA, GPIO0);
    st_init();
    i2c_init_f103();
/*     	
	st_fill_screen(ST_COLOR_ORANGE);

	st_draw_string_withbg(10, 5, "16ton presents", ST_COLOR_CYAN, ST_COLOR_BLACK, &font_fixedsys_mono_24);
	st_draw_string(5, 50, "white magic probe", 0xffff, &font_fixedsys_mono_24);
    st_draw_string(10, 100, "stlink swim", 0xffff, &font_fixedsys_mono_24);

*/
    adc_start();
	blackmagic_usb_init(1);

	usbgpio_init();
  

        
     	st_fill_screen(ST_COLOR_BLACK);

	st_draw_string_withbg(10, 5, "16ton presents", ST_COLOR_YELLOW, ST_COLOR_BLACK, &font_fixedsys_mono_24);
	st_draw_string(5, 50, "white magic probe", 0xffff, &font_fixedsys_mono_24);
    st_draw_string(10, 100, "usb adc i2c", 0xffff, &font_fixedsys_mono_24);

//    st_draw_rectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

} else {     	
     		rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
     	
	/* Setup GPIO ports */
	gpio_set_mode(TMS_PORT, GPIO_MODE_OUTPUT_2_MHZ,
	              GPIO_CNF_INPUT_FLOAT, TMS_PIN);
	gpio_set_mode(TCK_PORT, GPIO_MODE_OUTPUT_2_MHZ,
	              GPIO_CNF_OUTPUT_PUSHPULL, TCK_PIN);
	gpio_set_mode(TDI_PORT, GPIO_MODE_OUTPUT_2_MHZ,
	              GPIO_CNF_OUTPUT_PUSHPULL, TDI_PIN);

	platform_nrst_set_val(false);

	gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ,
	              GPIO_CNF_OUTPUT_PUSHPULL, led_idle_run);

	/* Relocate interrupt vector table here */
//	extern int vector_table;
//	SCB_VTOR = (uint32_t)&vector_table;

//	platform_timing_init();

    systime_setup(72000);

	blackmagic_usb_init(0);

//    i2c_init_f103();


    adc_init();
     
    st_init();
	//rotating display to potrait mode
	//st_rotate_display(1);

	// Filling the display with some color
	st_fill_screen(ST_COLOR_NAVY);

	st_draw_string_withbg(10, 2, "16ton presents", ST_COLOR_YELLOW, ST_COLOR_BLACK, &font_fixedsys_mono_24);
	st_draw_string(10, 100, "white magic probe", 0xffff, &font_fixedsys_mono_24);
	// left lines lft/right updwn then rightvlines lft/rght up/dwn 
	st_draw_rectangle(240, 15, 80, 25, ST_COLOR_YELLOW);
//		fibre_run(&usb_task);

//	fibre_scheduler_main_loop();
   }
}

void platform_nrst_set_val(bool assert)
{
	if (assert) {
		gpio_set_mode(NRST_PORT, GPIO_MODE_OUTPUT_2_MHZ,
		              GPIO_CNF_OUTPUT_OPENDRAIN, nrst_pin);
		gpio_clear(NRST_PORT, nrst_pin);
	} else {
		gpio_set_mode(NRST_PORT, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_PULL_UPDOWN, nrst_pin);
		gpio_set(NRST_PORT, nrst_pin);
	}
}

bool platform_nrst_get_val()
{
	return gpio_get(NRST_PORT, nrst_pin) == 0;
}

static void adc_init(void)
{
	rcc_periph_clock_enable(RCC_ADC1);

	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_ANALOG, GPIO0);

	adc_power_off(ADC1);
	adc_disable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);
        adc_enable_temperature_sensor();
	adc_power_on(ADC1);

	/* Wait for ADC starting up. */
	for (int i = 0; i < 800000; i++)    /* Wait a bit. */
		__asm__("nop");

	adc_reset_calibration(ADC1);
	adc_calibrate(ADC1);
}

const char *platform_target_voltage(void)
{
	static char ret[] = "0.00V";
	static char ret2[] = "0.00V";
	const uint8_t channel = 0;
	adc_set_regular_sequence(ADC1, 1, (uint8_t*)&channel);
	adc_start_conversion_direct(ADC1);
	/* Wait for end of conversion. */
	while (!adc_eoc(ADC1));
	uint32_t platform_adc_value = adc_read_regular(ADC1);

	const uint8_t ref_channel = 17;
	adc_set_regular_sequence(ADC1, 1, (uint8_t*)&ref_channel);
	adc_start_conversion_direct(ADC1);
	/* Wait for end of conversion. */
	while (!adc_eoc(ADC1));
	uint32_t vrefint_value = adc_read_regular(ADC1);

	/* Value in mV*/
	uint32_t val = (platform_adc_value * 2400) / vrefint_value;
	ret[0] = '0' +	val / 1000;
	ret[2] = '0' + (val /  100) % 10;
	ret[3] = '0' + (val /	10) % 10;

    st_fill_screen(ST_COLOR_NAVY);
    strcpy(ret2, ret);
    st_draw_string(10, 10, ret2, 0xffff, &font_fixedsys_mono_24);
    
    	/* prepare the scheduler */

	return ret;
}
