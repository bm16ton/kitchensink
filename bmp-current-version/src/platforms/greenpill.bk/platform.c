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
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>

jmp_buf fatal_error_jmpbuf;
extern char _ebss[];
static void adc_init(void);

void platform_init(void)
{
	volatile uint32_t *magic = (uint32_t *)_ebss;
	/* Enable GPIO peripherals */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOB);
    gpio_mode_setup(GPIOC, GPIO_MODE_INPUT,
					GPIO_PUPD_PULLUP, GPIO14);

	/* Check the USER button*/
	if (!gpio_get(GPIOC, GPIO14) ||
		((magic[0] == BOOTMAGIC0) && (magic[1] == BOOTMAGIC1)))
	{
		magic[0] = 0;
		magic[1] = 0;
		/* Assert blue LED as indicator we are in the bootloader */
		gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT,
						GPIO_PUPD_NONE, GPIO13);
		gpio_clear(GPIOC, GPIO13);
		/* Jump to the built in bootloader by mapping System flash.
		   As we just come out of reset, no other deinit is needed!*/
		rcc_periph_clock_enable(RCC_SYSCFG);
		SYSCFG_MEMRM &= ~3;
		SYSCFG_MEMRM |= 1;
		scb_reset_core();
	}

	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT,
					GPIO_PUPD_NONE, GPIO13);
	gpio_clear(GPIOC, GPIO13);

	rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);

	/* Enable peripherals */
	rcc_periph_clock_enable(RCC_OTGFS);
	rcc_periph_clock_enable(RCC_CRC);

	/* Set up USB Pins and alternate function*/
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,  GPIO11 | GPIO12);
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



    SCB_VTOR = (uint32_t) 0x08000000;

	platform_timing_init();
	blackmagic_usb_init();
	usbuart_init();
    adc_init();
	gpio_clear(GPIOC, GPIO13);

	// https://github.com/libopencm3/libopencm3/pull/1256#issuecomment-779424001
	OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS | OTG_GCCFG_PWRDWN;
	OTG_FS_GCCFG &= ~(OTG_GCCFG_VBUSBSEN | OTG_GCCFG_VBUSASEN);
}

void platform_nrst_set_val(bool assert) { (void)assert; }
bool platform_nrst_get_val(void) { return false; }

static void adc_init(void)
{
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO2);
	rcc_periph_clock_enable(RCC_ADC1);
	adc_power_off(ADC1);
	adc_disable_scan_mode(ADC1);
	adc_set_sample_time(ADC1, ADC_CHANNEL2, ADC_SMPR_SMP_3CYC);
	adc_power_on(ADC1);
}
/*
static void adc_init(void)
{
	rcc_periph_clock_enable(RCC_ADC1);


	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO2);

	adc_power_off(ADC1);
	adc_disable_scan_mode(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_3CYC);
    adc_set_single_conversion_mode(ADC1);
    adc_set_right_aligned(ADC1);
	adc_power_on(ADC1);
	uint8_t channels[] = {ADC_CHANNEL2};
    adc_set_regular_sequence(ADC1, 1, channels);
	
}

const char *platform_target_voltage(void)
{

	static char ret[] __attribute__ ((aligned(4))) = "0.0V";
	const uint8_t channel = ADC_CHANNEL2;
	adc_set_regular_sequence(ADC1, 1, (uint8_t*)&channel);
    adc_start_conversion_regular(ADC1);
	while (!adc_eoc(ADC1));
	    ;
	uint32_t platform_adc_value = adc_read_regular(ADC1);
	



//	const uint8_t ref_channel = 17;
//	adc_set_regular_sequence(ADC1, 1, (uint8_t*)&ref_channel);

//	while (!adc_eoc(ADC1));
//	uint32_t vrefint_value = adc_read_regular(ADC1);

	// Value in mV
	uint32_t value = (platform_adc_value * 3.3) / 4095;
//	ret[0] = '0' +	val / 1000;
//	ret[2] = '0' + (val /  100) % 10;
//	ret[3] = '0' + (val /	10) % 10;

	value *= 3379; // 3.3 * 1024 == 3379.2 
	value += 104858; // round, 0.05V * 2 ^ 21 == 104857.6 \
	ret[0] = (value >> 21) + '0';
	value &= (1 << 21) - 1;
	value *= 10;
	ret[2] = (value >> 21) + '0';
	ret[sizeof(ret-1)] = '\0';

//    return val;
    
    return ret;
}
*/
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

	return ret;
}

void platform_request_boot(void)
{
	uint32_t *magic = (uint32_t *)&_ebss;
	magic[0] = BOOTMAGIC0;
	magic[1] = BOOTMAGIC1;
	scb_reset_system();
}


