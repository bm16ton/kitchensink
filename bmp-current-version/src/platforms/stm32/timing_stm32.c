/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2015 Gareth McMullin <gareth@blacksphere.co.nz>
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
#include "librfn/time.h"
#include "general.h"
#include "morse.h"
#include <assert.h>
#include <string.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include "platform.h"

//#include "systime.h"
#ifdef BLACKPILLV2
#define F_CPU 168000000
#endif
volatile uint32_t systick_ms;
static uint32_t cpufreq = 1;


uint8_t running_status;
static volatile uint32_t time_ms;
uint32_t swd_delay_cnt = 0;

//static int morse_tick;


static uint64_t sys_tick_counter;


void systime_setup(uint32_t cpufreq_kHz) {
	cpufreq = cpufreq_kHz;
	systick_set_reload(cpufreq_kHz-1);
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
//	nvic_set_priority(NVIC_SYSTICK_IRQ, 14 << 4);
	systick_counter_enable();
	systick_interrupt_enable();
}


void platform_timing_init(void)
{
	/* Setup heartbeat timer */
//	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	/* Interrupt us at 10 Hz */
//	systick_set_reload(rcc_ahb_frequency / (8 * SYSTICKHZ) );
	/* SYSTICK_IRQ with low priority */
//	cpufreq = cpufreq_kHz;
	systick_set_reload(72000-1);
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
//	nvic_set_priority(NVIC_SYSTICK_IRQ, 14 << 4);
	systick_interrupt_enable();
	systick_counter_enable();
}

void platform_delay(uint32_t ms)
{
	platform_timeout timeout;
	platform_timeout_set(&timeout, ms);
	while (!platform_timeout_is_expired(&timeout));
}

void sys_tick_handler(void)
{
    systick_ms++;
/*    if (systick_ms % 2000 == 0) {
//  millis_count++;
	lcdshow();
	systick_ms = 0;
//	gpio_toggle(GPIOC, GPIO13);
	}
    */
	time_ms += SYSTICKMS;
    sys_tick_counter += 1000;
}

void milli_sleep(uint32_t delay)
{
	uint32_t wake = systick_ms + delay;
	while (wake > systick_ms) {
		continue;
	}
}

uint32_t time_now()
{
	return sys_tick_counter;
}


uint64_t time64_now()
{
	return sys_tick_counter;
}

uint32_t platform_time_ms(void)
{
	return time_ms;
}

uint32_t get_time_ms(void) {
	return systick_ms;
}

uint32_t millis(void)
{
    return systick_ms;
}


uint32_t get_time_us32(void) {
	uint32_t us = 1000ul*systick_ms;
	uint32_t ticks = systick_get_value();
	return us + (1000ul*ticks)/cpufreq;
}

uint64_t get_time_us64(void) {
	uint64_t us = 1000ull*systick_ms;
	uint32_t ticks = systick_get_value();
	return us + (1000ull*ticks)/cpufreq;
}

void delay(uint32_t delay) {
	uint32_t wake = systick_ms + delay;
	while (wake > systick_ms);
}


void delay_ms(uint32_t delay) {
	uint32_t wake = systick_ms + delay;
	while (wake > systick_ms);
}

/* Assume some USED_SWD_CYCLES per clock
 * and  CYCLES_PER_CNT Cycles per delay loop cnt with 2 delay loops per clock
 */

/* Values for STM32F103 at 72 MHz */
#define USED_SWD_CYCLES 22
#define CYCLES_PER_CNT 10
void platform_max_frequency_set(uint32_t freq)
{
	int divisor = rcc_ahb_frequency - USED_SWD_CYCLES * freq;
	if (divisor < 0) {
		swd_delay_cnt = 0;
		return;
	}
	divisor /= 2;
	swd_delay_cnt = divisor/(CYCLES_PER_CNT * freq);
	if ((swd_delay_cnt * (CYCLES_PER_CNT * freq)) < (unsigned int)divisor)
		swd_delay_cnt++;
}

uint32_t platform_max_frequency_get(void)
{
	uint32_t ret = rcc_ahb_frequency;
	ret /= USED_SWD_CYCLES + CYCLES_PER_CNT * swd_delay_cnt;
	return ret;
}
