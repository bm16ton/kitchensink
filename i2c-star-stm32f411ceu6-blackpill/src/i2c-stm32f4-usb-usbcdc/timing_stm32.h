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
#ifndef __TIMING_STM32_H
#define __TIMING_STM32_H

#include <stdint.h>

extern uint32_t swd_delay_cnt;
extern uint8_t running_status;

void platform_timing_init(void);
//void platform_delay(uint32_t ms);

uint32_t platform_max_frequency_get(void);
void platform_max_frequency_set(uint32_t freq);
void systime_setup(uint32_t cpufreq_kHz);


uint32_t get_time_ms(void);
uint32_t get_time_us32(void);
uint64_t get_time_us64(void);

void delay_ms(uint32_t delay);
#endif

