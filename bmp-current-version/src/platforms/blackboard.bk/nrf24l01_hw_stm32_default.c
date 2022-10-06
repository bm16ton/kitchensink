/*
 * This file is part of the libemb project.
 *
 * Copyright (C) 2011 Stefan Wendler <sw@kaltpost.de>
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

#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/gpio.h>
#include <libopencm3/stm32/spi.h>

#include "include/nrf24l01_hw.h"

#define SPI_CS_HIGH		gpio_set(GPIOB, GPIO7)
#define SPI_CS_LOW		gpio_clear(GPIOB, GPIO7)

#define SPI_CE_HIGH     gpio_set(GPIOB, GPIO6);
#define SPI_CE_LOW      gpio_clear(GPIOB, GPIO6);
void nrf_init(void)
{

     rcc_periph_clock_enable(RCC_SPI1);
     /* Configure SCK and MOSI */
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO5);
	gpio_set_af(GPIOB, GPIO_AF5, GPIO5 | GPIO3);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
							GPIO5 | GPIO3);
     /* Configure MISO */

    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE,
        GPIO7	  // NSS - slave select
        | GPIO4 //miso
//       | TS_CS_PIN  // gpio A9
    );

    gpio_set_af(GPIOB, GPIO_AF5,
        GPIO7     // SPI1_NSS
        | GPIO4 //miso
//        | TS_CS_PIN  // gpio A9
    );

    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
							GPIO6);
//	gpio_set(GPIOB, GPIO6);

    
    spi_init_master(
        SPI1,
        SPI_CR1_BAUDRATE_FPCLK_DIV_16,
        SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,   // CPOL: Clock low when idle
        SPI_CR1_CPHA_CLK_TRANSITION_1,     // CPHA: Clock phase: read on rising edge of clock
//        SPI_CR1_CPHA,
        SPI_CR1_DFF_8BIT,
        SPI_CR1_MSBFIRST);

    spi_set_full_duplex_mode(SPI1);
    spi_set_unidirectional_mode(SPI1);
    spi_disable_crc(SPI1);
    spi_enable_software_slave_management(SPI1);
    spi_set_nss_high(SPI1);
    SPI_I2SCFGR(SPI1) &= ~SPI_I2SCFGR_I2SMOD;
    // Have SPI peripheral manage NSS pin (pulled low when SPI enabled)
    spi_enable_ss_output(SPI1);
    SPI_CS_HIGH;
//    gpio_set(GPIOB, GPIO6);
    spi_enable(SPI1);

     nrf_spi_ceh();
     /* PCLOCK/8 as clock. */
//     spi_set_baudrate_prescaler(SPI1, SPI_CR1_BR_FPCLK_DIV_8);

     /* We want to control everything and generate the clock -> master. */
//     spi_set_master_mode(SPI1);
//     spi_set_clock_polarity_0(SPI1); /* SCK idle state low. */

     /* Bit is taken on the second (falling edge) of SCK. */
//     spi_set_clock_phase_0(SPI1);
//     spi_enable_ss_output(SPI1);

     
}

void nrf_spi_csh(void)
{
     SPI_CS_HIGH;
}

void nrf_spi_csl(void)
{
     SPI_CS_LOW;
}

void nrf_spi_ceh(void)
{
     SPI_CE_HIGH;
}

void nrf_spi_cel(void)
{
     SPI_CE_LOW;
}

unsigned char nrf_spi_xfer_byte(unsigned char data)
{
     return spi_xfer(SPI1, data);
}

