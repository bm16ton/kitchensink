/*
 * ws2812_spi.c
 *
 *  Created on: 14.08.2015
 *      Author: hd
 */

#include "ws2812_spi.h"
#include <libopencm3/stm32/spi.h>

void ws2812_init(uint32_t spi) {
	spi_init_master(spi, SPI_CR1_BAUDRATE_FPCLK_DIV_32, 0, 0, 0, 0);
	spi_set_dff_8bit(spi);
	spi_enable(spi);
}

static uint32_t ws2812_encode_byte(uint8_t b) {
	uint32_t result = 0;
	for (int i=0; i<8; i++) {
		result <<= 3;
		if (b & 0x80) {
			result |= 0b110;
		} else {
			result |= 0b100;
		}
		b<<=1;
	}
	return result;
}

static void ws2812_write_byte(uint32_t spi, uint8_t b) {
	uint32_t data = ws2812_encode_byte(b);
	spi_send(spi, (data>>16) & 0xFF);
	spi_send(spi, (data>> 8) & 0xFF);
	spi_send(spi, (data>> 0) & 0xFF);
}

void ws2812_write_rgb(uint32_t spi, uint8_t r, uint8_t g, uint8_t b) {
	for (int i=0; i<20; i++) { spi_send(spi, 0); }
	ws2812_write_byte(spi, g);
	ws2812_write_byte(spi, r);
	ws2812_write_byte(spi, b);
	spi_send(spi, 0);
}

void ws2812_write_hsv(uint32_t spi, float h, float s, float v) {

	float r,g,b;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"

	if (s==0) {

#pragma GCC diagnostic pop

		r = g = b = v;
	} else {
		h /= 60;
		int i = h;
		float f = h - i;
		float p = v * ( 1 - s );
		float q = v * ( 1 - s * f );
		float t = v * ( 1 - s * ( 1 - f ) );

		switch (i) {
			case 0:
				r = v; g = t; b = p;
				break;
			case 1:
				r = q; g = v; b = p;
				break;
			case 2:
				r = p; g = v; b = t;
				break;
			case 3:
				r = p; g = q; b = v;
				break;
			case 4:
				r = t; g = p; b = v;
				break;
			default: // case 5:
				r = v; g = p; b = q;
				break;
		}
	}

	ws2812_write_rgb(spi, r*255, g*255, b*255);
}
