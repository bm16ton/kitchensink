/*
 * ws2812_spi.h
 *
 *  Created on: 14.08.2015
 *      Author: hd
 */

#ifndef WS2812_SPI_H_
#define WS2812_SPI_H_

#include <stdint.h>

void ws2812_init(uint32_t spi);
void ws2812_write_rgb(uint32_t spi, uint8_t r, uint8_t g, uint8_t b);
void ws2812_write_hsv(uint32_t spi, float h, float s, float v);

#endif /* WS2812_SPI_H_ */
