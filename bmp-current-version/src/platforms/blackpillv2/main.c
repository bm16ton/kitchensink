#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include "systime.h"
#include "ws2812_spi.h"

int main(void) {

	rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
    systime_setup(84000);

	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_SPI1);

	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO5);
	gpio_set_af(GPIOB, GPIO_AF5, GPIO5);

	// use PIN B5 as ws2812 open drain output (1K pullup -> +5V)
	gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, GPIO5);
	ws2812_init(SPI1);

//	int h = 0;
//	while (1) {
//		h = (h+1) % 360;
		ws2812_write_hsv(SPI1, 120, 10, 1);
		delay_ms(15);
//    }

    return 0;
}
