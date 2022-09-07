#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include "lcd-dma.h"
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/ltdc.h>

#include "timing_stm32.h"
//#include "clock.h"
//#include "console.h"
#include "lcd-spi.h"
//#include "sdram.h"

#define LCD_WIDTH  240
#define LCD_HEIGHT 320
#define REFRESH_RATE 70 /* Hz */

#define HSYNC       10
#define HBP         20
#define HFP         10

#define VSYNC        2
#define VBP          2
#define VFP          4

/* Layer 1 (bottom layer) is ARGB8888 format, full screen. */

typedef uint32_t layer1_pixel;
#define LCD_LAYER1_PIXFORMAT LTDC_LxPFCR_ARGB8888


#define LCD_LAYER1_PIXEL_SIZE (sizeof(layer1_pixel))
#define LCD_LAYER1_WIDTH  LCD_WIDTH
#define LCD_LAYER1_HEIGHT LCD_HEIGHT
#define LCD_LAYER1_PIXELS (LCD_LAYER1_WIDTH * LCD_LAYER1_HEIGHT)
#define LCD_LAYER1_BYTES  (LCD_LAYER1_PIXELS * LCD_LAYER1_PIXEL_SIZE)

/* Layer 2 (top layer) is ARGB4444, a 128x128 square. */

typedef uint16_t layer2_pixel;
#define LCD_LAYER2_PIXFORMAT LTDC_LxPFCR_ARGB4444

#define LCD_LAYER2_PIXEL_SIZE (sizeof(layer2_pixel))
#define LCD_LAYER2_WIDTH 128
#define LCD_LAYER2_HEIGHT 128
#define LCD_LAYER2_PIXELS (LCD_LAYER2_WIDTH * LCD_LAYER2_HEIGH)
#define LCD_LAYER2_BYTES (LCD_LAYER2_PIXELS * LCD_LAYER2_PIXEL_SIZE)

/*
 * Pin assignments
 *     R2      = PC10, AF14
 *     R3      = PB0,  AF09
 *     R4      = PA11, AF14
 *     R5      = PA12, AF14
 *     R6      = PB1,  AF09
 *     R7      = PG6,  AF14
 *
 *     G2      = PA6,  AF14
 *     G3      = PG10, AF09
 *     G4      = PB10, AF14
 *     G5      = PB11, AF14
 *     G6      = PC7,  AF14
 *     G7      = PD3,  AF14
 *
 *     B2      = PD6,  AF14
 *     B3      = PG11, AF11
 *     B4      = PG12, AF09
 *     B5      = PA3,  AF14
 *     B6      = PB8,  AF14
 *     B7      = PB9,  AF14
 *
 * More pins...
 *     ENABLE  = PF10, AF14
 *     DOTCLK  = PG7,  AF14
 *     HSYNC   = PC6,  AF14
 *     VSYNC   = PA4,  AF14
 *     CSX     = PC2         used in lcd-spi
 *     RDX     = PD12        not used: read SPI
 *     TE      = PD11        not used: tearing effect interrupt
 *     WRX_DCX = PD13        used in lcd-spi
 *     DCX_SCL = PF7         used in lcd-spi
 *     SDA     = PF9         used in lcd-spi
 *     NRST    = NRST
 */

void lcd_dma_init(void)
{


	/*
	 * The datasheet says (Figure 16, page 151):
	 *     The LCD-TFT clock comes from PLLSAI.
	 *     PLLSRC selects either HSI or HSE.
	 *     PLLSAI's input clock is either HSI or HSE divided by PLLM.
	 *     PLLSAI's PLLLCDCLK output is the input * PLLSAIN / PLLSAIR.
	 *     LCD-TFT clock is PLLLCDCLK divided by PLLSAIDIVR.
	 *
	 * PLLSRC and PLLM are in the RCC_PLLCFGR register.
	 * PLLSAIN and PLLSAIR are in RCC_PLLSAICFGR.
	 * PLLSAIDIVR is in RCC_DCKCFGR;
	 *
	 * In our case,
	 * PLLSRC already selected HSE, which is 8 MHz.
	 * PLLM is already set to 8.  8 MHz / 8 = 1 MHz.
	 * We set PLLSAIN = 192 and PLLSAIR = 4.  1 MHz * 192 / 4 = 48 MHz.
	 * We set PLLSAIDIVR to 8.  48 MHz / 8 = 6 MHz.
	 * So the LCD-TFT pixel clock is 6 MHz.
	 *
	 * The number of clocks per frame is
	 * (VSYNC + VBP + LCD_HEIGHT + VFP) * (HSYNC + HBP + LCD_WIDTH + HFP) =
	 * (2 + 2 + 320 + 4) * (10 + 20 + 240 + 10) = 91840.
	 *
	 * So the refresh frequency is 6 MHz / 91840 ~= 65.6 Hz.
	 */

	uint32_t sain = 192;
	uint32_t saiq = (RCC_PLLSAICFGR >> RCC_PLLSAICFGR_PLLSAIQ_SHIFT) &
			RCC_PLLSAICFGR_PLLSAIQ_MASK;
	uint32_t sair = 4;
	RCC_PLLSAICFGR = (sain << RCC_PLLSAICFGR_PLLSAIN_SHIFT |
			  saiq << RCC_PLLSAICFGR_PLLSAIQ_SHIFT |
			  sair << RCC_PLLSAICFGR_PLLSAIR_SHIFT);
	RCC_DCKCFGR |= RCC_DCKCFGR_PLLSAIDIVR_DIVR_8 << RCC_DCKCFGR_PLLSAIDIVR_SHIFT;
	RCC_CR |= RCC_CR_PLLSAION;
	while ((RCC_CR & RCC_CR_PLLSAIRDY) == 0) {
		continue;
	}
	RCC_APB2ENR |= RCC_APB2ENR_LTDCEN;

	/*
	 * Configure the Synchronous timings: VSYNC, HSNC,
	 * Vertical and Horizontal back porch, active data area, and
	 * the front porch timings.
	 */
	LTDC_SSCR = (HSYNC - 1) << LTDC_SSCR_HSW_SHIFT |
		    (VSYNC - 1) << LTDC_SSCR_VSH_SHIFT;
	LTDC_BPCR = (HSYNC + HBP - 1) << LTDC_BPCR_AHBP_SHIFT |
		    (VSYNC + VBP - 1) << LTDC_BPCR_AVBP_SHIFT;
	LTDC_AWCR = (HSYNC + HBP + LCD_WIDTH - 1) << LTDC_AWCR_AAW_SHIFT |
		    (VSYNC + VBP + LCD_HEIGHT - 1) << LTDC_AWCR_AAH_SHIFT;
	LTDC_TWCR =
	    (HSYNC + HBP + LCD_WIDTH + HFP - 1) << LTDC_TWCR_TOTALW_SHIFT |
	    (VSYNC + VBP + LCD_HEIGHT + VFP - 1) << LTDC_TWCR_TOTALH_SHIFT;

	/* Configure the synchronous signals and clock polarity. */
	LTDC_GCR |= LTDC_GCR_PCPOL_ACTIVE_HIGH;

	/* If needed, configure the background color. */
	LTDC_BCCR = 0x00000000;

	/* Configure the needed interrupts. */
	LTDC_IER = LTDC_IER_RRIE;
	nvic_enable_irq(NVIC_LCD_TFT_IRQ);

	/* Configure the Layer 1 parameters.
	 * (Layer 1 is the bottom layer.)    */


	/* Reload the shadow registers to active registers. */
	LTDC_SRCR |= LTDC_SRCR_VBR;

	/* Enable the LCD-TFT controller. */
	LTDC_GCR |= LTDC_GCR_LTDC_ENABLE;
}

void mutate_background_color(void)
{
	static uint32_t ints;
	ints += 3;
	uint32_t shift = ints >> 9;
	if (shift >= 3) {
		ints = shift = 0;
	}
	uint32_t component = ints & 0xFF;
	if (ints & 0x100) {
		component = 0xff - component;
	}

	LTDC_BCCR = component << 8 * shift;
}

/*
 * The sprite bounce algorithm works surprisingly well for a first
 * guess.  Whenever the sprite touches a wall, we reverse its velocity
 * normal to the wall, and pick a random velocity from -3 to +3
 * parallel to the wall.  (e.g., if it touches a side, reverse the X
 * velocity and pick a random Y velocity.)  That gives enough
 * unpredictability to make it interesting.
 *
 * The random numbers come from rand(), and we do not call srand().
 * That means the sprite makes exactly the same moves every time the
 * demo is run.  (Repeatability is a feature.)
 */

void move_sprite(void)
{
	static int8_t dx = 1, dy = 1;
	static int16_t x, y;
	static int16_t age;
	x += dx;
	y += dy;
	if (x < 0) {
		dy = rand() % 7 - 3;
		dx = -dx;
		x = 0;
		age = 0;
	if (y < 0) {
		dx = rand() % 7 - 3;
		dy = -dy;
		y = 0;
		age = 0;
	if (dy == 0 && dx == 0) {
		dy = y ? -1 : +1;
	}
	uint32_t h_start = HSYNC + HBP + x;

	LTDC_L2WHPCR = h_stop << LTDC_LxWHPCR_WHSPPOS_SHIFT |
		       h_start << LTDC_LxWHPCR_WHSTPOS_SHIFT;
	uint32_t v_start = VSYNC + VBP + y;

	LTDC_L2WVPCR = v_stop << LTDC_LxWVPCR_WVSPPOS_SHIFT |
		       v_start << LTDC_LxWVPCR_WVSTPOS_SHIFT;

	/* The sprite fades away as it ages. */
	age += 2;
	if (age > 0xFF) {
		age = 0xFF;
	}
	LTDC_L2CACR = 0x000000FF - age;
}
}
/*
 * Here is where all the work is done.  We poke a total of 6 registers
 * for each frame.
 */

void lcd_tft_isr(void)
{
	LTDC_ICR |= LTDC_ICR_CRRIF;

	mutate_background_color();
	move_sprite();

	LTDC_SRCR |= LTDC_SRCR_VBR;
}

/*
 * Checkerboard pattern.  Odd squares are transparent; even squares are
 * all different colors.
 */


/*
 * Layer 2 holds the sprite.  The sprite is a semitransparent
 * magenta/cyan diamond outlined in black.
 */

/*
int main(void)
{
	// init timers. 
	clock_setup();

	// set up USART 1.  
	console_setup(115200);
	console_stdio_setup();



	printf("Preloading frame buffers\n");

	draw_layer_1();
	draw_layer_2();

	printf("Initializing LCD\n");

	lcd_dma_init();
	lcd_spi_init();

	printf("Initialized.\n");

	while (1) {
		continue;
	}
}
*/
