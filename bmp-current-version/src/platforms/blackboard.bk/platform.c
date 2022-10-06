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
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include "general.h"
#include "usb.h"
#include "usbuart.h"
#include "morse.h"
#include <usb_i2c.h>
#include <usb_gpio.h>
#include <usb_adc.h>
#include <altusb.h>
#include <u8x8.h>
#include <setjmp.h>
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
#include <libopencm3/stm32/iwdg.h>
#include <librfn/fibre.h>
#include <librfn/time.h>
#include <librfn/util.h>
//#include "systime.h"
#include "xpt2046.h"

#include <i2c_explorer.h>
#include "ws2812_spi.h"
#include "timing_stm32.h"
#include "st7789_stm32_spi.h"
#include "fonts/font_fixedsys_mono_24.h"
#include "fonts/pic.h"

#include "fonts/bitmap_typedefs.h"
#include "ILI9486_Defines.h"
#include "fonts/font_ubuntu_48.h"
#include <libopencm3/stm32/usart.h>

#include "seesawneo.h"

#include "include/nrf24l01_regs.h"
#include "include/nrf24l01_hw.h"
#include "include/nrf24l01.h"


#define IRQ_TYPE_NONE		0
#define IRQ_TYPE_EDGE_RISING	0x00000001
#define IRQ_TYPE_EDGE_FALLING	0x00000002
#define IRQ_TYPE_EDGE_BOTH	IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING
#define IRQ_TYPE_LEVEL_HIGH	0x00000004
#define IRQ_TYPE_LEVEL_LOW	0x00000008

#define I2C_MEMORY_ADDR 0x49

static void i2c_setup2(void);
void adcboot(void);
void platform_request_boot2(void);
int usbmode;
void neopixel_init(void);
void adc_init(void);

void nrf_dump_regs(nrf_regs *r);

//void tsirq_pin_init(void);
//int tsxor = 1;
volatile int seesaw = 0;
static char ret2[] = "0.0V";
int adcrun = 0;
jmp_buf fatal_error_jmpbuf;
extern char _ebss[];
uint8_t usbd_control_buffer[256];

#define USART_CONSOLE USART1

#define FW_ADDR    0x08080000

void get_buffered_line(void);
static void usart_setup(void);



static inline __attribute__((always_inline)) void __set_MSP(uint32_t topOfMainStack)
{
  __asm volatile ("MSR msp, %0" : : "r" (topOfMainStack) : );
}

static inline __attribute__((always_inline)) void bootjump(void) {
    SCB_VTOR = FW_ADDR;
    __set_MSP(*(volatile uint32_t *)FW_ADDR);
	void (*start)(void) = (void *)*(volatile uint32_t *)(FW_ADDR + 4);
    start();
}

void platform_init(void)
{
	volatile uint32_t *magic = (uint32_t *)_ebss;
	/* Enable GPIO peripherals */
	if ((magic[0] == BOOTMAGIC6) && (magic[1] == BOOTMAGIC7)) 
	{
//	rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    magic[0] = 0;
	magic[1] = 0;
	/* Enable peripherals */
	
	    //FW_ADDR has to be aligned to 0x100
    bootjump();
    }
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOE);
	rcc_periph_clock_enable(RCC_GPIOD);
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT,
					GPIO_PUPD_PULLUP, GPIO0);
	gpio_mode_setup(GPIOE, GPIO_MODE_INPUT,
					GPIO_PUPD_PULLUP, GPIO4);
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
   
	if (!gpio_get(GPIOE, GPIO4) || ((magic[0] == BOOTMAGIC2) && (magic[1] == BOOTMAGIC3))) 
	{
		magic[0] = 0;
		magic[1] = 0;
	rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT,
			GPIO_PUPD_NONE, GPIO6);
	/* Enable peripherals */
    SCB_VTOR = (uint32_t) 0x08000000;
	rcc_periph_clock_enable(RCC_OTGFS);
	rcc_periph_clock_enable(RCC_CRC);
	rcc_periph_clock_enable(RCC_USART1);
//    rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
			GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);

    usbmode = 1;

	
	usart_setup();
    i2c_init();
    i2c_setup2();
    systime_setup(168000);
    
	usbgpio_init();
    blackmagic_usb_init(1);
	
    gpio_clear(GPIOA, GPIO6);


    dma2_setup();
    adc_start();
    neopixel_init();
    seesawneoint(24);
    clearseesaw(24);
    rcc_periph_clock_enable(RCC_DMA1);
    tftdma();
    st_init();
          
    st_fill_screen(ST_COLOR_YELLOW);

	st_draw_string_withbg(10, 5, "16ton presents", ST_COLOR_RED, ST_COLOR_PURPLE, &font_ubuntu_48);
//	st_draw_string(10, 50, "white magic probe", ST_COLOR_NAVY, &font_fixedsys_mono_24);
    st_draw_string(10, 69, "usb adc i2c", ST_COLOR_RED, &font_ubuntu_48);
    st_draw_bitmap(352, 192, &bm16ton);
    st_draw_rectangle(385, 95, 80, 25, ILI9486_RED);
	st_fill_rect(385, 95, 80, 25, ILI9486_RED);
	st_draw_string(385, 95, "ADC", ST_COLOR_PURPLE, &font_fixedsys_mono_24);
	st_draw_rectangle(385, 125, 80, 25, ILI9486_RED);
	st_fill_rect(385, 125, 80, 25, ILI9486_RED);
	st_draw_string(385, 125, "BMP", ST_COLOR_YELLOW, &font_fixedsys_mono_24);
	st_draw_rectangle(385, 155, 80, 25, ILI9486_RED);
	st_fill_rect(385, 155, 80, 25, ILI9486_RED);
	st_draw_string(385, 155, "can bus", ST_COLOR_YELLOW, &font_fixedsys_mono_24);
	OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS | OTG_GCCFG_PWRDWN;
	OTG_FS_GCCFG &= ~(OTG_GCCFG_VBUSBSEN | OTG_GCCFG_VBUSASEN);
    tsirq_pin_init();

 	OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS | OTG_GCCFG_PWRDWN;
	OTG_FS_GCCFG &= ~(OTG_GCCFG_VBUSBSEN | OTG_GCCFG_VBUSASEN);
	printf("Booted BMP\n");
	neoup(0x18, 0xa, 0xff, 0xc);
    neodown(0x18, 0x0, 0x0, 0x0);
//	return;
	} else if ((magic[0] == BOOTMAGIC6) && (magic[1] == BOOTMAGIC7)) 
	{
	rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    magic[0] = 0;
	magic[1] = 0;
	/* Enable peripherals */
	
	    //FW_ADDR has to be aligned to 0x100
    bootjump();
    
    
	rcc_periph_clock_enable(RCC_OTGFS);
	rcc_periph_clock_enable(RCC_CRC);
	rcc_periph_clock_enable(RCC_USART1);
    usbmode = 3;
    usart_setup();
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);
	#define CAN1_PORT GPIOD
    #define CAN1_PINS (GPIO0 | GPIO1)
    #define CAN1_AF 9
	gpio_mode_setup(CAN1_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, CAN1_PINS);
	gpio_set_output_options(CAN1_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ,
							CAN1_PINS);
	gpio_set_af(CAN1_PORT, CAN1_AF, CAN1_PINS);



	systime_setup(168000);
 	blackmagic_usb_init(2);
 	extern void slcan_init();
    slcan_init();
	i2c_setup2();
    adc_init();
    seesawneoint(24);
    clearseesaw(24);
    rcc_periph_clock_enable(RCC_DMA1);
    tftdma();
    st_init();

    delay(10);

    st_fill_screen(ST_COLOR_YELLOW);
    delay(122);

    st_draw_bitmap(0, 192, &bm16ton);
	st_draw_string(10, 20, "16TON'S OF SLCAN", ST_COLOR_BLACK, &font_ubuntu_48);

	st_draw_rectangle(385, 95, 80, 25, ILI9486_RED);
	st_fill_rect(385, 95, 80, 25, ILI9486_RED);
	st_draw_string(385, 95, "ADC", ST_COLOR_YELLOW, &font_fixedsys_mono_24);
	st_draw_rectangle(385, 125, 80, 25, ILI9486_RED);
	st_fill_rect(385, 125, 80, 25, ILI9486_RED);
	st_draw_string(385, 125, "BMP", ST_COLOR_YELLOW, &font_fixedsys_mono_24);
	st_draw_rectangle(385, 155, 80, 25, ILI9486_RED);
	st_fill_rect(385, 155, 80, 25, ILI9486_RED);
	st_draw_string(385, 155, "i2c_ex", ST_COLOR_YELLOW, &font_fixedsys_mono_24);
	st_draw_rectangle(385, 185, 80, 25, ILI9486_RED);
	st_fill_rect(385, 185, 80, 25, ILI9486_RED);
	st_draw_string(385, 185, "canbus", ST_COLOR_PURPLE, &font_fixedsys_mono_24);

    tsirq_pin_init();
//	sspeed = rcc_get_spi_clk_freq(SPI1);
//	sspeed2 = rcc_get_spi_clk_freq(SPI2);
	OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS | OTG_GCCFG_PWRDWN;
	OTG_FS_GCCFG &= ~(OTG_GCCFG_VBUSBSEN | OTG_GCCFG_VBUSASEN);
	printf("Booted BMP\n");

    neoup(0x18, 0xa, 0xff, 0xc);
    neodown(0x18, 0x0, 0x0, 0x0);
//    neoeveryother(0xff, 0x0, 0xff, 0x0, 0xff, 0x0);

	} else {



	rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

	/* Enable peripherals */
	rcc_periph_clock_enable(RCC_OTGFS);
	rcc_periph_clock_enable(RCC_CRC);
	rcc_periph_clock_enable(RCC_USART1);
    usbmode = 2;
    usart_setup();
	/* Set up USB Pins and alternate function*/
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);

//	GPIOC_OSPEEDR &= ~0xF30;
//	GPIOC_OSPEEDR |= 0xA20;

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

    
    systime_setup(168000);
 	blackmagic_usb_init(0);
	usbuart_init();
	i2c_setup2();
    adc_init();
    seesawneoint(24);
    
     clearseesaw(24);
//    sendi2cdma();
    nrf_init();
    
    static nrf_reg_buf addr;

    addr.data[0] = 0xE1;
    addr.data[1] = 0xF0;
    addr.data[2] = 0XF0;
    addr.data[3] = 0XF0;
    addr.data[4] = 0XF0;
//    gpio_clear(GPIOB, GPIO6);
    nrf_preset_esb(NRF_MODE_PTX, 70, 32, 3, NRF_RT_DELAY_250, &addr);
//    gpio_set(GPIOB, GPIO6);
    delay(1400);
//    gpio_set(GPIOB, GPIO6);
//    gpio_clear(GPIOB, GPIO6);

//    gpio_set(GPIOB, GPIO6);
    delay(100);
    nrf_payload   p;
    p.size = 32;
    p.data[0] = 123;
//    gpio_set(GPIOB, GPIO6);
//    gpio_set(GPIOB, GPIO7);
//    gpio_clear(GPIOB, GPIO7);
    int ret54 =  nrf_send(&p);
    delay(200);
//    int ret54 =  nrf_send_blocking(&p);
    if (ret54 >= 31) {
    printf("transmit too small %d\n", ret54);
    }
    nrf_dump_regs(&nrf_reg_def);
    delay(100);
    rcc_periph_clock_enable(RCC_DMA1);
    tftdma();
    st_init();
    delay(100);

    st_fill_screen(0xD69A);
    delay(122);

    st_draw_bitmap(0, 192, &bm16ton);
	st_draw_string(10, 20, "white magic probe", ST_COLOR_BLACK, &font_ubuntu_48);

	st_draw_rectangle(385, 95, 80, 25, ILI9486_RED);
	st_fill_rect(385, 95, 80, 25, ILI9486_RED);
	st_draw_string(385, 95, "ADC", ST_COLOR_YELLOW, &font_fixedsys_mono_24);
	st_draw_rectangle(385, 125, 80, 25, ILI9486_RED);
	st_fill_rect(385, 125, 80, 25, ILI9486_RED);
	st_draw_string(385, 125, "BMP", ST_COLOR_PURPLE, &font_fixedsys_mono_24);
	st_draw_rectangle(385, 155, 80, 25, ILI9486_RED);
	st_fill_rect(385, 155, 80, 25, ILI9486_RED);
	st_draw_string(385, 155, "i2c_ex", ST_COLOR_YELLOW, &font_fixedsys_mono_24);
	st_draw_rectangle(385, 185, 80, 25, ILI9486_RED);
	st_fill_rect(385, 185, 80, 25, ILI9486_RED);
	st_draw_string(385, 185, "can bus", ST_COLOR_YELLOW, &font_fixedsys_mono_24);

    tsirq_pin_init();

	OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS | OTG_GCCFG_PWRDWN;
	OTG_FS_GCCFG &= ~(OTG_GCCFG_VBUSBSEN | OTG_GCCFG_VBUSASEN);
	printf("Booted BMP\n");

    neoup(0x18, 0xa, 0xff, 0xc);
    neodown(0x18, 0x0, 0x0, 0x0);
//    neoeveryother(0xff, 0x0, 0xff, 0x0, 0xff, 0x0);
	}
}

int _write(int file, char *ptr, int len)
{
	int i;

	if (file == STDOUT_FILENO || file == STDERR_FILENO) {
		for (i = 0; i < len; i++) {
			if (ptr[i] == '\n') {
				usart_send_blocking(USART1, '\r');
			}
			usart_send_blocking(USART1, ptr[i]);
		}
		return i;
	}
	errno = EIO;
	return -1;
}


static void i2c_setup2(void) {
  // Set alternate functions for the SCL and SDA pins of I2C1. 
	// GPIO for I2C1 


	rcc_periph_clock_enable(RCC_I2C3);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO8);
	gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ,
				GPIO8);
	gpio_set_af(GPIOA, GPIO_AF4, GPIO8);
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO9);
	gpio_set_output_options(GPIOC, GPIO_OTYPE_OD, GPIO_OSPEED_100MHZ,
				GPIO9);
	gpio_set_af(GPIOC, GPIO_AF4, GPIO9);

	i2c_peripheral_disable(I2C3);
	i2c_set_clock_frequency(I2C3, 30);
	i2c_set_ccr(I2C3, 30 * 5);
	i2c_set_trise(I2C3, 30 + 1);
	i2c_enable_ack(I2C3);
	i2c_set_own_7bit_slave_address(I2C3, 0x32);
//	i2c_set_fast_mode(I2C3);
    i2c_set_standard_mode(I2C3);
	i2c_peripheral_enable(I2C3);

  for (uint32_t loop = 0; loop < 1000000; ++loop) {
    __asm__("nop");
  }
}  


static void usart_setup(void)
{
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO9);
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, 
	                GPIO10); 
	gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, 
					GPIO_OSPEED_100MHZ, GPIO10); 
	gpio_set_af(GPIOA, GPIO_AF7, GPIO10); 
	
	usart_set_baudrate(USART_CONSOLE, 115200);
	usart_set_databits(USART_CONSOLE, 8);
	usart_set_stopbits(USART_CONSOLE, USART_STOPBITS_1);
	usart_set_mode(USART_CONSOLE, USART_MODE_TX_RX);
	usart_set_parity(USART_CONSOLE, USART_PARITY_NONE);
	usart_set_flow_control(USART_CONSOLE, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART_CONSOLE);
}

void tsirq_pin_init(void)
{
	delay(100);
    nvic_enable_irq(NVIC_EXTI1_IRQ);					
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO1);
	delay(100);
	exti_select_source(EXTI1, GPIO1);
    exti_set_trigger(EXTI1, IRQ_TYPE_EDGE_FALLING);
	exti_enable_request(EXTI1);
}

void wdg(void) {
            uint32_t *magic = (uint32_t *)&_ebss;
            magic[0] = BOOTMAGIC2;
	        magic[1] = BOOTMAGIC3;
//	        iwdg_set_period_ms(2000);
	        iwdg_start();
}

void exti1_isr(void)
{
//	exti_reset_request(EXTI1);
    volatile int xrw;
    volatile int yrw;
    volatile int xraw;
    volatile int yraw;
    volatile int16_t thr;
//    volatile int zraw1;
//    volatile int zraw2;

    uint32_t *magic = (uint32_t *)&_ebss;
    xrw = ts_get_x();
    yrw = ts_get_y();
    thr = threshholdv();
    xraw = ts_get_x_raw();

    yraw = ts_get_y_raw();
    printf("threshhold =  %d\n", thr);
    printf("xraw =  %d\n", xraw);
    printf("yraw =  %d\n", yraw);
    printf("xrw =  %d\n", xrw);
    printf("yrw =  %d\n", yrw);

    

    if ((xraw >= 700 && xraw <= 800 ) && (yraw >= 400 && yraw <= 550)) {

           magic[0] = BOOTMAGIC2;
           magic[1] = BOOTMAGIC3;

	        GPIOA_MODER |= (0x00000000);

            scb_reset_system();
            scb_reset_core();

            return;
          
    }
    if ((xraw >= 850 && xraw <= 980 ) && (yraw >= 350 && yraw <= 450)) {
            scb_reset_system();
            platform_request_boot2();
    }
    
    if ((xraw >= 500 && xraw <= 600 ) && (yraw >= 2000 && yraw <= 2050)) {

          magic[0] = BOOTMAGIC6;
          magic[1] = BOOTMAGIC7;
//st_fill_screen(ILI9486_GREEN);
	        GPIOA_MODER |= (0x00000000);

            scb_reset_system();
            scb_reset_core();
            return;
//            return;    , 0x20,  0x01, 0x01, 220, 0, 220, 0, 220, 0, 250, 250, 0, 0x05
/*    clearseesaw(24);
    uint8_t cmdWrite[] = { 0xe, 0x2, 0x1 };
	i2c_transfer7(I2C3, I2C_MEMORY_ADDR, cmdWrite, sizeof(cmdWrite), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }
    uint8_t cmdWrite1[] = { 0xe, 0x3, 0x0, 0x24 };
	i2c_transfer7(I2C3, I2C_MEMORY_ADDR, cmdWrite1, sizeof(cmdWrite1), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }
    uint8_t cmdWrite2[] = { 0xe, 0x1, 0xa };
	i2c_transfer7(I2C3, I2C_MEMORY_ADDR, cmdWrite2, sizeof(cmdWrite2), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }
    uint8_t cmdWrite3[] = { 0xe, 0x5 };
	i2c_transfer7(I2C3, I2C_MEMORY_ADDR, cmdWrite3, sizeof(cmdWrite3), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }
    uint8_t cmdWrite13[] = { 0xe, 0x5 };
	i2c_transfer7(I2C3, I2C_MEMORY_ADDR, cmdWrite13, sizeof(cmdWrite13), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }
  
  uint8_t cmdWrite4[] = { 0xe, 0x4, 0x0, 0x3, 0x0, 0xff, 0x0 };
	i2c_transfer7(I2C3, I2C_MEMORY_ADDR, cmdWrite4, sizeof(cmdWrite4), NULL, 0);
 for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }

    uint8_t cmdWrite14[] = { 0xe, 0x5 };
	i2c_transfer7(I2C3, I2C_MEMORY_ADDR, cmdWrite14, sizeof(cmdWrite14), NULL, 0);

  uint8_t cmdWrite5[] = { 0xe, 0x4, 0x0, 0x0, 0x0, 0x0, 0x0 };
	i2c_transfer7(I2C3, I2C_MEMORY_ADDR, cmdWrite5, sizeof(cmdWrite5), NULL, 0);
	 for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
    }

   uint8_t cmdWrite21[] = { 0xe, 0x4, 0x0, 0xC, 0x0, 0xff, 0xff };
	i2c_transfer7(I2C3, I2C_MEMORY_ADDR, cmdWrite21, sizeof(cmdWrite21), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }
    uint8_t cmdWrite22[] = { 0xe, 0x5 };
	i2c_transfer7(I2C3, I2C_MEMORY_ADDR, cmdWrite22, sizeof(cmdWrite22), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }
   uint8_t cmdWrite23[] = { 0xe, 0x4, 0x0, 0xf, 0xff, 0x0, 0xff };
	i2c_transfer7(I2C3, I2C_MEMORY_ADDR, cmdWrite23, sizeof(cmdWrite23), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }
  
     uint8_t cmdWrite24[] = { 0xe, 0x5 };
	i2c_transfer7(I2C3, I2C_MEMORY_ADDR, cmdWrite24, sizeof(cmdWrite24), NULL, 0);
  for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }
  
  
  uint8_t cmdWrite25[] = { 0xe, 0x4, 0x0, 0x21, 0xff, 0xff, 0x0 };
	i2c_transfer7(I2C3, I2C_MEMORY_ADDR, cmdWrite25, sizeof(cmdWrite25), NULL, 0);
 for (uint32_t loop = 0; loop < 100; ++loop) {
    __asm__("nop");
  }
    uint8_t cmdWrite26[] = { 0xe, 0x5 };
	i2c_transfer7(I2C3, I2C_MEMORY_ADDR, cmdWrite14, sizeof(cmdWrite26), NULL, 0);
*/

     }
    exti_reset_request(EXTI1);
    exti_set_trigger(EXTI1, IRQ_TYPE_EDGE_FALLING);
       
}

void platform_nrst_set_val(bool assert) { (void)assert; }
bool platform_nrst_get_val(void) { return false; }


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

//    for (int i = 0; i < 1024; i++) {
//        tfttx[i] = i;
//    }

    // Begin a DMA transfer using that data
//    dma_start(tfttx, 1024);
//   st_fill_rect(1, 110, 74, 60, ILI9486_LIGHTGREY);
//   st_draw_string(10, 65, "Target IO", ILI9486_DARKGREEN, &font_ubuntu_48);
//	st_draw_string(10, 110, ret2, ST_COLOR_RED, &font_ubuntu_48);
	return ret;
}


void platform_request_boot2(void)
{
	uint32_t *magic = (uint32_t *)&_ebss;
	st_fill_screen(ILI9486_GREEN);
//	st_draw_string_withbg(90, 110, "DFU FIRMWARE UPGRADE", ST_COLOR_RED, ST_COLOR_WHITE, &font_fixedsys_mono_24);
	usart_disable(USART_CONSOLE);
	rcc_periph_clock_disable(RCC_USART1);
	rcc_periph_clock_disable(RCC_USART3);
    cm_disable_interrupts();
	usart_disable(USART_CONSOLE);
//	rcc_periph_clock_disable(SPI2);
//	gpio_clear(GPIOA, GPIO9);
	GPIOA_MODER |= (0x00000000);
	GPIOB_MODER |= (0x00000000);
	GPIOC_MODER |= (0x00000000);
	GPIOD_MODER |= (0x00000000);
	delay(122);
	magic[0] = BOOTMAGIC2;
	magic[1] = BOOTMAGIC3;
	scb_reset_system();
//	POO = (0x5FA << 16) |
//	SCB_AIRCR_SYSRESETREQ;
//	    while (1==1);
}

void platform_request_boot(void)
{
	uint32_t *magic = (uint32_t *)&_ebss;
	st_fill_screen(ILI9486_BLACK);
	st_draw_string_withbg(90, 110, "DFU FIRMWARE UPGRADE", ST_COLOR_RED, ST_COLOR_WHITE, &font_fixedsys_mono_24);
	usart_disable(USART_CONSOLE);
	rcc_periph_clock_disable(RCC_USART1);
//	gpio_clear(GPIOA, GPIO9);
	GPIOA_MODER |= (0x00000000);
	delay(122);
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

