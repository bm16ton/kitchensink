#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "usb_adc.h"
#include "usb.h"
#include <libopencm3/usb/usbd.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include "timing_stm32.h"
#include "st7789_stm32_spi.h"
#include "fonts/font_fixedsys_mono_24.h"
#include "fonts/font_ubuntu_48.h"

bool is_configured = true;
static int configured;
#define SAMPLES_PER_MESSAGE 30
int num_samples = 0;
uint16_t samples[SAMPLES_PER_MESSAGE] __attribute__ ((aligned(2)));
//static void send_samples(uint8_t ep);
static void adc_once(void);

static enum usbd_request_return_codes gadget0_control_request(usbd_device *dev,
	struct usb_setup_data *req,
	uint8_t **buf,
	uint16_t *len,
	usbd_control_complete_callback *complete)
{
	(void) dev;
	(void) complete;
	(void) buf;
    (void) req;
    (void) len;

	return USBD_REQ_NEXT_CALLBACK;
}

//uint32_t adc_res[32];
static uint16_t adc_res[17];

#ifndef F103
void dma2_setup(void) {

    dma_stream_reset(DMA2, DMA_STREAM4);
    dma_set_priority(DMA2, DMA_STREAM4, DMA_SxCR_PL_LOW);

    dma_set_memory_size(DMA2, DMA_STREAM4, DMA_SxCR_MSIZE_16BIT);
    dma_set_peripheral_size(DMA2, DMA_STREAM4, DMA_SxCR_PSIZE_16BIT);

    dma_enable_memory_increment_mode(DMA2, DMA_STREAM4);
    dma_enable_circular_mode(DMA2, DMA_STREAM4);

    dma_set_transfer_mode(DMA2, DMA_STREAM4, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
    dma_set_peripheral_address(DMA2, DMA_STREAM4, (uint32_t) & ADC_DR(ADC1));
    dma_set_memory_address(DMA2, DMA_STREAM4, (uint32_t) &adc_res[0]);//&adc_res);
    dma_set_number_of_data(DMA2, DMA_STREAM4, 1);


    dma_channel_select(DMA2, DMA_STREAM4, DMA_SxCR_CHSEL_0);

    //nvic_enable_irq(NVIC_DMA2_STREAM4_IRQ);
    //dma_enable_transfer_complete_interrupt(DMA2, DMA_STREAM4);
    dma_enable_stream(DMA2, DMA_STREAM4);
}
#endif

void dma2_stream4_isr(void) {
    //dma_clear_interrupt_flags(DMA2, DMA_CHANNEL4, DMA_IFCR_CGIF1);
}

void dma2_stream0_isr(void) {
    //dma_clear_interrupt_flags(DMA2, DMA_CHANNEL4, DMA_IFCR_CGIF1);
}

static char ret2[] = "0.0V ";


void adc_start(void)
{
	rcc_periph_clock_enable(RCC_ADC1);


//	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO2);
	rcc_periph_clock_enable(RCC_ADC1);
	rcc_periph_clock_enable(RCC_ADC2);
	adc_set_clk_prescale(ADC_CCR_ADCPRE_BY2);
	adc_power_off(ADC1);
	adc_power_off(ADC2);
	adc_disable_scan_mode(ADC1);
//	adc_set_continuous_conversion_mode(ADC1);
//	adc_set_right_aligned(ADC1);
	adc_set_sample_time(ADC1, ADC_CHANNEL2, ADC_SMPR_SMP_3CYC);
	adc_set_resolution(ADC1, 12);
//	uint8_t channels[] = {ADC_CHANNEL2};
//	adc_set_multi_mode(ADC_CCR_MULTI_DUAL_INTERLEAVED);
//    adc_enable_dma(ADC1);
//    adc_set_dma_continue(ADC1);
	adc_power_on(ADC1);
//	adc_power_on(ADC2);
//	adc_set_regular_sequence(ADC1, 1, channels);
//	adc_start_conversion_regular(ADC1);
//    adc_res[0] = adc_read_regular(ADC1);
/*   adc_power_off(ADC1);
    adc_set_clk_prescale(ADC_CCR_ADCPRE_BY2);
    adc_enable_scan_mode(ADC1);
    adc_set_continuous_conversion_mode(ADC1);
//    adc_disable_discontinuous_mode_regular(ADC1);
//    adc_enable_external_trigger_regular(ADC1, ADC_CR2_SWSTART, ADC_CR2_EXTEN_DISABLED);
    adc_set_right_aligned(ADC1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28CYC);
    adc_set_resolution(ADC1, 12);
    uint8_t channels[] = {ADC_CHANNEL2};
    adc_set_multi_mode(ADC_CCR_MULTI_DUAL_INTERLEAVED);
    //ADC_CCR_MULTI_INDEPENDENT
    //ADC_CCR_MULTI_TRIPLE_INTERLEAVED
    adc_set_regular_sequence(ADC1, 1, channels);
    adc_enable_dma(ADC1);
    adc_set_dma_continue(ADC1);
    adc_power_on(ADC1);

    adc_start_conversion_regular(ADC1);
    */

/*
#ifdef GREENPILL
    static uint8_t channel_seq[16];
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);

    adc_set_clk_prescale(ADC_CCR_ADCPRE_BY2);

    //adc_power_off(ADC1);

    adc_enable_scan_mode(ADC1);
    adc_set_continuous_conversion_mode(ADC1);
    adc_disable_discontinuous_mode_regular(ADC1);

    //adc_enable_temperature_sensor();

    adc_enable_external_trigger_regular(ADC1, ADC_CR2_SWSTART, ADC_CR2_EXTEN_DISABLED);
    adc_set_right_aligned(ADC1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28CYC);
    adc_set_resolution(ADC1, 8);


    channel_seq[0] = ADC_CHANNEL0;
    channel_seq[1] = ADC_CHANNEL1;

    adc_set_multi_mode(ADC_CCR_MULTI_INDEPENDENT);

    adc_set_regular_sequence(ADC1, 1, channel_seq);
    adc_enable_dma(ADC1);

    adc_power_on(ADC1);

    adc_start_conversion_regular(ADC1);
#endif
*/
}

int sendsamps = 0;

static void adc_once(void)
{
//	uint8_t channel_array[16];

//    char test[32] = {0};
    static char ret[] = "0.0V ";
//    gpio_clear(GPIOC, GPIO13);
//    uint8_t channels[] = {ADC_CHANNEL2};


//	channel_array[0] = {ADC_CHANNEL2};
	uint8_t channels[] = { ADC_CHANNEL2, };
	unsigned value;

	adc_set_regular_sequence(ADC1, 1, channels);
	adc_start_conversion_regular(ADC1);
	while (!(adc_eoc(ADC1)))
	;
//	for (unsigned i = 0; i < 20; i++)
//	  {
//		__asm__("nop");
//	  }
	value = adc_read_regular(ADC1);
//    value = adc_res[0];
	value *= 3379; /* 3.3 * 1024 == 3379.2 */
	value += 104858; /* round, 0.05V * 2 ^ 21 == 104857.6 */
	ret[0] = (value >> 21) + '0';
	value &= (1 << 21) - 1;
	value *= 10;
	ret[2] = (value >> 21) + '0';

    strcpy(ret2, ret);
//    printf("adc_res 1 and 2 %4lu  %4lu \n", adc_res[0], adc_res[1]);
//   st_fill_rect(1, 110, 74, 60, ST_COLOR_YELLOW);
//   st_draw_string(10, 65, "Target IO", ST_COLOR_DARKGREEN, &font_ubuntu_48);
//	st_draw_string(10, 110, ret, ST_COLOR_RED, &font_ubuntu_48);
	
	
    samples[num_samples++] = (uint32_t)ret;
    if (num_samples == SAMPLES_PER_MESSAGE) {
//        send_samples(ep);
 //       num_samples = 0;
    return;
    }
}


/*
static void adc_once(void)
{
unsigned value;
unsigned value2;
static char ret[] = "0.0V ";
static char ret2[] = "0.0V ";
value = adc_res[0];
value2 = adc_res[1];
	value *= 3379; // 3.3 * 1024 == 3379.2  
	value += 104858; // round, 0.05V * 2 ^ 21 == 104857.6  
	ret[0] = (value >> 21) + '0';
	value &= (1 << 21) - 1;
	value *= 10;
	ret[2] = (value >> 21) + '0';
	
	value2 *= 3379; // 3.3 * 1024 == 3379.2  
	value2 += 104858; // round, 0.05V * 2 ^ 21 == 104857.6  
	ret2[0] = (value >> 21) + '0';
	value2 &= (1 << 21) - 1;
	value2 *= 10;
	ret2[2] = (value >> 21) + '0';
	
	
	samples[0] = (uint32_t)ret;
	samples[1] = (uint32_t)ret2;
	return;
}
*/

static void gadget0_ss_out_cb(usbd_device *dev, uint8_t ep)
{
	(void) ep;
	(void)dev;
	uint16_t x;
	/* TODO - if you're really keen, perf test this. tiva implies it matters */
	/* char buf[64] __attribute__ ((aligned(4))); */
	uint8_t buf[BULK_EP_MAXPACKET + 1] __attribute__ ((aligned(2)));
	uint8_t *dest;


		dest = buf;

	x = usbd_ep_read_packet(usbdev, ep, dest, BULK_EP_MAXPACKET);
    if (x) {
        ;
        }
}


static void gadget0_ss_in_cb(usbd_device *dev, uint8_t ep)
{
	(void) dev;
//	uint8_t buf[BULK_EP_MAXPACKET + 1] __attribute__ ((aligned(2)));
//	uint8_t *src;


    adc_once();

/*
#ifdef GREENPILL
    uint16_t x = usbd_ep_write_packet(usbdev, ep,&samples, sizeof(samples));
    if (x != BULK_EP_MAXPACKET) {
	;
	}
*/
    uint16_t x = usbd_ep_write_packet(usbdev, ep,&ret2, sizeof(ret2));
    if (x != BULK_EP_MAXPACKET) {
	;
	}

	if (num_samples == SAMPLES_PER_MESSAGE) {
//        send_samples(ep);
    num_samples = 0;
    }

	//assert(x == sizeof(buf));

}

void usbadc_set_config(usbd_device *dev, uint16_t wValue)
{
//	(void)wValue;
    configured = wValue;

		usbd_ep_setup(dev, 0x03, USB_ENDPOINT_ATTR_BULK, BULK_EP_MAXPACKET,
			gadget0_ss_out_cb);
		usbd_ep_setup(dev, 0x83, USB_ENDPOINT_ATTR_BULK, BULK_EP_MAXPACKET,
			gadget0_ss_in_cb);
		usbd_register_control_callback(
			dev,
			USB_REQ_TYPE_VENDOR | USB_REQ_TYPE_INTERFACE,
			USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
			gadget0_control_request);
		/* Prime source for IN data. */
		gadget0_ss_in_cb(usbdev, 0x83);
}
/*
static void send_samples(uint8_t ep)
{

    usbd_ep_write_packet(usbdev, ep,&samples, sizeof(samples));
//     usbd_ep_write_packet(usbdev, 0x80 | ep, buf, x);
}
*/
