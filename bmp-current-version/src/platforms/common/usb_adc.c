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

bool is_configured = true;
static int configured;
#define SAMPLES_PER_MESSAGE 10
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

void adc_start(void)
{
	rcc_periph_clock_enable(RCC_ADC1);

#ifdef BLACKPILLV2
//	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO2);

	adc_power_off(ADC1);
	adc_disable_scan_mode(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_3CYC);
    adc_set_single_conversion_mode(ADC1);
    adc_set_right_aligned(ADC1);
	adc_power_on(ADC1);
	uint8_t channels[] = {ADC_CHANNEL2};
    adc_set_regular_sequence(ADC1, 1, channels);
#endif

#ifdef F103	
    adc_power_off(ADC1);

    // configure for regular single conversion
    adc_disable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);
    adc_disable_external_trigger_regular(ADC1);
    adc_set_right_aligned(ADC1);

    // power up
    adc_power_on(ADC1);
    delay(100);
    adc_reset_calibration(ADC1);
    adc_calibrate(ADC1);

    // configure A0 as ADC channel
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO1);
    uint8_t channels[] = {ADC_CHANNEL1};
    adc_set_regular_sequence(ADC1, 1, channels);
#endif
}

#ifdef F103
static void adc_once_f103(uint8_t ep)
{
    gpio_set(GPIOC, GPIO1);
    adc_start_conversion_direct(ADC1);
    while (!(adc_eoc(ADC1)))
        ;
    uint32_t value = adc_read_regular(ADC1);

    samples[num_samples++] = (uint16_t)value;
    if (num_samples == SAMPLES_PER_MESSAGE) {
//        send_samples(ep);
//        num_samples = 0;
    return;
    }
}
#endif

#ifndef F103
static void adc_once(void)
{
//	uint8_t channel_array[16];
#ifdef BLACKPILLV2
    gpio_clear(GPIOC, GPIO13);
    uint8_t channels[] = {ADC_CHANNEL2};
#endif 

#ifdef STLINKV3
uint8_t channels[] = { ADC_CHANNEL0, };
#endif
//	channel_array[0] = {ADC_CHANNEL2};
	adc_set_regular_sequence(ADC1, 1, channels);
	adc_start_conversion_regular(ADC1);
    while (!(adc_eoc(ADC1)))
        ;
        
    uint32_t value = adc_read_regular(ADC1);

    samples[num_samples++] = (uint16_t)value;
    if (num_samples == SAMPLES_PER_MESSAGE) {
//        send_samples(ep);
 //       num_samples = 0;
    return;
    }
}
#endif

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

#ifdef F103   
    adc_once_f103(ep);
#else
    adc_once();
#endif
   
    uint16_t x = usbd_ep_write_packet(usbdev, ep,&samples, sizeof(samples));
    if (x != BULK_EP_MAXPACKET) {
	;
	}

	if (num_samples == SAMPLES_PER_MESSAGE) {
//        send_samples(ep);
    num_samples = 0;
    }
	//assert(x == sizeof(buf));
#ifdef F103 	
	    gpio_clear(GPIOC, GPIO1);
#endif
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
