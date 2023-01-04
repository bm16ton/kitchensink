#ifndef USB_ADC_H
#define USB_ADC_H

#include <stdint.h>
#include <stdbool.h>
#include "usb.h"
#include "maxpacket.h"


void adc_start(void);

void usbadc_set_config(usbd_device *dev, uint16_t wValue);
void dma2_setup(void);

#endif
