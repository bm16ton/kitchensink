#ifndef USB_ADC_H
#define USB_ADC_H

#include <stdint.h>
#include <stdbool.h>
#include "usb.h"

#define BULK_EP_MAXPACKET 64

void adc_start(void);

void usbadc_set_config(usbd_device *dev, uint16_t wValue);
void dma2_setup(void);

#endif
