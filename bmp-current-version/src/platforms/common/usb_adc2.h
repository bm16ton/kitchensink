#ifndef USB_ADC2_H
#define USB_ADC2_H

#include <stdint.h>
#include <stdbool.h>
#include "usb.h"

#define BULK_EP_MAXPACKET 64

void adc2_start(void);

void usbadc2_set_config(usbd_device *dev, uint16_t wValue);

#endif
