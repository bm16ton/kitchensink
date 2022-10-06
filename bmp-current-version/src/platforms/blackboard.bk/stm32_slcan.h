#ifndef STM32_SLCAN_H
#define STM32_SLCAN_H

#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/usb/dfu.h>

#include "usb.h"

void slcan_init(void);
void slcan_usb_out_cb(usbd_device *dev, uint8_t ep);
void slcan_set_config(usbd_device *dev, uint16_t wValue);

#endif
