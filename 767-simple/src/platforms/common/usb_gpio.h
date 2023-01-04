#ifndef USB_GPIO_H
#define USB_GPIO_H

#include <stdint.h>
#include <stdbool.h>
#include "usb.h"

void usbgpio_init(void);
void gpio_set_config(usbd_device *usbd_dev, uint16_t wValue);

#endif
