#ifndef USB_I2C_H
#define USB_I2C_H

#include <stdint.h>
#include <stdbool.h>
#include "usb.h"

void i2c_init(void);
void i2c_init_f103(void);
void usb_set_config(usbd_device *dev, uint16_t wValue);

#endif
