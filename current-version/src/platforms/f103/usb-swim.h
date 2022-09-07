#ifndef USB_SWIM_H
#define USB_SWIM_H

void stlink_set_config(usbd_device *usbd_dev, uint16_t wValue);
void stlink_run(void);

#endif
