#ifndef STLINKV3_H
#define STLINKV3_H

#include <stdint.h>
#include <stdbool.h>
#include "usb.h"

void cdcacm_set_config(usbd_device *dev, uint16_t wValue);
void platform_nrst_set_val(bool assert);
bool platform_nrst_get_val();
#define GDB_IF_NO   0
#define UART_IF_NO  2
#define DFU_IF_NO   4
#if defined(PLATFORM_HAS_SLCAN)
# define TRACE_IF_NO 5
# define SLCAN_IF_NO 6
# define TOTAL_INTERFACES  8
#elif defined(PLATFORM_HAS_TRACESWO)
# define TRACE_IF_NO 5
# define TOTAL_INTERFACES  6
#else
# define TOTAL_INTERFACES  5
#endif

#endif
