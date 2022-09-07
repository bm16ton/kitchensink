#ifndef __CDCACM_H
#define __CDCACM_H

#include <libopencm3/usb/usbd.h>

#if defined(USB_HS)
# define CDCACM_PACKET_SIZE    512
#else
# define CDCACM_PACKET_SIZE     64
#endif

#if !defined(MAX_BINTERVAL)
# define MAX_BINTERVAL 255
#endif

#define TRACE_ENDPOINT_SIZE2 CDCACM_PACKET_SIZE

/* Use platform provided value if given. */
#if !defined(TRACE_ENDPOINT_SIZE2)
# define TRACE_ENDPOINT_SIZE2 CDCACM_PACKET_SIZE
#endif


#define CDCACM_GDB_ENDPOINT2	1
#define CDCACM_UART_ENDPOINT2	3
#define TRACE_ENDPOINT2			5
#define CDCACM_SLCAN_ENDPOINT2	6

//extern usbd_device *usbdev;

void cdcacm_init(void);
/* Returns current usb configuration, or 0 if not configured. */
int cdcacm_get_config(void);
int cdcacm_get_dtr(void);

#endif
