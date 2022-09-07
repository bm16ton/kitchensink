not finished switched to ii9341 but the files and most code are still around (coommented) for st7789 sh1106 and ssd1306;

currently at boot it checks a button on A0 for f4s and i dont remember for f1 check code and
either becomes a reg blackmagic probe or usb2i2c "i2c-star" plus usb 2 gpio with irq support and another interface for usb 2 gpio.

kernel drivers;
A couple drivers existt for gpio/irq but here i think i added the driver that combines the gpio driver withe usb to adc driver.It works ok until driver or usb device is removed then hellbreaks loose, the stand alone usb to adc driver  works fine even on removal.

usb 2 adc 
just run;
sudo cat /dev/adc-wmagic0
and receive binary adc data

all the pieces are hee and have been tested for the swim on f103s but didnt fel like adding another button for boot switching (had just temp  replaced the usb gpio/adc stuff in usb.c and usb_descriptors2.h)

hopefully soon ill get it cleaned up and a proper readme
