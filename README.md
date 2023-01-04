update: Now using ulpi usbhs, usb2adc now can be switched between sending raw (fast) or humanreadable (slow) via sysfs file "humanreadable". Probly more changes I dont remember.
KNOWN ISSUES SINCE SWITCHING TO USB-HS;
No more i2c2 or spi2 both had pins required by ulpi.
Auto usb dfu doesnt work since dfu only works on usb-fs.
USB-2-SERIAL doesnt work buffers need to be thought out and aligned.


updates; switched to ili9486 16bit spi shifter type like the waveshare rpi models. all is good thus far, the f407 actually have 1024k storage not its reported 512k so the onscreen button for canbus now jumps to memory address 512k im currently placing pecan firmware , but have tested very basic patch for arduino to also flash to 512k and that works too. ill place the pecan firmware i edited for 512k offset sumwhere in this source. Also wrote using adafruit seesaw driver for neopixels so i can reclaim spi port and finally use have a purpose for my seesaw (checkout my seesaw linux kernel driver for adc pwm on seesaw) and I dont remember what else its been to long without updating readme.
not finished switched to ii9341 but the files and most code are still around (coommented) for st7789 sh1106 and ssd1306;

currently at boot it checks a button on A0 for f4s and i dont remember for f1 check code and
either becomes a reg blackmagic probe or usb2i2c "i2c-star" plus usb 2 gpio with irq support and another interface for usb 2 gpio.

kernel drivers;
A couple drivers existt for gpio irq but here i think i added the driver that combines the gpio driver withe usb to adc driver.It works ok until driver or usb device is removed then hellbreaks loose, the stand alone usb to adc driver  works fine even on removal.

usb 2 adc 
just run;
sudo cat /dev/adc-wmagic0
and receive binary adc data

all the pieces are hee and have been tested for the swim on f103s but didnt fel like adding another button for boot switching (had just temp  replaced the usb gpio/adc stuff in usb.c and usb_descriptors2.h)

hopefully soon ill get it cleaned up and a proper readme


