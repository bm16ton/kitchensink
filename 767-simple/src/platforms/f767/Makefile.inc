CROSS_COMPILE ?= arm-none-eabi-
BMP_BOOTLOADER ?=
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

CFLAGS += -Istm32/include -mcpu=cortex-m7 -mthumb \
	-mfloat-abi=hard -mfpu=fpv5-sp-d16 \
	-DSTM32F7 -I../libopencm3/include \
	-Iplatforms/stm32 

LINKER_SCRIPT=platforms/stm32/f767.ld

LDFLAGS_BOOT = -lopencm3_stm32f7 \
	-Wl,-T,$(LINKER_SCRIPT) -nostartfiles -lc -lnosys \
	-Wl,-Map=mapfile -mthumb -mcpu=cortex-m7 -Wl,-gc-sections \
	-mfloat-abi=hard -mfpu=fpv5-sp-d16 \
	-L../libopencm3/lib

LDFLAGS += $(LDFLAGS_BOOT)
CFLAGS += -DDFU_SERIAL_LENGTH=13


CFLAGS += -DBLACKPILLV2
#CFLAGS += -DENABLE_NEOPIXEL

VPATH += platforms/stm32

SRC +=               \
	serialno.c	\
	timing.c	\
	timing_stm32.c	\
    st7789_stm32_spi.c \
    xpt2046.c \

    
ifneq ($(BMP_BOOTLOADER), 1)
all:	blackmagic.bin
else
all:	blackmagic.bin  blackmagic_dfu.bin blackmagic_dfu.hex
blackmagic_dfu: usbdfu.o dfucore.o dfu_f4.o serialno.o
	$(CC) $^ -o $@ $(LDFLAGS_BOOT)

blackmagic_dfu.bin:    blackmagic_dfu
	$(OBJCOPY) -O binary $^ $@

blackmagic_dfu.hex:    blackmagic_dfu
	$(OBJCOPY) -O ihex $^ $@
endif
host_clean:
	-$(Q)$(RM) blackmagic.bin

