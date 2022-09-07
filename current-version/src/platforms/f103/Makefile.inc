CROSS_COMPILE ?= arm-none-eabi-
ST_BOOTLOADER ?=
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy

OPT_FLAGS = -Os
CFLAGS += -mcpu=cortex-m3 -mthumb -DSTM32F1 -I../libopencm3/include \
	-I platforms/stm32
LDFLAGS_BOOT := $(LDFLAGS) --specs=nano.specs -lopencm3_stm32f1 \
	-Wl,-T,platforms/stm32/badassf103.ld -nostartfiles -lc \
	-Wl,-Map=mapfile -mthumb -mcpu=cortex-m3 -Wl,-gc-sections \
	-L../libopencm3/lib

CFLAGS += -DDFU_SERIAL_LENGTH=9
LDFLAGS = $(LDFLAGS_BOOT) -Wl,-Ttext=0x8000000


ifeq ($(ENABLE_DEBUG), 1)
LDFLAGS += --specs=rdimon.specs
else
LDFLAGS += --specs=nosys.specs
endif

ifeq ($(SWIM_AS_UART), 1)
CFLAGS += -DSWIM_AS_UART=1
endif
CFLAGS += -DF103

VPATH += platforms/stm32

SRC +=          \
	usbuart.c 	\
	serialno.c	\
	timing.c	\
	timing_stm32.c	\
    st7789_stm32_spi.c \
    SWIM.c \
    usb-swim.c \
    

all:	blackmagic.bin



host_clean:
	-$(Q)$(RM) *.bin *elf *hex
