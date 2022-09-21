#
# This file is part of the Black Magic Debug project.
#
# Copyright (C) 2022 1BitSquared <info@1bitsquared.com>
# Written by Rachel Mant <git@dragonmux.network>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

VPATH += platforms/common
VPATH += platforms/blackboard
CFLAGS += -Iplatforms/common -Iinclude -Iplatforms/blackboard

SRC +=             \
	jtagtap.c      \
	swdptap.c       \
	usb.c          \
	usb_serial.c   \
	usb_serial2.c   \
	usb_dfu_stub.c \
	usb_i2c.c \
    util.c \
    regdump.c \
    bitops.c \
    pwm.c \
    usb_adc.c \

    
ifeq ($(STLINKV3), 1)
SRC += stlinkv3.c \

else
SRC += i2c_ctx.c \
    usb_gpio.c \
    
endif
