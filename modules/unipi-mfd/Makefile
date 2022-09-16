# Note: Compiling kernel modules requires creating symlinks, which is not possible on certain 
# filesystems (notably VirtualBox vmfs); therefore we allow using /run/ through the 'symlink' target,
# if necessary.
 
LINUX_DIR_PATH = /lib/modules/$(shell uname -r)/build
ifdef CCPREFIX
    CCPAR = CROSS_COMPILE=${CCPREFIX}
endif
ifdef ARCH
    CCPAR += ARCH=${ARCH}
endif

EXTRA_CFLAGS=-I$(PWD)/../../unipi-hardware-id/include

MODULE_MAKE_FILE = Makefile
INSTALL = install

C_SRC_FILES = src/unipi_spi.c
C_SRC_FILES += src/unipi_spi_op.c
C_SRC_FILES += src/unipi_spi_op2.c
C_SRC_FILES += src/unipi_spi_firmware.c
C_SRC_FILES += src/unipi_regmap.c
C_SRC_FILES += src/unipi_channel.c

C_SRC_FILES1 += src/unipi_iogroup_bus.c
C_SRC_FILES1 += src/unipi_modbus.c
C_SRC_FILES1 += src/unipi_dali.c
C_SRC_FILES1 += src/unipi_gpio.c
C_SRC_FILES1 += src/unipi_iio.c
C_SRC_FILES1 += src/unipi_iio_legacy.c
C_SRC_FILES1 += src/unipi_leds.c
C_SRC_FILES1 += src/unipi_mfd.c
C_SRC_FILES1 += src/unipi_tty.c
C_SRC_FILES1 += src/unipi_uart.c

#C_SRC_FILES1 += src/unipi_plc.c
#C_SRC_FILES3 += src/unipi_leds.c

H_SRC_FILES = src/unipi_spi.h
H_SRC_FILES += src/unipi_spi_crc.h
H_SRC_FILES += src/unipi_common.h
H_SRC_FILES += src/unipi_iogroup_bus.h
H_SRC_FILES += src/unipi_channel.h
H_SRC_FILES += src/unipi_modbus.h
H_SRC_FILES += src/unipi_mfd.h
H_SRC_FILES += src/unipi_tty.h
H_SRC_FILES += src/unipi_uart.h

OBJ_FILES  = $(C_SRC_FILES:.c=.o)


#KERNEL_MODULE_NAME = unipi
obj-m += unipi.o
obj-m += unipi_iogroup_bus.o
obj-m += unipi_leds.o
obj-m += unipi_gpio.o
obj-m += unipi_iio.o
obj-m += unipi_iio_legacy.o
obj-m += unipi_uart.o
obj-m += unipi_mfd.o
obj-m += unipi_modbus.o
obj-m += unipi_dali.o

UNIPI_KERNEL_MODULES = $(obj-m:.o=.ko)

unipi-objs := ${OBJ_FILES}
unipi_iogroup_bus-objs := src/unipi_iogroup_bus.o
unipi_leds-objs := src/unipi_leds.o
unipi_gpio-objs := src/unipi_gpio.o
unipi_iio-objs := src/unipi_iio.o
unipi_iio_legacy-objs := src/unipi_iio_legacy.o
unipi_uart-objs := src/unipi_uart.o src/unipi_tty.o
unipi_mfd-objs := src/unipi_mfd.o
unipi_modbus-objs := src/unipi_modbus.o
unipi_dali-objs := src/unipi_dali.o

KBUILD_EXTRA_SYMBOLS = $(KBUILD_EXTMOD)/../unipi-id/Module.symvers

.PHONY: default
default: all ;

all: 
	$(MAKE) $(CCPAR) -C $(LINUX_DIR_PATH) M=${PWD} modules

modules_install: 
	$(MAKE) $(CCPAR) -C $(LINUX_DIR_PATH) M=${PWD} modules_install

dkms:
	@echo $(obj-m:.o=) > dkms.list
	$(INSTALL) -D $(MODULE_MAKE_FILE) -t $(INSTALL_MOD_PATH)
	$(INSTALL) -D $(C_SRC_FILES) $(C_SRC_FILES1) $(H_SRC_FILES) -t $(INSTALL_MOD_PATH)/src

clean: 
	rm -f ${unipi-objs} src/.*.o.cmd src/*.o src/.*.o.d
	rm -f ${UNIPI_KERNEL_MODULES} $(obj-m:.o=.mod.c) $(obj-m:.o=.mod) .*.cmd *.o modules.order Module.symvers
	rm -rf .tmp_versions

