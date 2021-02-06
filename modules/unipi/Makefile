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


MODULE_MAKE_FILE = Makefile
INSTALL = install

C_SRC_FILES = src/unipi_spi.c
C_SRC_FILES += src/unipi_iio.c
C_SRC_FILES += src/unipi_gpio.c
C_SRC_FILES += src/unipi_uart.c
C_SRC_FILES += src/unipi_sysfs.c
C_SRC_FILES += src/unipi_misc.c
C_SRC_FILES += src/unipi_platform.c
C_SRC_FILES += src/unipi_tty.c

H_SRC_FILES = src/unipi_spi.h
H_SRC_FILES += src/unipi_iio.h
H_SRC_FILES += src/unipi_gpio.h
H_SRC_FILES += src/unipi_uart.h
H_SRC_FILES += src/unipi_sysfs.h
H_SRC_FILES += src/unipi_misc.h
H_SRC_FILES += src/unipi_platform.h
H_SRC_FILES += src/unipi_common.h
H_SRC_FILES += src/unipi_tty.h

OBJ_FILES  = $(C_SRC_FILES:.c=.o)
#OBJ_FILES = src/unipi_spi.o
#OBJ_FILES += src/unipi_iio.o
#OBJ_FILES += src/unipi_gpio.o
#OBJ_FILES += src/unipi_uart.o
#OBJ_FILES += src/unipi_sysfs.o
#OBJ_FILES += src/unipi_misc.o
#OBJ_FILES += src/unipi_platform.o
#OBJ_FILES += src/unipi_tty.o

KERNEL_MODULE_NAME = unipi
obj-m += ${KERNEL_MODULE_NAME}.o
unipi-objs := ${OBJ_FILES}

.PHONY: default
default: all ;

all: 
	$(MAKE) $(CCPAR) -C $(LINUX_DIR_PATH) M=${PWD} modules

modules_install: 
	$(MAKE) $(CCPAR) -C $(LINUX_DIR_PATH) M=${PWD} modules_install

dkms:
	$(INSTALL) -D $(MODULE_MAKE_FILE) -t $(INSTALL_MOD_PATH)
	$(INSTALL) -D $(C_SRC_FILES) $(H_SRC_FILES) -t $(INSTALL_MOD_PATH)/src

clean: 
	rm -f ${unipi-objs} src/.*.o.cmd
	rm -f ${KERNEL_MODULE_NAME}.ko ${KERNEL_MODULE_NAME}.mod.c .*.o.cmd .*.ko.cmd *.o modules.order Module.symvers
	rm -rf .tmp_versions

