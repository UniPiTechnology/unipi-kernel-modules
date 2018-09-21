# Note: Compiling kernel modules requires creating symlinks, which is not possible on certain 
# filesystems (notably VirtualBox vmfs); therefore we allow using /run/ through the 'symlink' target,
# if necessary.
 
SYMLINK_DIR_PATH = /run/kernel/unipi_spi
LINUX_DIR_PATH = /root/linux/4_9_14/linux/
SRC_DIR_PATH = $(PWD)/src
BIN_DIR_PATH = $(PWD)/bin

MODULE_MAKE_FILE = Makefile

C_SRC_FILES = unipi_spi.c
C_SRC_FILES += unipi_iio.c
C_SRC_FILES += unipi_gpio.c
C_SRC_FILES += unipi_uart.c
C_SRC_FILES += unipi_sysfs.c
C_SRC_FILES += unipi_misc.c
C_SRC_FILES += unipi_platform.c
C_SRC_FILES += unipi_tty.c

H_SRC_FILES = unipi_spi.h
H_SRC_FILES += unipi_iio.h
H_SRC_FILES += unipi_gpio.h
H_SRC_FILES += unipi_uart.h
H_SRC_FILES += unipi_sysfs.h
H_SRC_FILES += unipi_misc.h
H_SRC_FILES += unipi_platform.h
H_SRC_FILES += unipi_common.h
H_SRC_FILES += unipi_tty.h

OBJ_FILES = src/unipi_spi.o
OBJ_FILES += src/unipi_iio.o
OBJ_FILES += src/unipi_gpio.o
OBJ_FILES += src/unipi_uart.o
OBJ_FILES += src/unipi_sysfs.o
OBJ_FILES += src/unipi_misc.o
OBJ_FILES += src/unipi_platform.o
OBJ_FILES += src/unipi_tty.o

KERNEL_MODULE_NAME = unipi
obj-m += ${KERNEL_MODULE_NAME}.o
unipi-objs := ${OBJ_FILES}

TARGET_PLC_PATH = tomunipi:

.PHONY: default
default: all ;

all: 
	make ARCH=arm CROSS_COMPILE=${CCPREFIX} -C ${LINUX_DIR_PATH} M=${PWD} modules

modules_install: 
	make ARCH=arm CROSS_COMPILE=${CCPREFIX} -C ${LINUX_DIR_PATH} M=${PWD} modules_install

clean: 
	rm -f ${unipi-objs} src/.*.o.cmd
	rm -f ${KERNEL_MODULE_NAME}.ko ${KERNEL_MODULE_NAME}.mod.c .*.o.cmd .*.ko.cmd *.o modules.order Module.symvers
	rm -rf .tmp_versions

transfer: clean symlink
	scp ${BIN_DIR_PATH}/${KERNEL_MODULE_NAME}.ko ${TARGET_PLC_PATH}
	
symlink: clean
	rm -r -f ${SYMLINK_DIR_PATH}
	mkdir -p ${SYMLINK_DIR_PATH}/src
	mkdir -p ${SYMLINK_DIR_PATH}/bin
	cp ${PWD}/${MODULE_MAKE_FILE} ${SYMLINK_DIR_PATH} 
	for f in ${C_SRC_FILES}; do\
		ln -s ${SRC_DIR_PATH}/$$f ${SYMLINK_DIR_PATH}/src ;\
		done
	for f in ${H_SRC_FILES}; do\
		ln -s ${SRC_DIR_PATH}/$$f ${SYMLINK_DIR_PATH}/src ;\
		done
	cd ${SYMLINK_DIR_PATH}; make all
	mv ${SYMLINK_DIR_PATH}/${KERNEL_MODULE_NAME}.ko ${BIN_DIR_PATH}
	rm -r -f ${SYMLINK_DIR_PATH}
