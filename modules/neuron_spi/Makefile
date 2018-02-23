# Note: Compiling kernel modules requires creating symlinks, which is not possible on certain 
# filesystems (notably VirtualBox vmfs); therefore we allow using /run/ through the 'symlink' target,
# if necessary.
 
SYMLINK_DIR_PATH = /run/kernel/neuron_spi
LINUX_DIR_PATH = /root/bokula/linux
SRC_DIR_PATH = $(PWD)/src
BIN_DIR_PATH = $(PWD)/bin

MODULE_MAKE_FILE = Makefile

C_SRC_FILES = unipi_spi.c
C_SRC_FILES += unipi_iio.c
C_SRC_FILES += unipi_gpio.c
C_SRC_FILES += unipi_uart.c
C_SRC_FILES += unipi_sysfs.c

H_SRC_FILES = unipi_spi.h
H_SRC_FILES += unipi_iio.h
H_SRC_FILES += unipi_gpio.h
H_SRC_FILES += unipi_uart.h
H_SRC_FILES += unipi_sysfs.h

OBJ_FILES = src/unipi_spi.o
OBJ_FILES += src/unipi_iio.o
OBJ_FILES += src/unipi_gpio.o
OBJ_FILES += src/unipi_uart.o
OBJ_FILES += src/unipi_sysfs.o

KERNEL_MODULE_NAME = unipi
obj-m += ${KERNEL_MODULE_NAME}.o
unipi-objs := ${OBJ_FILES}

TARGET_PLC_PATH = tomunipi:/root/

.PHONY: default
default: symlink ;

all: 
	make ARCH=arm CROSS_COMPILE=${CCPREFIX} -C ${LINUX_DIR_PATH} M=${PWD} modules

clean: 
	make -C ${LINUX_DIR_PATH} M=$(PWD) clean
	rm -f ${BIN_DIR_PATH}/${KERNEL_MODULE_NAME}.ko

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
