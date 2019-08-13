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

C_SRC_FILES = 8250_dwunipi.c

OBJ_FILES = 8250_dwunipi.o

KERNEL_MODULE_NAME = dw-apb-485
obj-m += ${KERNEL_MODULE_NAME}.o
dw-apb-485-objs := ${OBJ_FILES}

.PHONY: default
default: all ;

all: 
	make $(CCPAR) -C $(LINUX_DIR_PATH) M=${PWD} modules

modules_install: 
	make $(CCPAR) -C $(LINUX_DIR_PATH) M=${PWD} modules_install

clean: 
	rm -f ${rtc-unipi-objs} .*.o.cmd
	rm -f ${KERNEL_MODULE_NAME}.ko ${KERNEL_MODULE_NAME}.mod.c .*.o.cmd .*.ko.cmd *.o modules.order Module.symvers
	rm -rf .tmp_versions
