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

C_SRC_FILES = rtc-unipi.c

OBJ_FILES  = $(C_SRC_FILES:.c=.o)

KERNEL_MODULE_NAME = rtc-unipi
obj-m += ${KERNEL_MODULE_NAME}.o
#rtc-unipi-objs := ${OBJ_FILES}

.PHONY: default
default: all ;

all: 
	$(MAKE) $(CCPAR) -C $(LINUX_DIR_PATH) M=${PWD} modules

modules_install: 
	$(MAKE) $(CCPAR) -C $(LINUX_DIR_PATH) M=${PWD} modules_install

dkms:
	$(INSTALL) -D $(C_SRC_FILES) $(MODULE_MAKE_FILE) -t $(INSTALL_MOD_PATH)

clean: 
	rm -f ${rtc-unipi-objs} .*.o.cmd
	rm -f ${KERNEL_MODULE_NAME}.ko ${KERNEL_MODULE_NAME}.mod.c .*.o.cmd .*.ko.cmd *.o modules.order Module.symvers
	rm -rf .tmp_versions
