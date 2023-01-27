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
	@echo $(obj-m:.o=) > dkms.list
	$(INSTALL) -D $(C_SRC_FILES) $(MODULE_MAKE_FILE) -t $(INSTALL_MOD_PATH)

clean: 
	rm -f ${unipi-objs} src/.*.o.cmd src/*.o src/.*.o.d
	rm -f ${UNIPI_KERNEL_MODULES} $(obj-m:.o=.mod.c) $(obj-m:.o=.mod) .*.cmd *.o modules.order Module.symvers
	rm -rf .tmp_versions
