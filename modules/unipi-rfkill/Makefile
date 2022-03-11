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

#EXTRA_CFLAGS=-I$(PWD)/../../unipi-hardware-id/include

MODULE_MAKE_FILE = Makefile
INSTALL = install

#### skip making for old kernel 4.x 
DOIT = $(shell grep -qse "^4\." include/config/kernel.release || echo 1)
ifeq ($(DOIT), 1)

C_SRC_FILES += src/unipi_rfkill.c
#H_SRC_FILES += src/unipi_rfkill.h
OBJ_FILES  = $(C_SRC_FILES:.c=.o)
obj-m += unipi_rfkill.o
UNIPI_KERNEL_MODULES = $(obj-m:.o=.ko)
unipi_rfkill-objs := ${OBJ_FILES}

endif

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
	rm -f ${unipi-objs} src/.*.o.cmd src/*.o src/.*.o.d
	rm -f ${UNIPI_KERNEL_MODULES} $(obj-m:.o=.mod.c) $(obj-m:.o=.mod) .*.cmd *.o modules.order Module.symvers
	rm -rf .tmp_versions

