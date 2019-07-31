###
### devicetree overlay for unipispi board on Raspberry Pi
###

LINUX_DIR_PATH = /lib/modules/$(shell uname -r)/build
ifdef CCPREFIX
    CCPAR = CROSS_COMPILE=${CCPREFIX}
endif
ifdef ARCH
    CCPAR += ARCH=${ARCH}
endif

PWD := $(shell pwd)

dtbo-y += neuron-spi-new.dtbo

targets += $(dtbo-y)
always  := $(dtbo-y)


all:
	make $(CCPAR) -C $(LINUX_DIR_PATH) M=$(PWD)

clean:
	make $(CCPAR) -C $(LINUX_DIR_PATH) M=$(PWD) clean


#DTC = dtc -O dtb -@

#%.dtbo: %.dts
#	$(DTC) -o $@ $<

#install: clean neuron-spi-new.dtbo
#	$(shell mount -o remount,rw /boot) \
#	$(shell cp neuron-spi-new.dtbo /boot/overlays)

