# Note: Compiling kernel modules requires creating symlinks, which is not possible on certain 
# filesystems (notably VirtualBox vmfs); therefore we allow using /run/ through the 'symlink' target,
# if necessary.

MODULES_DIR_PATH = ${PWD}/modules/
MODULES_LIST = rtc-unipi/ unipi-id/ unipi-rfkill/ unipi-mfd/

.PHONY: default
#default: symlink ;
default: all ;

all:
	for m in ${MODULES_LIST}; do\
		cd ${MODULES_DIR_PATH}$$m; $(MAKE) all || exit 1;\
		done

modules_install: install ;

install:
	for m in ${MODULES_LIST}; do\
		cd ${MODULES_DIR_PATH}$$m; $(MAKE) modules_install INSTALL_MOD_PATH=${DESTDIR} || exit 1;\
		done

dkms:
	for m in ${MODULES_LIST}; do\
		cd ${MODULES_DIR_PATH}$$m; $(MAKE) dkms INSTALL_MOD_PATH=${DESTDIR}/$$m || exit 1;\
		done
	i=0; for m in ${MODULES_LIST}; do\
		for module in `cat ${MODULES_DIR_PATH}$$m/dkms.list`; do\
			echo "BUILT_MODULE_NAME[$${i}]=$${module}";\
			echo "BUILT_MODULE_LOCATION[$${i}]=$${m%/}";\
			echo "DEST_MODULE_LOCATION[$${i}]=/extra";\
			echo "";\
			i=$$((i+1));\
			done;\
		done > dkms.conf
	@cat dkms.conf

clean:
	for m in ${MODULES_LIST}; do\
		cd ${MODULES_DIR_PATH}$$m; $(MAKE) clean;\
		done
