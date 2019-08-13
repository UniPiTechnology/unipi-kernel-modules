# Note: Compiling kernel modules requires creating symlinks, which is not possible on certain 
# filesystems (notably VirtualBox vmfs); therefore we allow using /run/ through the 'symlink' target,
# if necessary.
 
MODULES_DIR_PATH = ${PWD}/modules/
MODULES_LIST = unipi/ rtc-unipi/

.PHONY: default
#default: symlink ;
default: all ;

all: 
	for m in ${MODULES_LIST}; do\
		cd ${MODULES_DIR_PATH}$$m; make all;\
		done

modules_install: install ;

install: 
	for m in ${MODULES_LIST}; do\
		cd ${MODULES_DIR_PATH}$$m; make modules_install INSTALL_MOD_PATH=${DESTDIR};\
		done

dkms:
	for m in ${MODULES_LIST}; do\
		cd ${MODULES_DIR_PATH}$$m; make dkms INSTALL_MOD_PATH=${DESTDIR}/$$m;\
		done

clean: 
	for m in ${MODULES_LIST}; do\
		cd ${MODULES_DIR_PATH}$$m; make clean;\
		done

