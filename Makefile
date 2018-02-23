# Note: Compiling kernel modules requires creating symlinks, which is not possible on certain 
# filesystems (notably VirtualBox vmfs); therefore we allow using /run/ through the 'symlink' target,
# if necessary.
 
MODULES_DIR_PATH = ${PWD}/modules/
MODULES_LIST = neuron_spi/
TRANSFER_MODULE = neuron_spi/

.PHONY: default
default: symlink ;

all: 
	for m in ${MODULES_LIST}; do\
		cd ${MODULES_DIR_PATH}$$m; make all;\
		done

clean: 
	for m in ${MODULES_LIST}; do\
		cd ${MODULES_DIR_PATH}$$m; make clean;\
		done

transfer:
	cd ${MODULES_DIR_PATH}${TRANSFER_MODULE}; make transfer;
	
symlink:
	for m in ${MODULES_LIST}; do\
		cd ${MODULES_DIR_PATH}$$m; make symlink;\
		done
