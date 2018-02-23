obj-m += neuronspi.o

all: 
	make ARCH=arm CROSS_COMPILE=${CCPREFIX} -C /root/bokula/linux M=$(PWD) modules

clean: 
	make -C /root/bokula/linux M=$(PWD) clean

transfer: clean all
	scp neuronspi.ko tomunipi:/root/

