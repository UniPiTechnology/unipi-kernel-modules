
CC = dtc -O dtb -@

.PHONY = clean
.PHONY += all

%.dtbo: %.dts
	$(CC) -o $@ $<

install: clean neuron-spi-new.dtbo
	$(shell mount -o remount,rw /boot) \
	$(shell cp neuron-spi-new.dtbo /boot/overlays)

clean:
	$(shell rm -r -f neuron-spi-new.dtbo)

