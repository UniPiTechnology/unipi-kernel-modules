* Kernel modules required for UniPi Neuron/Axon/Patron/Iris PLC

You need Linux kernel source to compile this modules.
Tested for kernel versions 4.11 .. 5.15.5
For bullseye USE_MFD

* Create package with

```
.dpkg-buildpackage --target-type=arm-linux-gnueabihf -t arm-linux-gnueabihf -aarmhf -uc -us -b
```

* To cross-compile modules

CROSS_COMPILE=aarch64-linux-gnu- ARCH=arm64 CCPREFIX=aarch64-linux-gnu- make LINUX_DIR_PATH=<path to linux src tree>
