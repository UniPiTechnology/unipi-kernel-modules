* Kernel modules required for UniPi Neuron/Axon PLC

You need Linux kernel source to compile this modules.
Tested for kernel versions 4.11 .. 4.19

* Create package with

```
.dpkg-buildpackage --target-type=arm-linux-gnueabihf -t arm-linux-gnueabihf -aarmhf -uc -us -b
```
