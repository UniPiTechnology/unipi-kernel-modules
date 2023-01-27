* Kernel modules (version 2) required for UniPi Neuron/Axon/Patron/Iris PLC

You need Linux kernel source to compile this modules.
Tested for kernel versions 5.10 .. 5.15.5


* To cross-compile modules

CROSS_COMPILE=aarch64-linux-gnu- ARCH=arm64 CCPREFIX=aarch64-linux-gnu- make LINUX_DIR_PATH=<path to linux src tree>


* Device tree overlays for activating modules

Device tree sources for various platforms are available in repositories

   https://github.com/UniPiTechnology/os-configurator-data-patron
   https://github.com/UniPiTechnology/os-configurator-data-iris
   https://github.com/UniPiTechnology/os-configurator-data-neuron
