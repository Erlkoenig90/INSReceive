This is a test project for the STM32F407VG micro controller for demonstrating some unusual DMA behaviour.

* For compilation, the [GNU ARM Embedded](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm) toolchain is used
* This project can be compiled to run in flash memory via
  ```make TGMEM=FLASH```
* To compile for running from RAM:
  ```make TGMEM=RAM```
* This project can also be imported as an eclipse project, if the [GNU ARM Eclipse](http://gnuarmeclipse.github.io/) plugin is installed.

The compiled binary is found in the build/ subdirectory.