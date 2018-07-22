#!/bin/bash

make clean
make
cp st7565.ko /lib/modules/$(uname -r)/kernel/drivers/st7565
depmod
rmmod st7565.ko
insmod st7565.ko