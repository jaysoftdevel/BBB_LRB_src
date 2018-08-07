#!/bin/bash

echo "rebuiding"
make clean
make

echo "reinserting module"
rmmod /dev/st7565
insmod st7565.ko
