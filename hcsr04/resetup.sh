#!/bin/bash

SLOTS="/sys/devices/bone_capemgr.9/slots"
OVERLAY="hcsr04_FL"
# remove old overlay
echo "-`cat $SLOTS|grep $OVERLAY|cut -d: -f1|cut -d " " -f2`" > $SLOTS
# remake and install
make clean
make
make install
./install-driver.sh