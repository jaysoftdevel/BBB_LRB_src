#!/bin/bash

# override user leds in kernel space
LEDS="/sys/class/leds"
NUM=50

for i in $(seq 0 3)
do
	echo timer > $LEDS/beaglebone:green:usr$i/trigger
	echo $NUM > $LEDS/beaglebone:green:usr$i/delay_on
	echo $NUM > $LEDS/beaglebone:green:usr$i/delay_off
	NUM=$((NUM+NUM))
done
echo should blink now...
sleep 10
echo reverting...
echo heartbeat > $LEDS/beaglebone:green:usr0/trigger
echo mmc0 > $LEDS/beaglebone:green:usr1/trigger
echo cpu0 > $LEDS/beaglebone:green:usr2/trigger
echo mmc1 > $LEDS/beaglebone:green:usr3/trigger
echo done...
