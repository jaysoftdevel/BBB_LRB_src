#!/bin/bash

GPIO="/sys/class/gpio"

if [ $# -ne 1 ]
then
     echo usage: ./triggerpin.sh [pinnumber]
     exit
fi
echo $1

# export pin
echo "exporting pin "$1
echo $1 > $GPIO"/export"
sleep .02
# configure pin
echo "out"> $GPIO"/gpio"$1"/direction"
# write trigger sequece
echo "starting sequece on pin "$1
echo 0 > $GPIO"/gpio"$1"/value"
sleep .01
echo 1 > $GPIO"/gpio"$1"/value"
sleep .01
echo 0 > $GPIO"/gpio"$1"/value"
# unexport pin
echo "unexporting pin "$1
sleep .02
echo $1 > $GPIO"/unexport"
