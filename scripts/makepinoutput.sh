#!/bin/bash

GPIO="/sys/class/gpio"


if [ $# -ne 2 ]
then
     echo "usage: ./makepinout.sh [PINNUMBER] [IN|OUT]"
     exit
fi

echo $1 > $GPIO/export
echo $2 > $GPIO/gpio$1/direction
