#!/bin/bash

USER=debian
#TARGET=192.168.178.37
TARGET=192.168.7.2
TARGETPATH=/home/debian/Projects

EXCLUDES=" sync.sh"


for i in $EXCLUDES
do
		EXCLSTR=$EXCLSTR" --exclude=\"$i\""
done
rsync -av . --exclude=".*" --exclude=".*/" $EXCLSTR $USER@$TARGET:$TARGETPATH

