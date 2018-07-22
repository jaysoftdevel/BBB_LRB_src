obj-m += st7565.o

all: 
	#make -C /usr/src/linux-${shell uname -r}/ SUBDIRS=${shell pwd} modules
	make -C /usr/src/linux-headers-${shell uname -r}/ SUBDIRS=${shell pwd} modules
clean:
	#make -C /usr/src/linux-${shell uname -r}/ SUBDIRS=${shell pwd} clean
	make -C /usr/src/linux-headers-${shell uname -r}/ SUBDIRS=${shell pwd} clean
