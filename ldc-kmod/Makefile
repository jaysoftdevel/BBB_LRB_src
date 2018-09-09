obj-m += st7565.o

SRC := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install
clean:
	make -C $(KERNEL_SRC) M=$(SRC) clean

# use this for local build on BBB using debian
#all: 
#	make -C /usr/src/linux-headers-${shell uname -r}/ SUBDIRS=${shell pwd} modules

#clean:
#	make -C /usr/src/linux-headers-${shell uname -r}/ SUBDIRS=${shell pwd} clean
