#CC = arm-linux-gnueabi-gcc
CFLAGS = -Wall -DTESTSTEPPER -DDEBUG

all: stepperL

#libiofunc.a: ../gpio/iofunc/iolib.c
#	@echo ">> Creating gpio lib"
#	make -C ../gpio/iofunc/ clean
#	make -C ../gpio/iofunc/ lib

stepperL: stepperL.c
	@echo ">> Compiling stepperL binary"
	$(CC) $(CFLAGS) -c -o stepperL.o stepperL.c -I../gpio/iofunc
	$(CC) -o stepperL stepperL.o -L../gpio/iofunc -liofunc

clean:
	rm -r stepperL.o stepperL
