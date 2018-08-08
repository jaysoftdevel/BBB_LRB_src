#CC = arm-linux-gnueabi-gcc
CFLAGS = -Wall -DTESTSTEPPER -DDEBUG

all: stepperL

libiofunc.a: ../gpio/iolib.c
	@echo ">> Creating gpio lib"
	make -C ../gpio/ clean
	make -C ../gpio/ lib

stepperL: stepperL.c
	@echo ">> Compiling stepperL binary"
	$(CC) $(CFLAGS) -c -o stepperL.o stepperL.c -I../gpio
	$(CC) -o stepperL stepperL.o -L../gpio -liofunc

clean:
	rm -r stepperL.o stepperL
