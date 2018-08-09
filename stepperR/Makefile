#CC = gcc
CFLAGS = -Wall -DTESTSTEPPER -DDEBUG

all: stepperR

libiofunc.a: ../gpio/iolib.c
	@echo ">> Creating gpio lib"
	make -C ../gpio/ clean
	make -C ../gpio/ lib

stepperR: stepperR.c
	@echo ">> Compiling stepperR binary"
	$(CC) $(CFLAGS) -c -g -o stepperR.o stepperR.c -I../gpio
	$(CC) -g -o stepperR stepperR.o -L../gpio -liofunc

clean:
	rm -r stepperR.o stepperR
