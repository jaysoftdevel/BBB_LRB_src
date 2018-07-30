#CC = gcc
CFLAGS = -Wall -DTESTSTEPPER -DDEBUG

all: stepperR

libiofunc.a: ../gpio/iofunc/iolib.c
	@echo ">> Creating gpio lib"
	make -C ../gpio/iofunc/ clean
	make -C ../gpio/iofunc/ lib

stepperR: stepperR.c
	@echo ">> Compiling stepperR binary"
	$(CC) $(CFLAGS) -c -o stepperR.o stepperR.c -I../gpio/iofunc
	$(CC) -o stepperR stepperR.o -L../gpio/iofunc -liofunc

clean:
	rm -r stepperR.o stepperR
