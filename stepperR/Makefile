CC = gcc
CFLAGS = -Wall -DTESTSTEPPER -DDEBUG

all: stepperR

libiofunc.a: ../GPIO/iofunc/iolib.c
	@echo ">> Creating GPIO lib"
	make -C ../GPIO/iofunc/ clean
	make -C ../GPIO/iofunc/ lib

stepperR: stepperR.c libiofunc.a
	@echo ">> Compiling StepperMotor_Control sample"
	$(CC) $(CFLAGS) -c -o stepperR.o stepperR.c -I../GPIO/iofunc
	$(CC) -o stepperR stepperR.o -L../GPIO/iofunc -liofunc

clean:
	rm -r stepperR.o stepperR