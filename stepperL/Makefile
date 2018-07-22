CC = gcc
CFLAGS = -Wall -DTESTSTEPPER -DDEBUG

all: stepperL

libiofunc.a: ../GPIO/iofunc/iolib.c
	@echo ">> Creating GPIO lib"
	make -C ../GPIO/iofunc/ clean
	make -C ../GPIO/iofunc/ lib

stepperL: stepperL.c libiofunc.a
	@echo ">> Compiling StepperMotor_Control sample"
	$(CC) $(CFLAGS) -c -o stepperL.o stepperL.c -I../GPIO/iofunc
	$(CC) -o stepperL stepperL.o -L../GPIO/iofunc -liofunc

clean:
	rm -r stepperL.o stepperL