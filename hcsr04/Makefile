CC = gcc
CFLAGS = -Wall -DTESTHCSR04
PRU_ASM = pasm
DTC = dtc

all: hcsr04-00A0.dtbo hcsr04_FL.bin hcsr04_FC.bin hcsr04_FR.bin hcsr04_RL.bin hcsr04_RR.bin hcsr04

hcsr04-00A0.dtbo: hcsr04.dts
	@echo ">> Compiling Driver"
	$(DTC) -O dtb -o hcsr04-00A0.dtbo -b 0 -@ hcsr04.dts

hcsr04_FL.bin: hcsr04_FL.p
	@echo ">> Generating PRU binary FL"
	$(PRU_ASM) -b hcsr04_FL.p

hcsr04_FC.bin: hcsr04_FC.p
	@echo ">> Generating PRU binary FC"
	$(PRU_ASM) -b hcsr04_FC.p
	
hcsr04_FR.bin: hcsr04_FR.p
	@echo ">> Generating PRU binary FR"
	$(PRU_ASM) -b hcsr04_FR.p
	
hcsr04_RL.bin: hcsr04_RL.p
	@echo ">> Generating PRU binary RL"
	$(PRU_ASM) -b hcsr04_RL.p
	
hcsr04_RR.bin: hcsr04_RR.p
	@echo ">> Generating PRU binary RR"
	$(PRU_ASM) -b hcsr04_RR.p

libiofunc.a: ../GPIO/iofunc/iolib.c
	@echo ">> Creating GPIO lib"
	make -C ../GPIO/iofunc/ clean
	make -C ../GPIO/iofunc/ lib
	
hcsr04: hcsr04.c libiofunc.a
	@echo ">> Compiling HC-SR04_FL example"
	$(CC) $(CFLAGS) -c -o hcsr04.o hcsr04.c -I../GPIO/iofunc
	$(CC) -lpthread -lprussdrv -o hcsr04 hcsr04.o -L../GPIO/iofunc -liofunc

clean:
	rm -rf hcsr04 hcsr04.o hcsr04_FL.bin hcsr04_FC.bin hcsr04_FR.bin hcsr04_RL.bin hcsr04_RR.bin

install: hcsr04-00A0.dtbo hcsr04_FL.bin hcsr04_FC.bin hcsr04_FR.bin hcsr04_RL.bin hcsr04_RR.bin hcsr04 
	cp hcsr04-00A0.dtbo /lib/firmware
	# echo hcsr04_FL > /sys/devices/bone_capemgr.9/slots
	# cat /sys/devices/bone_capemgr.9/slots