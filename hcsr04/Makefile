#CC = arm-linux-gnueabi-gcc
CFLAGS = -Wall -DTESTHCSR04
PRU_ASM=pasm
DTC = dtc
IPASM_PATH=$(PASM_PATH)# import from rootbot-hcsr04 recipe


all: hcsr04_FL.bin hcsr04_FC.bin hcsr04_FR.bin hcsr04_RL.bin hcsr04_RR.bin hcsr04

hcsr04-00A0.dtbo: hcsr04.dts
	@echo ">> Compiling Driver"
	$(DTC) -O dtb -o hcsr04-00A0.dtbo -b 0 -@ hcsr04.dts

hcsr04_FL.bin: hcsr04_FL.p
	@echo ">> Generating PRU binary FL"
	$(IPASM_PATH)$(PRU_ASM) -b hcsr04_FL.p

hcsr04_FC.bin: hcsr04_FC.p
	@echo ">> Generating PRU binary FC"
	$(IPASM_PATH)$(PRU_ASM) -b hcsr04_FC.p
	
hcsr04_FR.bin: hcsr04_FR.p
	@echo ">> Generating PRU binary FR"
	$(IPASM_PATH)$(PRU_ASM) -b hcsr04_FR.p
	
hcsr04_RL.bin: hcsr04_RL.p
	@echo ">> Generating PRU binary RL"
	$(IPASM_PATH)$(PRU_ASM) -b hcsr04_RL.p
	
hcsr04_RR.bin: hcsr04_RR.p
	@echo ">> Generating PRU binary RR"
	$(IPASM_PATH)$(PRU_ASM) -b hcsr04_RR.p

libiofunc.a: ../GPIO/iofunc/iolib.c
	@echo ">> Creating GPIO lib"
	make -C ../GPIO/iofunc/ clean
	make -C ../GPIO/iofunc/ lib
	
hcsr04: hcsr04.c
	@echo ">> Compiling HC-SR04 binary"
	$(CC) $(CFLAGS) hcsr04.c prussdrv.c -o hcsr04 -I../gpio -I. -L. -L../gpio -liofunc

clean:
	rm -rf hcsr04 hcsr04.o hcsr04_FL.bin hcsr04_FC.bin hcsr04_FR.bin hcsr04_RL.bin hcsr04_RR.bin

#install: hcsr04-00A0.dtbo hcsr04_FL.bin hcsr04_FC.bin hcsr04_FR.bin hcsr04_RL.bin hcsr04_RR.bin hcsr04 
#	cp hcsr04-00A0.dtbo /lib/firmware
	# echo hcsr04_FL > /sys/devices/bone_capemgr.9/slots
	# cat /sys/devices/bone_capemgr.9/slots
