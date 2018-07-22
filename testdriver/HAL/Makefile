CC=gcc
STEPPER_R_PATH=../../stepperR
STEPPER_L_PATH=../../stepperL
HCSR_PATH=../../hcsr04
GPIO_PATH=../../GPIO/iofunc
LCD_PATH=../../lcd_kernel_module
INC=-I$(LCD_PATH) -I$(HCSR_PATH) -I$(GPIO_PATH) -I$(STEPPER_R_PATH) -I$(STEPPER_L_PATH)
LIBS=-L../../GPIO/iofunc -liofunc -lpthread -lprussdrv
CFLAGS=-Wall #-DDEBUG -D__DEBUG
SRC=$(shell find ../.. -type f -iname '*.c')
OBJ=hcsr04.o stepperR.o stepperL.o
PRU_ASM = pasm
KBUILD_NOPEDANTIC=1

libiofunc.a: $(GPIO_PATH)/iolib.c
	@echo ">> Creating GPIO lib"
	make -C $(GPIO_PATH) clean
	make -C $(GPIO_PATH) lib
	
hcsr04_FL.bin: $(HCSR_PATH)/hcsr04_FL.p
	@echo ">> Generating PRU binary FL"
	$(PRU_ASM) -b $(HCSR_PATH)/hcsr04_FL.p

hcsr04_FC.bin: $(HCSR_PATH)/hcsr04_FC.p
	@echo ">> Generating PRU binary FC"
	$(PRU_ASM) -b $(HCSR_PATH)/hcsr04_FC.p
	
hcsr04_FR.bin: $(HCSR_PATH)/hcsr04_FR.p
	@echo ">> Generating PRU binary FR"
	$(PRU_ASM) -b $(HCSR_PATH)/hcsr04_FR.p
	
hcsr04_RL.bin: $(HCSR_PATH)/hcsr04_RL.p
	@echo ">> Generating PRU binary RL"
	$(PRU_ASM) -b $(HCSR_PATH)/hcsr04_RL.p
	
hcsr04_RR.bin: $(HCSR_PATH)/hcsr04_RR.p
	@echo ">> Generating PRU binary RR"
	$(PRU_ASM) -b $(HCSR_PATH)/hcsr04_RR.p
	
hcsr04.o: $(HCSR_PATH)/hcsr04.c
	@echo ">> Creating hcsro4.o"
	$(CC) $(CFLAGS) $< -c $(INC)
	
stepperR.o: $(STEPPER_R_PATH)/stepperR.c
	@echo ">> Creating stepperR.o"
	$(CC) $(CFLAGS) $< -c $(INC)
	
stepperL.o: $(STEPPER_L_PATH)/stepperL.c
	@echo ">> Creating stepperL.o"
	$(CC) $(CFLAGS) $< -c $(INC)
	
	
test: libiofunc.a hcsr04.o hcsr04_FL.bin hcsr04_FC.bin hcsr04_FR.bin hcsr04_RL.bin hcsr04_RR.bin stepperR.o stepperL.o
	@echo ">> Creating test application"
	$(CC) -o test test.c $(INC) $(CFLAGS) $(OBJ) $(LIBS)

scheduler: libiofunc.a hcsr04.o hcsr04_FL.bin hcsr04_FC.bin hcsr04_FR.bin hcsr04_RL.bin hcsr04_RR.bin stepperR.o stepperL.o 
	@echo ">> Creating schedule application"
	$(CC) -o $@ scheduler.c $(INC) $(CFLAGS) $(OBJ) $(LIBS)
	
