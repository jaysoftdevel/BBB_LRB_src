/*
 *
 * StepperMotor_Control.c
 *
 * Possible items:
 * H-Bridge: sn754410
 * step motor: TMCM-MOTOR
 *
 * according to guide:
 * http://www.instructables.com/id/Drive-a-Stepper-Motor-with-an-AVR-Microprocessor/?ALLSTEPS
 *
 */

#include "stepperL.h"

static struct tPos coilPos;

#ifdef TESTSTEPPER
int main(void) {
	int delay=DELAY;
	// init gpio
#ifdef DEBUG
	printf("setup gpio\n");
#endif
	initStepperLGpio();
	int i = 0;
#ifdef DEBUG
	printf("wait to get started\n");
#endif
	usleep(2000000);
#ifdef DEBUG
	printf("start...\n");
#endif
	for (i; i < CYCLES; i++) {
		//printf("rtp %i delay: %i\n",i,delay);
		// roundtippin
		pin_high(PORT_P8, COIL_PIN_NORTH);
		pin_low(PORT_P8, COIL_PIN_EAST);
		pin_low(PORT_P8, COIL_PIN_SOUTH);
		pin_low(PORT_P8, COIL_PIN_WEST);
		usleep(delay);
		pin_low(PORT_P8, COIL_PIN_NORTH);
		pin_high(PORT_P8, COIL_PIN_EAST);
		pin_low(PORT_P8, COIL_PIN_SOUTH);
		pin_low(PORT_P8, COIL_PIN_WEST);
		usleep(delay);
		pin_low(PORT_P8, COIL_PIN_NORTH);
		pin_low(PORT_P8, COIL_PIN_EAST);
		pin_high(PORT_P8, COIL_PIN_SOUTH);
		pin_low(PORT_P8, COIL_PIN_WEST);
		usleep(delay);
		pin_low(PORT_P8, COIL_PIN_NORTH);
		pin_low(PORT_P8, COIL_PIN_EAST);
		pin_low(PORT_P8, COIL_PIN_SOUTH);
		pin_high(PORT_P8, COIL_PIN_WEST);
		usleep(delay);
	}
	pin_low(PORT_P8, COIL_PIN_NORTH);
	pin_low(PORT_P8, COIL_PIN_EAST);
	pin_low(PORT_P8, COIL_PIN_SOUTH);
	pin_low(PORT_P8, COIL_PIN_WEST);
	// deinit gpio
#ifdef DEBUG
	printf("done roundtripping, deiniting...\n");
#endif
	deinitStepperLGpio();
	return (0);
}
#endif /*TESTSTEPPER*/

int stepFwdL() {
	int i;
	for(i=0;i<NUM_OF_COILS;i++){
#ifdef DEBUG
		printf("i: %i coil: %i\n",i,coilPos.pos);
#endif
		if(i!=coilPos.pos){
#ifdef DEBUG
			printf("set pin %i low\n",COIL_PIN_OFFSET+i);
#endif
			pin_low(PORT_P8, COIL_PIN_OFFSET+i);
		}
		else{
#ifdef DEBUG
			printf("set pin %i high\n",COIL_PIN_OFFSET+coilPos.pos);
#endif
			pin_high(PORT_P8, COIL_PIN_OFFSET+coilPos.pos);
		}
	}
	coilPos.pos++;
	return (0);
}

int stepRevL() {
	int i;
	for(i=0;i<NUM_OF_COILS;i++){
#ifdef DEBUG
		printf("i: %i coil: %i\n",i,coilPos.pos);
#endif
		if(i!=coilPos.pos){
#ifdef DEBUG
			printf("set pin %i low\n",COIL_PIN_OFFSET+i);
#endif
			pin_low(PORT_P8, COIL_PIN_OFFSET+i);
		}
		else{
#ifdef DEBUG
			printf("set pin %i high\n",COIL_PIN_OFFSET+coilPos.pos);
#endif
			pin_high(PORT_P8, COIL_PIN_OFFSET+coilPos.pos);
		}
	}
	coilPos.pos--;
	return (0);
}

int stepLNone(){
	pin_low(PORT_P8, COIL_PIN_NORTH);
	pin_low(PORT_P8, COIL_PIN_EAST);
	pin_low(PORT_P8, COIL_PIN_SOUTH);
	pin_low(PORT_P8, COIL_PIN_WEST);
}
/**
 *
 * move this function to a global util library for this project
 *
 **/
//void setGPIOClock() {
//	int fd = open("/dev/mem", O_RDWR | O_SYNC);
//	if (fd < 0) {
//		printf("Could not open GPIO memory fd\n");
//		return;
//	}
//
//	unsigned int *clock_gpio2;
//	clock_gpio2 = (unsigned int *) mmap(NULL, 0x1000, PROT_READ | PROT_WRITE,
//			MAP_SHARED, fd, 0x44E00000);
//	// 0xb0 is CM_PER_GPIO2_CLKCTRL as given in the TRM, use 0xb4 for GPIO_3 (see the TRM)
//	int offsetInMemory = 0xb0;
//	// get the value, we divide by 4 because it is a byte offset
//	int memValue = clock_gpio2[(offsetInMemory / 4)];
//#ifdef DEBUG
//	// print it – it will probably be 0x030000 if the clock has never been enabled
//	printf("Value = %04x\n", memValue);
//#endif
//	// now set it, this enables the memory
//	clock_gpio2[(offsetInMemory / 4)] = 0x02;
//	close(fd);
//
//}

int initStepperLGpio() {

#ifdef DEBUG
	printf("setting up pins\n");
#endif
	if (iolib_setdir(PORT_P8, COIL_PIN_NORTH, DIR_OUT)) {
		printf("could not set pin %i on port %i with direction %i\n",
				COIL_PIN_NORTH, PORT_P8, DIR_OUT);
		return (-2);
	}
	if (iolib_setdir(PORT_P8, COIL_PIN_EAST, DIR_OUT)) {
		printf("could not set pin %i on port %i with direction %i\n",
				COIL_PIN_EAST, PORT_P8, DIR_OUT);
		return (-2);
	}
	if (iolib_setdir(PORT_P8, COIL_PIN_SOUTH, DIR_OUT)) {
		printf("could not set pin %i on port %i with direction %i\n",
				COIL_PIN_SOUTH, PORT_P8, DIR_OUT);
		return (-2);
	}
	if (iolib_setdir(PORT_P8, COIL_PIN_WEST, DIR_OUT)) {
		printf("could not set pin %i on port %i with direction %i\n",
				COIL_PIN_WEST, PORT_P8, DIR_OUT);
		return (-2);
	}
	pin_low(PORT_P8, COIL_PIN_NORTH);
	pin_low(PORT_P8, COIL_PIN_EAST);
	pin_low(PORT_P8, COIL_PIN_SOUTH);
	pin_low(PORT_P8, COIL_PIN_WEST);
#ifdef DEBUG
	printf("pins ok\n");
#endif

	return (0);
}

//int deinitStepperLGpio() {
//	iolib_free();
//	return (0);
//}

