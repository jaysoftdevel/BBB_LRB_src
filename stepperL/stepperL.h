// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef Ultrtasonic_Distance_Simple_H_
#define Ultrtasonic_Distance_Simple_H_

#include <stdio.h>
#include <unistd.h>

#include <fcntl.h>
#include <sys/mman.h>

#include "iolib.h"

#define CYCLES 100000

// default DIOs
#define PORT_P8 8
// GPIO2_8, GPIO2_9, GPIO2_16, GPIO2_17
// arranged controvers to stepper R
#define COIL_PIN_NORTH 36	// BLUE
#define COIL_PIN_EAST 35	// RED
#define COIL_PIN_SOUTH 34	// GREEN
#define COIL_PIN_WEST 33	// BLACK
#define COIL_PIN_OFFSET COIL_PIN_WEST	// offset for base pin in iolib
#define NUM_OF_COILS 4	// for the 4 coil stepper motor
#define DELAY  1500 /* microseconds between steps */

/* Global variables of the driver */
#define CM_PER_BASE 0x44e00000
#define CM_WKUP_BASE 0x44e00400
#define GPIO0_CLK_CONFIG_OFFSET 0x8


//add your function definitions for the project Ultrtasonic_Distance_Simple here
typedef unsigned char uint8;
typedef struct tPos{
	uint8 pos :2;
}tPos;

int initStepperLGpio();
int deinitStepperLGpio();
int stepLNone();
int stepFwdL();

//Do not add code below this line
#endif /* STEPPERMOTOR_CONTROL.H */
