/*
 * hcsr04.h
 *
 *  Created on: Oct 6, 2015
 *      Author: done
 */

#include <stdio.h>
#include <unistd.h>
#include <linux/gpio.h>

#include "prussdrv.h"
#include "pruss_intc_mapping.h"

#include <time.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <errno.h>

//#include "iolib.h"

#ifndef HCSR04_H_
#define HCSR04_H_

#define DEBUG
#define IOLIB_DBG
#define CYCLES 500
#define DIST_PER_NSEC 58.440

// define pins and ports
#define PORT_P8 8
#define ECHO_PIN_FL 46
#define ECHO_PIN_FC 45
#define ECHO_PIN_FR 44
#define ECHO_PIN_RL 43
#define ECHO_PIN_RR 42
#define TRIGGER_PIN_FL 41
#define TRIGGER_PIN_FC 40
#define TRIGGER_PIN_FR 39
#define TRIGGER_PIN_RL 38
#define TRIGGER_PIN_RR 37

// define PRU shared mem offset
#define PRU_FL_OFFSET 0
#define PRU_FC_OFFSET 1
#define PRU_FR_OFFSET 2
#define PRU_RL_OFFSET 3
#define PRU_RR_OFFSET 4

/* function prototypes*/
int initPru1();
int initPins();
int deinitPru1();
int getDistancePRU1(char* pruBin, int index);
float getDistanceFL();
float getDistanceFC();
float getDistanceFR();
float getDistanceRL();
float getDistanceRR();


#endif /* HCSR04_H_ */
