/*
 Testdriver for:
 - lcd panel
 - step motors
 - sonars

 package
 */
#include "lcd.h"
#include "hcsr04.h"
#include "stepperR.h"
#include <sys/ioctl.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define TESTCYCLES 20000

static DisplayData displayData;
struct timespec start, stop;

int main(void){
	testDisplay();
	//testHcsr04();
	//testStepper();
	return (0);
}

int testStepper(){
	int i;
	initStepperRGpio();
	printf("starting stepper\n");
	for(i=0;i<TESTCYCLES;i++){
		usleep(1400);
//		gettimeofday(&start,0);
		stepFwdR();
//		gettimeofday(&stop,0);
//		printf("duration: %fus\n",((stop.tv_nsec-start.tv_nsec)));
	}
	sleep(2);
	stepLNone();
	sleep(2);
	for(i=0;i<TESTCYCLES;i++){
		usleep(1400);
//		gettimeofday(&start,0);
		stepRevR();
//		gettimeofday(&stop,0);
//		printf("duration: %fus\n",((stop.tv_nsec-start.tv_nsec)));
	}
	stepLNone();
	deinitStepperGpio();
	return(0);
}

int testHcsr04(void){
	float distance=0,i;
	printf("init pins\n");
	initHcsr04Gpio();
	printf("init pru\n");
	initPru1();
	printf("go into cycle\n");
	for(i=0;i<TESTCYCLES;i++){
		gettimeofday(&start,0);
		printf("get distance\n");
		distance=getDistanceFL();
		printf("done\n");
		gettimeofday(&stop,0);
		printf("distance FL: %.2f duration: %.2fus\n",distance,(float)((stop.tv_nsec-start.tv_nsec)/1000));
		gettimeofday(&start,0);
		distance=getDistanceFC();
		gettimeofday(&stop,0);
		printf("distance FC: %.2f duration: %.2fus\n",distance,(float)((stop.tv_nsec-start.tv_nsec)/1000));
		gettimeofday(&start,0);
		distance=getDistanceFR();
		gettimeofday(&stop,0);
		printf("distance FR: %.2f duration: %.2fus\n",distance,(float)((stop.tv_nsec-start.tv_nsec)/1000));
		gettimeofday(&start,0);
		distance=getDistanceRL();
		gettimeofday(&stop,0);
		printf("distance RL: %.2f duration: %.2fus\n",distance,(float)((stop.tv_nsec-start.tv_nsec)/1000));
		gettimeofday(&start,0);
		distance=getDistanceRR();
		gettimeofday(&stop,0);
		printf("distance RR: %.2f duration: %.2fus\n",distance,(float)((stop.tv_nsec-start.tv_nsec)/1000));
	}
	deinitPru1();
	return 0;
}

int testDisplay(void) {
	printf("starting test driver\n");
	printf("open lcd\n");
	int lcd = open("/dev/st7565", O_SYNC);
	printf("file: %x\n", lcd);
	if (lcd < 0) {
		printf("error opening lcd: %s\n", strerror(errno));
		return -1;
	}
	printf("Tester: clear lcd\n");
	if (ioctl(lcd, IOCTL_LCD_CLEAR_ALL, NULL ) == -1) {
		printf("could not clear: %s\n", strerror(errno));
	}
	usleep(100000);
	printf("Tester: do intro\n");
	if (ioctl(lcd, IOCTL_LCD_INTRO, NULL ) == -1) {
		printf("could not clear: %s\n", strerror(errno));
	}
	usleep(100000);
	if (ioctl(lcd, IOCTL_LCD_CLEAR_ALL, NULL ) == -1) {
		printf("could not clear: %s\n", strerror(errno));
	}
	usleep(100000);
	printf("Tester: test driver ending...\n");
	if (ioctl(lcd, IOCTL_LCD_TEXT_MODE, "test string...") == -1) {
		printf("could not clear: %s\n", strerror(errno));
	}
	usleep(100000);
	if (ioctl(lcd, IOCTL_LCD_CLEAR_ALL, NULL ) == -1) {
		printf("could not clear: %s\n", strerror(errno));
	}
	printf("Tester: do intro\n");
	if (ioctl(lcd, IOCTL_LCD_SETUP_WORKING_MODE, NULL ) == -1) {
		printf("could not clear: %s\n", strerror(errno));
	}
	usleep(100000);
	unsigned char a = 0, b = 0, c = 100, d = 0, i;
	double avgs[3]={0};
	for (i = 0; i < TESTCYCLES; i++) {
		displayData.distanceSensors.distFrontLeft = a++;
		displayData.distanceSensors.distFrontCenter = b--;
		displayData.distanceSensors.distFrontRight = i;
		displayData.distanceSensors.distRearLeft = b - a++;
		displayData.distanceSensors.distRearRight = (a++) - b--;
		displayData.connectionStatus.ping = c--;
		displayData.connectionStatus.connectionStatus =	true;
		displayData.motorStatus.positionLeft = d;
		displayData.motorStatus.positionRight =	d++;

		getloadavg(avgs,1);
		displayData.currentLoad=avgs[0];
		printf("1: %.2f\t2: %.2f\t3: %.2lf\n",avgs[0],avgs[1],avgs[2]);

		if (ioctl(lcd, IOCTL_LCD_WORKING_MODE, &displayData) == -1) {
			printf("could not write data: %s\n", strerror(errno));
		}
		printf("cycle: %i\n",i);
		usleep(100000);
	}

	close(lcd);
	return 0;
}
