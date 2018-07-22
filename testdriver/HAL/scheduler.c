#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <setjmp.h>
#include <pthread.h>
#include <sched.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
// user includes
#include "lcd.h"
#include "hcsr04.h"
#include "stepperR.h"
#include "stepperL.h"

#define SLIMIT 10000
#define DLIMIT 150

FILE *fp;
long double a[4], b[4];
int loadavg;

static int nextStepR = 1;
static int nextStepL = 1;
static int nextDisp = 1;
static int killThreads = 0;
static int posLeft = 0;
static int posRight = 0;
static unsigned int delayStepR = 1500;
static unsigned int delayStepL = 1500;
static unsigned int delaySonar = 100000;
static unsigned int delayDisp = 300000;
static unsigned int heartBeat = 200;
static unsigned int beatStepR = 0;
static unsigned int beatStepL = 0;
static unsigned int beatDisp = 0;
static unsigned int gStepR = 0;
static unsigned int gStepL = 0;
static unsigned int gDisp = 0;
static unsigned int rval = 0;
static int lcd;
static DisplayData displayData;

// threads
pthread_t tidStepR;
pthread_t tidStepL;
pthread_t tidDisp;
pthread_t tidSonar;

void actHB_handler(int x) {
	beatStepR += heartBeat;
	beatStepL += heartBeat;
	beatDisp += heartBeat;
	// right stepper
	if (beatStepR >= delayStepR) {
		nextStepR = 1;
		beatStepR = 0;
	}
	// left stepper
	if (beatStepL >= delayStepL) {
		nextStepL = 1;
		beatStepL = 0;
	}
	// display
	if (beatDisp >= delayDisp) {
		nextDisp = 1;
		beatDisp = 0;
	}
	return;
}

void* threadStepR(void* arg) {
	static int cnt = 0;
	while (!killThreads) {
		if (nextStepR) {
			gStepR++;
			nextStepR = 0;
			stepFwdR();
		}
		// do stuff!!!!
		while (!nextStepR) {
			usleep(50);
		}
	}
}

void* threadStepL(void* arg) {
	static int cnt = 0;
	while (!killThreads) {
		if (nextStepL) {
			//printf("step %i gStep %i\n", ++cnt,gStep);
			gStepL++;
			nextStepL = 0;
			stepFwdL();
		}
		// do stuff!!!!
		while (!nextStepL) {
			usleep(50);
		}
	}
}

void* threadDisp(void* arg) {
	static int cnt = 0;
	while (!killThreads) {
		if (nextDisp) {
			gDisp++;
			nextDisp = 0;
			displayData.connectionStatus.ping = 0;
			displayData.connectionStatus.connectionStatus = 1;
			displayData.motorStatus.positionLeft = gStepL % 200;
			displayData.motorStatus.positionRight = gStepR % 200;

			displayData.currentLoad = 0;
			if (ioctl(lcd, IOCTL_LCD_WORKING_MODE, &displayData) == -1) {
				printf("could not write data: %s\n", strerror(errno));
			}
		}
		// do stuff!!!!
		while (!nextDisp) {
			//printf("not\n");
			usleep(100);
		}
	}
}

void* threadSonar(void* arg) {
	// complete parallel low prio running thread
	while (!killThreads) {
		displayData.distanceSensors.distFrontLeft = getDistanceFL();
		usleep(delaySonar);
		displayData.distanceSensors.distFrontCenter = getDistanceFC();
		usleep(delaySonar);
		displayData.distanceSensors.distFrontRight = getDistanceFR();
		usleep(delaySonar);
		displayData.distanceSensors.distRearLeft = getDistanceRL();
		usleep(delaySonar);
		displayData.distanceSensors.distRearRight = getDistanceRR();
	}
}

int main(int argc, char ** argv) {
	struct timeval start, stop;
	int time = 0;
	// custom thread attributes
	pthread_attr_t custom_sched_attr;
	int fifo_max_prio, fifo_min_prio, fifo_mid_prio;
	struct sched_param fifo_param;

	fp = fopen("/proc/stat", "r");

	// init stepper gpios
	setGPIOClock();
	if (iolib_init() != 0) {
		printf("error initializing gpio lib\n");
		return (-1);
	}
	initStepperRGpio();
	stepRNone();
	initStepperLGpio();
	stepLNone();

	// init display
	lcd = open("/dev/st7565", O_SYNC);
	if (lcd < 0) {
		printf("error opening lcd: %s\n", strerror(errno));
		return -1;
	}
	if (ioctl(lcd, IOCTL_LCD_CLEAR_ALL, NULL ) == -1) {
		printf("could not clear: %s\n", strerror(errno));
	}
	if (ioctl(lcd, IOCTL_LCD_SETUP_WORKING_MODE, NULL ) == -1) {
		printf("could not set to working mode: %s\n", strerror(errno));
	}

	// init sonar sensors
	initHcsr04Gpio();
	initPru1();

	// configure high prio thread attributes for stepper driver
	pthread_attr_init(&custom_sched_attr);
	pthread_attr_setinheritsched(&custom_sched_attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&custom_sched_attr, SCHED_FIFO);
	fifo_max_prio = sched_get_priority_max(SCHED_FIFO);
	fifo_min_prio = sched_get_priority_min(SCHED_FIFO);
	fifo_mid_prio = (fifo_min_prio + fifo_max_prio) / 2;
	fifo_param.sched_priority = fifo_mid_prio;
	pthread_attr_setschedparam(&custom_sched_attr, &fifo_param);

	// creating and running stepper thread
	if (pthread_create(&tidStepR, &custom_sched_attr, &threadStepR, NULL)
			!= 0) {
		printf("error creating thread\n");
		return -1;
	}
	// creating and running stepper thread
	if (pthread_create(&tidStepL, &custom_sched_attr, &threadStepL, NULL)
			!= 0) {
		printf("error creating thread\n");
		return -1;
	}
	char* str;
	int len;
	/*int err=pthread_setname_np(threadStep,"blubbbb");
	 if(err!=0){
	 printf("out: %s, %i\n",strerror(err),err);
	 return -1;
	 }*/

	// creating and running display thread
	if (pthread_create(&tidDisp, NULL, &threadDisp, NULL) != 0) {
		printf("error creating thread\n");
		return -1;
	}

	// creating and running sonar thread
	if (pthread_create(&tidSonar, NULL, &threadSonar, NULL) != 0) {
		printf("error creating thread\n");
		return -1;
	}

	struct sigaction actHB;
	// alarm handler
	actHB.sa_handler = actHB_handler;
	actHB.sa_flags = SA_ONSTACK;
	sigaction(SIGALRM, &actHB, NULL);

	/* Set an alarm to go off after 1,000 microseconds (one thousandth
	 of a second). */
	printf("go\n");
	gettimeofday(&start, NULL);
	ualarm(heartBeat, heartBeat);
	while (!killThreads) {
		// do stuff!!!!
		//printf("step %i disp %i\n",gStepR,gDisp);
		//while (1) {
		if (gDisp >= DLIMIT && gStepR >= SLIMIT) {
//			printf("get stop time\n");
			gettimeofday(&stop, NULL);
			printf("done...: %.3lfms\n",
					((((double) stop.tv_sec) - ((double) start.tv_sec)) * 1000
							+ ((((double) stop.tv_usec)
									- ((double) start.tv_usec)) / 1000))
							/ 1000);
			killThreads = 1;
		}
		usleep(5);
		//}
		//break;
	}
	printf("deiniting\n");
	close(lcd);
	stepRNone();
	stepLNone();
	fclose(fp);
	deinitStepperGpio();
	return 0;
}

