#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <setjmp.h>
#include <pthread.h>
#include <sys/time.h>
#include <errno.h>
#include <string.h>

#define DELAY 10000
#define SLIMIT 100000
#define DLIMIT 15000

static int nextStepR = 1;
static int nextStepL = 1;
static int nextDisp = 1;
static unsigned int delayStepR = 15000;
static unsigned int delayStepL = 15000;
static unsigned int delayDisp = 100000;
static unsigned int heartBeat = 500;
static unsigned int beatStepR = 0;
static unsigned int beatStepL = 0;
static unsigned int beatDisp = 0;
static unsigned int gStepR = 0;
static unsigned int gStepL = 0;
static unsigned int gDisp = 0;
static unsigned int rval = 0;
// threads
pthread_t tidStepR;
pthread_t tidStepL;
pthread_t tidDisp;

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
	while (1) {
		if (nextStepR) {
			//printf("step %i gStep %i\n", ++cnt,gStep);
			gStepR++;
			nextStepR = 0;
		}
		if (gStepR >= SLIMIT) {
			pthread_exit(&rval);
			printf("step exit\n");
		}
		// do stuff!!!!
		while (!nextStepR) {
			//printf("not\n");
			usleep(5);
		}
	}
}

void* threadStepL(void* arg) {
	static int cnt = 0;
	while (1) {
		if (nextStepL) {
			//printf("step %i gStep %i\n", ++cnt,gStep);
			gStepL++;
			nextStepL = 0;
		}
		if (gStepL >= SLIMIT) {
			pthread_exit(&rval);
			printf("step exit\n");
		}
		// do stuff!!!!
		while (!nextStepL) {
			//printf("not\n");
			usleep(5);
		}
	}
}

void* threadDisp(void* arg) {
	static int cnt = 0;
	while (1) {
		if (nextDisp) {
			//printf("disp %i gDisp %i\n", ++cnt,gDisp);
			gDisp++;
			nextDisp = 0;
		}
		if (gDisp >= DLIMIT) {
			pthread_exit(&rval);
			printf("disp exit\n");
		}
		// do stuff!!!!
		while (!nextDisp) {
			//printf("not\n");
			usleep(5);
		}
	}
}

int main(int argc, char ** argv) {
	struct timeval start, stop;
	int time = 0;
	// creating and running stepper thread
	if (pthread_create(&tidStepR, NULL, &threadStepR, NULL) != 0) {
		printf("error creating thread\n");
		return -1;
	}
	// creating and running stepper thread
	if (pthread_create(&tidStepL, NULL, &threadStepL, NULL) != 0) {
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
	printf("thread name: %s len: %x\n", str, len);
	// creating and running display thread
	if (pthread_create(&tidDisp, NULL, &threadDisp, NULL) != 0) {
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
	while (1) {
		// do stuff!!!!

		//while (1) {
		if (gDisp >= DLIMIT && gStepR >= SLIMIT) {
			gettimeofday(&stop, NULL);
			printf("done...: %.3lfms\n",
					((((double) stop.tv_sec) - ((double) start.tv_sec)) * 1000
							+ ((((double) stop.tv_usec)
									- ((double) start.tv_usec)) / 1000))
							/ 1000);
			break;
		}
		usleep(5);
		//}
		//break;
	}
	return 0;
}

