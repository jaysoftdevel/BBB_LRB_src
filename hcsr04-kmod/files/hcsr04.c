
#ifndef HCSRO4_KERNEL_MODULE_C
#define HCSRO4_KERNEL_MODULE_C

/* Kernel Programming */
#define MODULE
#define LINUX
#define __KERNEL__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <asm/uaccess.h>  /* for put_user */
#include <asm/errno.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <asm/delay.h>

#include <linux/mm.h> 

#include "hcsr04.h"

#include "prussdrv.h"
#include "pruss_intc_mapping.h"

//#include <time.h>
//#include <fcntl.h>
//#include <sys/mman.h>

//#include "iolib.h"

static void *pruDataMem;
static volatile unsigned int *pruData;

#define TESTHCSR04

#ifdef TESTHCSR04
int main(void) {
//	initPru1();
#ifdef DEBUG
	printk("PRU MAIN: running binary\n");
#endif
	/* run pru asm */
	// Print the distance received from the sonar
	// At 20 degrees in dry air the speed of sound is 342.2 cm/sec
	// so it takes 29.12 us to make 1 cm, i.e. 58.44 us for a roundtrip of 1 cm
	int dist[5]={0};
	long int total=0;
	int i = CYCLES;


	/* start clock stuff*/
	//struct timespec start, stop;
	//gettimeofday(&start,0);
	//printk("clk: %li\n", start.tv_nsec);
	/* stop clock stuff*/

	printk("initializing pins...\n");
	initHcsr04Gpio();
	initPru1();
	for (i; i > 0; i--) {
//		gettimeofday(&start,0);
		dist[0]=getDistancePRU1("hcsr04_FL.bin",PRU_FL_OFFSET);
		dist[1]=getDistancePRU1("hcsr04_FC.bin",PRU_FC_OFFSET);
		dist[2]=getDistancePRU1("hcsr04_FR.bin",PRU_FR_OFFSET);
		dist[3]=getDistancePRU1("hcsr04_RL.bin",PRU_RL_OFFSET);
		dist[4]=getDistancePRU1("hcsr04_RR.bin",PRU_RR_OFFSET);
//		printk("disabled measurement as no sensors are around...\n");
//		usleep(100000);
//		gettimeofday(&stop,0);
		printk("%i:\tFL %.2f\tFC %.2f\tFR %.2f\tRL %.2f\tRR %.2f\t\n",CYCLES+1-i,dist[0] / DIST_PER_NSEC,dist[1] / DIST_PER_NSEC,dist[2] / DIST_PER_NSEC,dist[3] / DIST_PER_NSEC,dist[4] / DIST_PER_NSEC);
		// check for overrun
//		if(stop.tv_nsec-start.tv_nsec<0){
//			printk("> %li %lu\n",stop.tv_nsec-start.tv_nsec,1000000+stop.tv_nsec-start.tv_nsec);
//			total+=1000000+stop.tv_nsec-start.tv_nsec;
//		}
//		else{
//			total+=(long)stop.tv_nsec-start.tv_nsec;
//		}
		udelay(10000);
	}
//	printk("total %f\n",(float)(total/1000));
//	printk("average roundtripping: %f\n",(float)(total/(CYCLES*5))/1000);
#ifdef DEBUG
	printk("PRU MAIN: exit...\n");
#endif
	// deinit
	deinitPru1();
#ifdef DEBUG
	printk("free GPIOs\n");
#endif
	//iolib_free();
	return (0);

}
#endif

/* Global variables of the driver */ 
#define CM_PER_BASE 0x44e00000 
#define CM_WKUP_BASE 0x44e00400 
#define GPIO0_CLK_CONFIG_OFFSET 0x8 

//void setGPIOClock(){
//   int fd = open("/dev/mem", O_RDWR | O_SYNC);
//   if (fd < 0)
//   {
//      printk("Could not open GPIO memory fd\n");
//      return;
//   }
//
//   unsigned int *clock_gpio2;
//  clock_gpio2 = (unsigned int *) mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x44E00000);
//  // 0xb0 is CM_PER_GPIO2_CLKCTRL as given in the TRM, use 0xb4 for GPIO_3 (see the TRM)
//  int offsetInMemory = 0xb0;
//  // get the value, we divide by 4 because it is a byte offset
//  int memValue = clock_gpio2[(offsetInMemory/4)];
//  // print it – it will probably be 0x030000 if the clock has never been enabled
//  printk("Value = %04x\n", memValue);
//  // now set it, this enables the memory
//  clock_gpio2[(offsetInMemory/4)] = 0x02;
//  close(fd);
//}

//int initHcsr04Gpio(){
	//printk("initilizing gpio\n");
	//printk("enable clocks\n");
	
	////setGPIOClock();

////	if(iolib_init()!=0){
////		printk("error initializing gpio lib\n");
////		return (-1);
////	}
	//printk("setting up pins\n");
	//// FL
	////if(iolib_setdir(PORT_P8, ECHO_PIN_FL, DIR_IN)){
	//if(iolib_setdir(PORT_P8,ECHO_PIN_FL, DIR_IN)){
		//printk("could not set pin %i on port %i with direction %i\n",ECHO_PIN_FL,PORT_P8,DIR_IN);
		//return (-2);
	//}
	//printk("FL_E ");
	//if(iolib_setdir(PORT_P8, TRIGGER_PIN_FL, DIR_OUT)){
		//printk("could not set pin %i on port %i with direction %i\n",TRIGGER_PIN_FL,PORT_P8,DIR_OUT);
		//return (-2);
	//}
	//printk("FL_T ");
	//// FC
	//if(iolib_setdir(PORT_P8, ECHO_PIN_FC, DIR_IN)){
		//printk("could not set pin %i on port %i with direction %i\n",ECHO_PIN_FC,PORT_P8,DIR_IN);
		//return (-2);
	//}
	//printk("FC_E ");
	//if(iolib_setdir(PORT_P8, TRIGGER_PIN_FC, DIR_OUT)){
		//printk("could not set pin %i on port %i with direction %i\n",TRIGGER_PIN_FC,PORT_P8,DIR_OUT);
		//return (-2);
	//}
	//printk("FC_T ");
	//// FR
	//if(iolib_setdir(PORT_P8, ECHO_PIN_FR, DIR_IN)){
		//printk("could not set pin %i on port %i with direction %i\n",ECHO_PIN_FR,PORT_P8,DIR_IN);
		//return (-2);
	//}
	//printk("FR_E ");
	//if(iolib_setdir(PORT_P8, TRIGGER_PIN_FR, DIR_OUT)){
		//printk("could not set pin %i on port %i with direction %i\n",TRIGGER_PIN_FR,PORT_P8,DIR_OUT);
		//return (-2);
	//}
	//printk("FR_T ");
	//// RL
	//if(iolib_setdir(PORT_P8, ECHO_PIN_FR, DIR_IN)){
		//printk("could not set pin %i on port %i with direction %i\n",ECHO_PIN_FR,PORT_P8,DIR_IN);
		//return (-2);
	//}
	//printk("RL_E ");
	//if(iolib_setdir(PORT_P8, TRIGGER_PIN_FR, DIR_OUT)){
		//printk("could not set pin %i on port %i with direction %i\n",TRIGGER_PIN_FR,PORT_P8,DIR_OUT);
		//return (-2);
	//}
	//printk("RL_T ");
	//// RR
	//if(iolib_setdir(PORT_P8, ECHO_PIN_RR, DIR_IN)){
		//printk("could not set pin %i on port %i with direction %i\n",ECHO_PIN_RR,PORT_P8,DIR_IN);
		//return (-2);
	//}
	//printk("RR_E ");
	//if(iolib_setdir(PORT_P8, TRIGGER_PIN_RR, DIR_OUT)){
		//printk("could not set pin %i on port %i with direction %i\n",TRIGGER_PIN_RR,PORT_P8,DIR_OUT);
		//return (-2);
	//}
	//printk("RR_T ");
	//printk("done setting up GPIO\n");
	//return (0);
//}

int initPru1() {
#ifdef DEBUG
	printk("PRU INIT: enter init routine\n");
#endif
	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
	prussdrv_init();
#ifdef DEBUG
	printk("PRU INIT: open driver\n");
#endif
	/* Open PRU Interrupt */
	if (prussdrv_open(PRU_EVTOUT_1)) {
		// Handle failure
		printk(">> PRU open failed:\n");
		return 1;
	}
#ifdef DEBUG
	printk("PRU INIT: init interrupt\n");
#endif
	/* Get the interrupt initialized */
	prussdrv_pruintc_init(&pruss_intc_initdata);
#ifdef DEBUG
	printk("PRU INIT: map data memory\n");
#endif
	/* Get pointers to PRU local memory */
	prussdrv_map_prumem(PRUSS0_PRU1_DATARAM, &pruDataMem);
	pruData = (unsigned int *) pruDataMem;
#ifdef DEBUG
	printk("PRU INIT: leaving init routine\n");
#endif
	return (0);
}

int deinitPru1() {
	/* Disable PRU and close memory mapping*/
#ifdef DEBUG
	printk("PRU DEINIT: disable pru\n");
#endif
	prussdrv_pru_disable(1);
	prussdrv_exit();
	//iolib_free();
	return (0);
}

float getDistanceFL(){
	return  (float)(getDistancePRU1("hcsr04_FL.bin",PRU_FL_OFFSET)/ DIST_PER_NSEC);
}

float getDistanceFC(){
	return (getDistancePRU1("hcsr04_FC.bin",PRU_FC_OFFSET)/ DIST_PER_NSEC);
}

float getDistanceFR(){
	return getDistancePRU1("hcsr04_FR.bin",PRU_FR_OFFSET)/ DIST_PER_NSEC;
}

float getDistanceRL(){
	return getDistancePRU1("hcsr04_RL.bin",PRU_RL_OFFSET)/ DIST_PER_NSEC;
}

float getDistanceRR(){
	return getDistancePRU1("hcsr04_RR.bin",PRU_RR_OFFSET)/ DIST_PER_NSEC;
}

int getDistancePRU1(char* pruBin, int index) {
	/* binary on pru 1 */
#ifdef DEBUG
	printk("PRU ASM: reinit pru\n");
#endif
	//initPru1();
#ifdef DEBUG
	printk("PRU ASM: in asm execution\n");
#endif
	prussdrv_exec_program(1, pruBin);
	/* Get measurements */
#ifdef DEBUG
	printk("PRU ASM: waiting for event\n");
#endif
	// Wait for the PRU interrupt
	prussdrv_pru_wait_event(PRU_EVTOUT_1);
#ifdef DEBUG
	printk("PRU ASM: clearing event\n");
#endif
	prussdrv_pru_clear_event(PRU_EVTOUT_1, PRU1_ARM_INTERRUPT);
#ifdef DEBUG
	printk("PRU ASM: leaving asm\n");
#endif
	//retData=pruData[index];
	//prussdrv_pru_reset(1);
	//deinitPru1();
	return pruData[index];
}

int init_module(void)
{
   printk("initiaziling HCSR04... place code!!!\n");
	
   return 0;
}


void cleanup_module(void)
{
  printk("Deinitializing HCSR04... place code!!!\n");
}    

MODULE_AUTHOR("jaysoftdevel");
MODULE_LICENSE("GPL");
MODULE_VERSION("2:1.0") ;
MODULE_DESCRIPTION("sonar kernel module hcsr04");


#endif HCSRO4_KERNEL_MODULE_C
