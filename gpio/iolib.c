// iolib.c
// Simple I/O library
// v1 October 2013 - shabaz

#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include "iolib.h"

#define DEBUG 1

const unsigned int ioregion_base[] = { GPIO0, GPIO1, GPIO2, GPIO3 };

const char p8_bank[] = { -1, -1, 1, 1, 1, 1, 2, 2, 2, 2, 1, 1, 0, 0, 1, 1, 0, 2,
		0, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 0, 0, 0, 2, 0, 2, 2, 2, 2, 2, 2, 2,
		2, 2, 2, 2 };

const unsigned int p8_bitmask[] = { 0, 0, 1 << 6, 1 << 7, 1 << 2, 1 << 3, 1
		<< 2, 1 << 3, 1 << 5, 1 << 4, 1 << 13, 1 << 12, 1 << 23, 1 << 26, 1
		<< 15, 1 << 14, 1 << 27, 1 << 1, 1 << 22, 1 << 31, 1 << 30, 1 << 5, 1
		<< 4, 1 << 1, 1 << 0, 1 << 29, 1 << 22, 1 << 24, 1 << 23, 1 << 25, 1
		<< 10, 1 << 11, 1 << 9, 1 << 17, 1 << 8, 1 << 16, 1 << 14, 1 << 15, 1
		<< 12, 0, 1 << 10, 1 << 11, 1 << 8, 1 << 9, 1 << 6, 1 << 7 };

const char p9_bank[] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 1, 0, 1, 1,
		1, 0, 0, 0, 0, 0, 0, 1, -1, 3, -1, 3, 3, 3, -1, 3, -1, -1, -1, -1, -1,
		-1, -1, -1, -1, 0, 0, -1, -1, -1, -1 };

const unsigned int p9_bitmask[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 << 30, 1
		<< 28, 1 << 31, 1 << 18, 1 << 16, 1 << 19, 1 << 5, 1 << 4, 1 << 13, 1
		<< 12, 1 << 3, 1 << 2, 1 << 17, 0, 1 << 21, 0, 1 << 19, 1 << 17, 1
		<< 15, 0, 1 << 14, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 << 20, 1 << 7, 0, 0, 0,
		0 };

int memh;
static int ctrlh = 0;
volatile unsigned int *gpio_addr[4] = { NULL, NULL, NULL, NULL };
volatile unsigned int *ctrl_addr = NULL;
char* bank[2];
unsigned int* port_bitmask[2];

int iolib_init(void) {
	int i;

	if (memh) {
#ifdef DEBUG
			printf("iolib_init: memory already mapped?\n");
#endif
		return (-1);
	}

	bank[0] = (char*) p8_bank;
	bank[1] = (char*) p9_bank;
	port_bitmask[0] = (unsigned int*) p8_bitmask;
	port_bitmask[1] = (unsigned int*) p9_bitmask;

	memh = open("/dev/mem", O_RDWR);
#ifdef DEBUG
	printf("status of memh open is %x\n", memh);
#endif
	for (i = 0; i < 4; i++) {
		gpio_addr[i] = mmap(0, GPIOX_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, memh, ioregion_base[i]);
		if (gpio_addr[i] == MAP_FAILED) {
#ifdef DEBUG
			printf("iolib_init: gpio mmap failure!\n");
#endif
			return (-1);
		}
#ifdef DEBUG
		else{
			printf("gpio[%i] = %x\n",i,(unsigned int)gpio_addr[i]);
		}
#endif
	}

#ifdef DEBUG
	printf("Enable all GPIO clocks\n");
#endif
	unsigned int *clock_gpio;
	clock_gpio = (unsigned int *) mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, memh, 0x44E00000);
	// 0xb0 is CM_PER_GPIO2_CLKCTRL as given in the TRM, use 0xb4 for GPIO_3 (see the TRM)
	int offsetInMemory_GPIO1 = 0xac;
	int offsetInMemory_GPIO2 = 0xb0;
	int offsetInMemory_GPIO3 = 0xb4;
	// get the value, we divide by 4 because it is a byte offset
	int memValueGPIO1 = clock_gpio[(offsetInMemory_GPIO1/4)];
	int memValueGPIO2 = clock_gpio[(offsetInMemory_GPIO2/4)];
	int memValueGPIO3 = clock_gpio[(offsetInMemory_GPIO3/4)];
#ifdef DEBUG
	// print it – it will probably be 0x030000 if the clock has never been enabled
	printf("GPIO1 clock value = %04x\n", memValueGPIO1);
	printf("GPIO2 clock value = %04x\n", memValueGPIO2);
	printf("GPIO3 clock value = %04x\n", memValueGPIO3);
#endif
	// now set it, this enables the memory
	printf("Setting GPIO1 clock\n");
	clock_gpio[(offsetInMemory_GPIO1/4)] = 0x02;
	printf("Setting GPIO2 clock\n");
	clock_gpio[(offsetInMemory_GPIO2/4)] = 0x02;
	printf("Setting GPIO3 clock\n");
	clock_gpio[(offsetInMemory_GPIO3/4)] = 0x02;
	printf("setting clocks done");
	if (PINMUX_EN) {
		ctrl_addr = mmap(0, CONTROL_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, ctrlh, CONTROL_MODULE);
		if (ctrl_addr == MAP_FAILED) {
#ifdef DEBUG
				printf("iolib_init: control module mmap failure!\n");
#endif
			return (-1);
		}
#ifdef DEBUG
		else{
			printf("ctrl_addr = %x\n",ctrl_addr);
		}
#endif
	}
	return (0);
}

int iolib_free(void) {
	if (memh != 0) {
		close(memh);
	}
	if (ctrlh != 0) {
		close(ctrlh);
	}
	return (0);
}

int iolib_setdir(char port, char pin, char dir) {
	//return 0;
	int i;
	int param_error = 0;
	volatile unsigned int* reg;

	printf("### iolib_setdir called with port: %x, pin: %x, dir: %x\n",port, pin, dir);
	// sanity checks
	if (memh == 0){
		param_error = 1;
	}
	if ((port<8) || (port>>9)){
		param_error = 2;
	}
	if ((pin < 1) || (pin > 46)){
		param_error = 3;
	}
	if (bank[port][pin] < 0){
		param_error = 4;
	}
	if (param_error != 0) {
#ifdef DEBUG
			printf("iolib_setdir: parameter error! Num: %x\n", param_error);
#endif
		return (-1);
	}

	// set the bit in the OE register in the appropriate region
#ifdef DEBUG
		for (i = 0; i < 4; i++) {
			printf("mmap region %d address is 0x%08x\n", i, gpio_addr[i]);
		}
		printf("iolib_setdir: bank is %d\n", bank[port - 8][pin - 1]);
#endif
	reg = (void*) gpio_addr[bank[port - 8][pin - 1]] + GPIO_OE;
#ifdef DEBUG
		printf("address: %x\n", reg);
#endif
	if (dir == DIR_OUT) {
		*reg &= ~(port_bitmask[port - 8][pin - 1]);
	} else if (dir == DIR_IN) {
		*reg |= port_bitmask[port - 8][pin - 1];
	}
#ifdef DEBUG
		printf("done setting pin\n");
#endif
	return (0);
}

inline void pin_high(char port, char pin) {
	*((unsigned int *) ((void *) gpio_addr[bank[port - 8][pin - 1]]
			+ GPIO_SETDATAOUT)) = port_bitmask[port - 8][pin - 1];
}

inline void pin_low(char port, char pin) {
	*((unsigned int *) ((void *) gpio_addr[bank[port - 8][pin - 1]]
			+ GPIO_CLEARDATAOUT)) = port_bitmask[port - 8][pin - 1];
}

inline char is_high(char port, char pin) {
	return ((*((unsigned int *) ((void *) gpio_addr[bank[port - 8][pin - 1]]
			+ GPIO_DATAIN)) & port_bitmask[port - 8][pin - 1]) != 0);
}

inline char is_low(char port, char pin) {
	return ((*((unsigned int *) ((void *) gpio_addr[bank[port - 8][pin - 1]]
			+ GPIO_DATAIN)) & port_bitmask[port - 8][pin - 1]) == 0);
}

int iolib_delay_ms(unsigned int msec) {
	int ret;
	struct timespec a;
	if (msec > 999) {
		fprintf(stderr,
				"delay_ms error: delay value needs to be less than 999\n");
		msec = 999;
	}
	a.tv_nsec = ((long) (msec)) * 1E6d;
	a.tv_sec = 0;
	if ((ret = nanosleep(&a, NULL)) != 0) {
		fprintf(stderr, "delay_ms error: %s\n", strerror(errno));
	}
	return (0);
}

void setGPIOClock() {
	int fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd < 0) {
		printf("Could not open GPIO memory fd\n");
		return;
	}

	unsigned int *clock_gpio2;
	printf("iolib: setting clock\n");
	clock_gpio2 = (unsigned int *) mmap(NULL, 0x1000, PROT_READ | PROT_WRITE,
			MAP_SHARED, fd, 0x44E00000);
	// 0xb0 is CM_PER_GPIO2_CLKCTRL as given in the TRM, use 0xb4 for GPIO_3 (see the TRM)
	int offsetInMemory = 0xb0;
	// get the value, we divide by 4 because it is a byte offset
	int memValue = clock_gpio2[(offsetInMemory / 4)];
	// print it – it will probably be 0x030000 if the clock has never been enabled
	printf("Value = %04x\n", memValue);
	// now set it, this enables the memory
	clock_gpio2[(offsetInMemory / 4)] = 0x02;
	close(fd);
}

int deinitStepperGpio() {
	iolib_free();
	return (0);
}
