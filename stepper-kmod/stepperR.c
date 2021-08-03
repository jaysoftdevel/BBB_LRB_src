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
#ifndef STEPPERMOTOR_CONTROL_R_C
#define STEPPERMOTOR_CONTROL_R_C

//#define DEBUG

/* Kernel Programming */
//#define MODULE
#define LINUX
//#define __KERNEL__

//#if defined(CONFIG_MODVERSIONS) && ! defined(MODVERSIONS)
//   #include <linux/modversions.h>
//   #define MODVERSIONS
//#endif
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
                   
#include "stepperR.h"

long stepperR_control(struct file *f, unsigned int control, unsigned long value);
static ssize_t stepperR_write(struct file *f, const char __user *buf, size_t len, loff_t *off);

static dev_t second; // Global variable for the second device number 
static struct cdev c_dev; // Global variable for the character device structure
static struct class *cl; // Global variable for the device class

static struct file_operations pugs_fops = { .owner = THIS_MODULE,
		.unlocked_ioctl = stepperR_control, .write = stepperR_write };
		
// Struct for each GPIO pin
struct gpio_pin {
	const char * name;
	unsigned gpio;
};

// Struct to point to all GPIO pins
struct gpio_platform_data {
	struct gpio_pin * pins;
	int num_pins;
};

// Struct for interface definition
// !! take care of pin order here!!!: N-S-E-W !!
static struct gpio_pin stepperR_gpio_pins[] = { { .name = "stepperR::north", .gpio =
COIL_PIN_NORTH, }, { .name = "stepperR::south", .gpio = COIL_PIN_SOUTH, }, { .name =
	"stepperR::east", .gpio = COIL_PIN_EAST, }, { .name = "stepperR::west", .gpio = 
	COIL_PIN_WEST, }, };

static struct gpio_platform_data stepperR_gpio_pin_info = { .pins =
		stepperR_gpio_pins, .num_pins = ARRAY_SIZE(stepperR_gpio_pins), };


// used for buffer
char * rx_buffer;
int BUFFER_SIZE = 8;

static struct tPos coilPos;

int stepFwdR(void) {
	int i;
	for(i=0;i<NUM_OF_COILS;i++){
		if(i!=coilPos.pos){
#ifdef DEBUG
			printk("set pin %i low\n",stepperR_gpio_pins[i].gpio);
#endif
			gpio_set_value(stepperR_gpio_pins[i].gpio, 0);
		}
		else{
#ifdef DEBUG
			printk("set pin %i high\n",stepperR_gpio_pins[i].gpio);
#endif
			gpio_set_value(stepperR_gpio_pins[i].gpio, 1);
		}
	}
	coilPos.pos++;
	return (0);
}

int stepRevR(void) {
	int i;
	for(i=0;i<NUM_OF_COILS;i++){
		if(i!=coilPos.pos){
#ifdef DEBUG
			printk("set pin %i low\n",stepperR_gpio_pins[i].gpio);
#endif
			gpio_set_value(stepperR_gpio_pins[i].gpio, 0);
		}
		else{
#ifdef DEBUG
			printk("set pin %i high\n",stepperR_gpio_pins[i].gpio);
#endif
			gpio_set_value(stepperR_gpio_pins[i].gpio, 1);
		}
	}
	coilPos.pos--;
	return (0);
}

int stepRNone(void){
	printk("set all pint to low\n");
	gpio_set_value(COIL_PIN_NORTH, 0);
	gpio_set_value(COIL_PIN_EAST, 0);
	gpio_set_value(COIL_PIN_SOUTH, 0);
	gpio_set_value(COIL_PIN_WEST, 0);
	return (0);
}

static int __init stepperR_init(void)
{
	printk("[%s] initializiing stepperR\n",__FUNCTION__);
	// allocate a buffer and zero it out
	rx_buffer = kmalloc(BUFFER_SIZE,  GFP_KERNEL);
	memset(rx_buffer, 0, BUFFER_SIZE);
#ifdef DEBUG
	printk("[%s] registering stepperR chr dev\n",__FUNCTION__);
#endif //DEBUG
	// register a character device
	if (alloc_chrdev_region(&second, 0, 1, "stepperR") < 0)
			{
				return -1;
			}
			if ((cl = class_create(THIS_MODULE, "chardrv3")
	) == NULL)
	{
		unregister_chrdev_region(second, 1);
		return -1;
	}
#ifdef DEBUG
	printk("[%s] creating stepperR device\n",__FUNCTION__);
#endif //DEBUG
	if (device_create(cl, NULL, second, NULL, "stepperR") == NULL)
	{
		class_destroy(cl);
		unregister_chrdev_region(second, 1);
		return -1;
	}
#ifdef DEBUG
	printk("[%s] stepperR cdev init\n",__FUNCTION__);
#endif //DEBUG
	cdev_init(&c_dev, &pugs_fops);
	if (cdev_add(&c_dev, second, 1) == -1)
	{
		device_destroy(cl, second);
		class_destroy(cl);
		unregister_chrdev_region(second, 1);
		return -1;
	}

	// request access to GPIO, set them all as outputs (initially low)
	printk("[%s] registering stepperR gpio pins\n",__FUNCTION__);
	int err, i;
	i = 0;
	for(i = 0; i < stepperR_gpio_pin_info.num_pins; i++) {
		printk("[%s] register pin %d with gpio %d with name %s\n",__FUNCTION__,i,stepperR_gpio_pins[i].gpio, stepperR_gpio_pins[i].name);
		err = gpio_request(stepperR_gpio_pins[i].gpio, stepperR_gpio_pins[i].name);
		if(err) {
			printk("[%s] Could not get access to GPIO %i, error code: %i\n",__FUNCTION__, stepperR_gpio_pins[i].gpio, err);
		}
		err = gpio_direction_output(stepperR_gpio_pins[i].gpio, 0);
		if(err) {
			printk("[%s] Could not set value of GPIO %i, error code: %i\n",__FUNCTION__, stepperR_gpio_pins[i].gpio, err);
		}
	}
	// ready to go!
	printk("[%s] stepperR registered!\n",__FUNCTION__);

	return 0;
}

long stepperR_control(struct file *f, unsigned int control, unsigned long value) {
#ifdef DEBUG
	printk("[%s] controlling\n",__FUNCTION__);
#endif
	return 0;
}

static ssize_t stepperR_write(struct file *f, const char __user *buf, size_t len, loff_t *off) {
#ifdef DEBUG
	printk("[%s] start printing\n",__FUNCTION__);
#endif

return len;
}

static void __exit stepperR_exit(void)
{
	printk("[%s] shutting down...",__FUNCTION__);
	// release buffer
	if (rx_buffer) {
		kfree(rx_buffer);
	}

	// release GPIO
	int i = 0;
	for(i = 0; i < stepperR_gpio_pin_info.num_pins; i++) {
		gpio_free(stepperR_gpio_pins[i].gpio);
	}

	// unregister character device
	cdev_del(&c_dev);
	device_destroy(cl, second);
	class_destroy(cl);
	unregister_chrdev_region(second, 1);
	printk("[%s] stepperR unregistered\n",__FUNCTION__);
}

/********************/

int init_module(void)
{
	int i;
	printk("Hello World stepperR!\n");
	stepperR_init();
	msleep(1000);
	stepFwdR();
	msleep(1000);
	stepFwdR();
	msleep(2000);
	
	for(i=0;i<50;i++){
		printk("5 wait for %i\n",i);
		udelay(500);
		stepFwdR();
	}
	msleep(2000);
	for(i=0;i<100;i++){
		printk("2 wait for %i\n",i);
		udelay(200);
		stepFwdR();
	}
	msleep(2000);
	for(i=0;i<200;i++){
		printk("1 wait for %i\n",i);
		udelay(100);
		stepFwdR();
	}
	msleep(2000);
	for(i=0;i<200;i++){
		printk("1 wait for %i\n",i);
		udelay(50);
		stepFwdR();
	}
	msleep(2000);
	for(i=0;i<400;i++){
		printk("1 wait for %i\n",i);
		ndelay(20);
		stepFwdR();
	}
	return 0;
}

void cleanup_module(void)
{
	stepperR_exit();
	printk("Goodbye Cruel World stepperR!\n");
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

MODULE_LICENSE("GPL");

#endif /* STEPPERMOTOR_CONTROL_R_H */
