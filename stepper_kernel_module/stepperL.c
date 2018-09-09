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
#ifndef STEPPERMOTOR_CONTROL_L_C
#define STEPPERMOTOR_CONTROL_L_C

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
                   
#include "stepperL.h"

long stepperL_control(struct file *f, unsigned int control, unsigned long value);
static ssize_t stepperL_write(struct file *f, const char __user *buf, size_t len, loff_t *off);

static dev_t second; // Global variable for the second device number 
static struct cdev c_dev; // Global variable for the character device structure
static struct class *cl; // Global variable for the device class

static struct file_operations pugs_fops = { .owner = THIS_MODULE,
		.unlocked_ioctl = stepperL_control, .write = stepperL_write };
		
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
static struct gpio_pin stepperL_gpio_pins[] = { { .name = "stepperL::north", .gpio =
COIL_PIN_NORTH, }, { .name = "stepperL::south", .gpio = COIL_PIN_SOUTH, }, { .name =
"stepperL::east", .gpio = COIL_PIN_EAST, }, { .name = "stepperL::west", .gpio = COIL_PIN_WEST, }, };

static struct gpio_platform_data stepperL_gpio_pin_info = { .pins =
		stepperL_gpio_pins, .num_pins = ARRAY_SIZE(stepperL_gpio_pins), };

// used for buffer
char * rx_buffer;
int BUFFER_SIZE = 8;

static struct tPos coilPos;

int stepFwdL(void) {
	int i;
	for(i=0;i<NUM_OF_COILS;i++){
		if(i!=coilPos.pos){
#ifdef DEBUG
			printk("set pin %i low\n",stepperL_gpio_pins[i].gpio);
#endif
			gpio_set_value(stepperL_gpio_pins[i].gpio, 0);
		}
		else{
#ifdef DEBUG
			printk("set pin %i high\n",stepperL_gpio_pins[i].gpio);
#endif
			gpio_set_value(stepperL_gpio_pins[i].gpio, 1);
		}
	}
	coilPos.pos++;
	return (0);
}                                                                                       //

int stepRevL(void) {
	int i;
	for(i=0;i<NUM_OF_COILS;i++){
#ifdef DEBUG
		printk("i: %i coil: %i\n",stepperL_gpio_pins[i].gpio);
#endif
		if(i!=coilPos.pos){
#ifdef DEBUG
			printk("set pin %i low\n",stepperL_gpio_pins[i].gpio);
#endif
			gpio_set_value(stepperL_gpio_pins[i].gpio, 0);
		}
		else{
#ifdef DEBUG
			printk("set pin %i high\n",stepperL_gpio_pins[i].gpio);
#endif
			gpio_set_value(stepperL_gpio_pins[i].gpio, 1);
		}
	}
	coilPos.pos--;
	return (0);
}

int stepLNone(void){
	printk("set all pint to low\n");
	gpio_set_value(COIL_PIN_NORTH, 0);
	gpio_set_value(COIL_PIN_EAST, 0);
	gpio_set_value(COIL_PIN_SOUTH, 0);
	gpio_set_value(COIL_PIN_WEST, 0);
}

static int __init stepperL_init(void)
{
	printk("[%s] initializiing stepperL\n",__FUNCTION__);
	// allocate a buffer and zero it out
	rx_buffer = kmalloc(BUFFER_SIZE,  GFP_KERNEL);
	memset(rx_buffer, 0, BUFFER_SIZE);
#ifdef DEBUG
	printk("[%s] registering stepperL chr dev\n",__FUNCTION__);
#endif //DEBUG
	// register a character device
	if (alloc_chrdev_region(&second, 0, 1, "stepperL") < 0)
			{
				return -1;
			}
			if ((cl = class_create(THIS_MODULE, "chardrv2")
	) == NULL)
	{
		unregister_chrdev_region(second, 1);
		return -1;
	}
#ifdef DEBUG
	printk("[%s] creating stepperL device\n",__FUNCTION__);
#endif //DEBUG
	if (device_create(cl, NULL, second, NULL, "stepperL") == NULL)
	{
		class_destroy(cl);
		unregister_chrdev_region(second, 1);
		return -1;
	}
#ifdef DEBUG
	printk("[%s] stepperL cdev init\n",__FUNCTION__);
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
	printk("[%s] registering stepperL gpio pins\n",__FUNCTION__);
	int err, i;
	i = 0;
	for(i = 0; i < stepperL_gpio_pin_info.num_pins; i++) {
		printk("[%s] register pin %d with gpio %d with name %s\n",__FUNCTION__,i,stepperL_gpio_pins[i].gpio, stepperL_gpio_pins[i].name);
		err = gpio_request(stepperL_gpio_pins[i].gpio, stepperL_gpio_pins[i].name);
		if(err) {
			printk("[%s] Could not get access to GPIO %i, error code: %i\n",__FUNCTION__, stepperL_gpio_pins[i].gpio, err);
		}
		err = gpio_direction_output(stepperL_gpio_pins[i].gpio, 0);
		if(err) {
			printk("[%s] Could not set value of GPIO %i, error code: %i\n",__FUNCTION__, stepperL_gpio_pins[i].gpio, err);
		}
	}
	// ready to go!
	printk("[%s] stepperL registered!\n",__FUNCTION__);

	return 0;
}

long stepperL_control(struct file *f, unsigned int control, unsigned long value) {
#ifdef DEBUG
	printk("[%s] controlling\n",__FUNCTION__);
#endif
	return 0;
}

static ssize_t stepperL_write(struct file *f, const char __user *buf, size_t len, loff_t *off) {
#ifdef DEBUG
	printk("[%s] start printing\n",__FUNCTION__);
#endif

return len;
}

static void __exit stepperL_exit(void)
{
	printk("[%s] shutting down...",__FUNCTION__);
	// release buffer
	if (rx_buffer) {
		kfree(rx_buffer);
	}

	// release GPIO
	int i = 0;
	for(i = 0; i < stepperL_gpio_pin_info.num_pins; i++) {
		gpio_free(stepperL_gpio_pins[i].gpio);
	}

	// unregister character device
	cdev_del(&c_dev);
	device_destroy(cl, second);
	class_destroy(cl);
	unregister_chrdev_region(second, 1);
	printk("[%s] stepperL unregistered\n",__FUNCTION__);
}

/********************/

int init_module(void)
{
	int i;
	printk("Hello World stepperL!\n");
	stepperL_init();
	msleep(1000);
	stepFwdL();
	msleep(1000);
	stepFwdL();
	msleep(2000);
	
	for(i=0;i<50;i++){
		printk("5 wait for %i\n",i);
		udelay(500);
		stepFwdL();
	}
	msleep(2000);
	for(i=0;i<100;i++){
		printk("2 wait for %i\n",i);
		udelay(200);
		stepFwdL();
	}
	msleep(2000);
	for(i=0;i<200;i++){
		printk("1 wait for %i\n",i);
		udelay(100);
		stepFwdL();
	}
	msleep(2000);
	for(i=0;i<200;i++){
		printk("1 wait for %i\n",i);
		udelay(50);
		stepFwdL();
	}
	msleep(2000);
	for(i=0;i<400;i++){
		printk("1 wait for %i\n",i);
		ndelay(20);
		stepFwdL();
	}
	return 0;
}

void cleanup_module(void)
{
	stepperL_exit();
	printk("Goodbye Cruel World stepperL!\n");
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

MODULE_LICENSE("GPL");


#endif /* STEPPERMOTOR_CONTROL_L_H */
