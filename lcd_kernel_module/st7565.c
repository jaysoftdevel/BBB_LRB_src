#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/ioctl.h>
#include <asm/uaccess.h>
#include <linux/spi/spidev.h>
#include <linux/delay.h>
#include "font.h"
#include "lcd.h"

// prototypes
void st7565_init_lcd(void);
void lcd_ascii5x7_string(unsigned int xPos, unsigned int yPos,
		unsigned char * str);
void lcd_ascii5x7(unsigned int xPos, unsigned int yPos, unsigned char c);
void lcd_clear(void);
void lcd_set_page(unsigned char page, unsigned int column);
void lcd_transfer_data(unsigned char value, unsigned int A0);
void lcd_byte(unsigned char bits);
long lcd_control(struct file *f, unsigned int control, unsigned long value);
void lcd_intro(void);
void lcd_write_banner(void);
void lcd_setup_working_mode_frame(void);
void st7565_deinit(void);

// macro to convert bank and gpio into pin number
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

// Data and control lines
#define ST7565_CS	GPIO_TO_PIN(1, 13)	// P8_11
#define ST7565_RST	GPIO_TO_PIN(2, 1)	// P8_18
#define ST7565_AP	GPIO_TO_PIN(0, 23)	// P8_13
#define ST7565_A0	GPIO_TO_PIN(0, 26)	// P8_14
#define ST7565_CLK	GPIO_TO_PIN(1, 15)	// P8_15
#define ST7565_SI	GPIO_TO_PIN(1, 14)	// P8_16
#define IOCTL_TEST _IOW(0, 6, unsigned long)

// used for buffer
char * rx_buffer;
int BUFFER_SIZE = 64;

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
static struct gpio_pin st7565_gpio_pins[] = { { .name = "st7565::cs", .gpio =
ST7565_CS, }, { .name = "st7565::rst", .gpio = ST7565_RST, }, { .name =
		"st7565::a0", .gpio = ST7565_A0, }, { .name = "st7565::clk", .gpio =
ST7565_CLK, }, { .name = "st7565::si", .gpio = ST7565_SI, }, { .name =
		"st7565::ap", .gpio = ST7565_AP, }, };

static struct gpio_platform_data st7565_gpio_pin_info = { .pins =
		st7565_gpio_pins, .num_pins = ARRAY_SIZE(st7565_gpio_pins), };

static dev_t first; // Global variable for the first device number 
static struct cdev c_dev; // Global variable for the character device structure
static struct class *cl; // Global variable for the device class
//static unsigned char lcd_x, lcd_y;

static int st7565_open(struct inode *i, struct file *f) {
#ifdef DEBUG
	printk("opening st7565\n");
#endif
	st7565_init_lcd();
	return 0;
}

static int st7565_close(struct inode *i, struct file *f) {
#ifdef DEBUG
	printk("closing st7565\n");
#endif
	st7565_deinit();
	return 0;
}

static ssize_t st7565_read(struct file *f, char __user *buf, size_t len,
		loff_t *off) {
	// nothing to do here
	return 0;
}

static ssize_t st7565_write(struct file *f, const char __user *buf, size_t len,
		loff_t *off) {
#ifdef DEBUG
	printk("start printing\n");
#endif
	//zero the input buffer
	memset(rx_buffer, 0, BUFFER_SIZE);

	// copy the incoming data from userspace to a buffer in kernel space
	int retval = copy_from_user(rx_buffer, buf, len);

	// display the data on the LCD
	lcd_ascii5x7_string(0, 1, rx_buffer);
#ifdef DEBUG
	printk("printed %i chars\n", len);
#endif
	// return the number of characters written
	return len;
}

void lcd_intro(void/*int x, int y, int v*/) {
	unsigned char i = 1, j = 0, val = 0x55;
	for (i = 0; i < LCD_COLS; i++) {
		for (j = 0; j < LCD_PAGES; j++) {
			lcd_set_page(j, i);
			udelay(300);
			lcd_transfer_data(val, 1);
		}
		// invert
		val = ~val;
	}
	msleep(100);
	for (i = LCD_COLS; i > 0; i--) {
		for (j = 0; j < LCD_PAGES; j++) {
			lcd_set_page(j, i);
			udelay(300);
			lcd_transfer_data(0x0, 1);
		}
	}
	msleep(300);
	//write banner
	lcd_write_banner();

	// to be moved into first write access by application
#ifdef DEBUG
	printk("clearing...\n");
#endif
	msleep(1000);
	for (i = LCD_COLS; i > 0; i--) {
		for (j = 0; j < LCD_PAGES; j++) {
			lcd_set_page(j, i);
			udelay(300);
			lcd_transfer_data(0x0, 1);
		}
	}
}

void lcd_setup_working_mode_frame(void) {
#ifdef DEBUG
	printk("setup working mode\n");
#endif
	// setting up working frame, not yet values
	lcd_ascii5x7_string(4, 1 * TOKENSIZE, "FL");
	lcd_ascii5x7_string(4, 7 * TOKENSIZE, "FC");
	lcd_ascii5x7_string(4, 14 * TOKENSIZE, "FR");
	lcd_ascii5x7_string(5, 1 * TOKENSIZE, "RL");
	lcd_ascii5x7_string(5, 7 * TOKENSIZE, "RR");
	lcd_ascii5x7_string(6, 1 * TOKENSIZE, "RotL");
	lcd_ascii5x7_string(6, 10 * TOKENSIZE, "RotR");
	lcd_ascii5x7_string(7, 1 * TOKENSIZE, "Ping");
	lcd_ascii5x7_string(7, 10 * TOKENSIZE, "CS");
	lcd_ascii5x7_string(0, 1 * TOKENSIZE, "load");
}

void lcd_update_display_data(DisplayData dd) {
	char* buffer = kmalloc(4, GFP_KERNEL);
#ifdef DEBUG
	printk("pinting 1\n");
#endif
	// getting distance sensor values
	snprintf(buffer, 4, "%3d", dd.distanceSensors.distFrontLeft);
	lcd_ascii5x7_string(4, 1 + 3 * TOKENSIZE, buffer);
#ifdef DEBUG
	printk("pinting 2\n");
#endif
	snprintf(buffer, 4, "%3d", dd.distanceSensors.distFrontCenter);
	lcd_ascii5x7_string(4, 1 + 10 * TOKENSIZE, buffer);
#ifdef DEBUG
	printk("pinting 3\n");
#endif
	snprintf(buffer, 4, "%3d", dd.distanceSensors.distFrontRight);
	lcd_ascii5x7_string(4, 1 + 17 * TOKENSIZE, buffer);
#ifdef DEBUG
	printk("pinting 4\n");
#endif
	snprintf(buffer, 4, "%3d", dd.distanceSensors.distRearLeft);
	lcd_ascii5x7_string(5, 1 + 3 * TOKENSIZE, buffer);
#ifdef DEBUG
	printk("pinting 5\n");
#endif
	snprintf(buffer, 4, "%3d", dd.distanceSensors.distRearRight);
	lcd_ascii5x7_string(5, 1 + 10 * TOKENSIZE, buffer);
#ifdef DEBUG
	printk("pinting 6\n");
#endif
	// getting motor positions
	snprintf(buffer, 4, "%3d", dd.motorStatus.positionLeft);
	lcd_ascii5x7_string(6, 1 + 5 * TOKENSIZE, buffer);
#ifdef DEBUG
	printk("pinting 7\n");
#endif
	snprintf(buffer, 4, "%3d", dd.motorStatus.positionRight);
	lcd_ascii5x7_string(6, 1 + 14 * TOKENSIZE, buffer);
#ifdef DEBUG
	printk("pinting 8\n");
#endif
	// getting connection status
	snprintf(buffer, 4, "%4d", dd.connectionStatus.ping);
	lcd_ascii5x7_string(7, 1 + 5 * TOKENSIZE, buffer);
#ifdef DEBUG
	printk("pinting 9\n");
#endif
	if (dd.connectionStatus.connectionStatus == true) {
		lcd_ascii5x7_string(7, 1 + 14 * TOKENSIZE, "up");
	} else {
		lcd_ascii5x7_string(7, 1 + 14 * TOKENSIZE, "down");
	}
#ifdef DEBUG
	printk("pinting 10 %i\n", dd.currentLoad);
#endif
	// getting current load, no conversion necessary
	snprintf(buffer, 4, "%3d", dd.currentLoad);
	lcd_ascii5x7_string(0, 1 + 5 * TOKENSIZE, buffer);
#ifdef DEBUG
	printk("pinting 11\n");
#endif
	kfree(buffer);
#ifdef DEBUG
	printk("pinting 12\n");
#endif
}

long lcd_control(struct file *f, unsigned int control, unsigned long value) {
	if (control == IOCTL_LCD_TEXT_MODE) {
#ifdef DEBUG
		printk("received ioctl: control %x value %s\n", control, (char*) value);
#endif
		memset(rx_buffer, 0, strlen((char* )value));
		copy_from_user(rx_buffer, (char*) value, strlen((char*) value));
		lcd_ascii5x7_string(0, 10, rx_buffer);
	} else if (control == IOCTL_LCD_CLEAR_ALL) {
#ifdef DEBUG
		printk("clearing display\n");
#endif
		lcd_clear();
	}
	if (control == IOCTL_LCD_INTRO) {
#ifdef DEBUG
		printk("doing intro\n");
#endif
		lcd_intro();
	} else if (control == IOCTL_LCD_SETUP_WORKING_MODE) {
#ifdef DEBUG
		printk("setting up working mode frame\n");
#endif
		lcd_setup_working_mode_frame();
	} else if (control == IOCTL_LCD_WORKING_MODE) {
		static DisplayData dd;
		copy_from_user(&dd, (void*) value, sizeof(DisplayData));
#ifdef DEBUG
		printk(
				"Display values: FL %x FC %x FR %x RL %x RR %x\nPing %x conStat %x\nML %x MR %x\ncurrentLoad %x\n",
				dd.distanceSensors.distFrontLeft,
				dd.distanceSensors.distFrontCenter,
				dd.distanceSensors.distFrontRight,
				dd.distanceSensors.distRearLeft,
				dd.distanceSensors.distRearRight, dd.connectionStatus.ping,
				dd.connectionStatus.connectionStatus,
				dd.motorStatus.positionLeft, dd.motorStatus.positionRight,
				dd.currentLoad);
#endif
		printk("before update\n");
		lcd_update_display_data(dd);
		printk("after update\n");

	}
#ifdef DEBUG
	printk("control was %i\n", control);
#endif
	return 0;
}

static struct file_operations pugs_fops = { .owner = THIS_MODULE, .open =
		st7565_open, .release = st7565_close, .read = st7565_read,
		.unlocked_ioctl = lcd_control, .write = st7565_write };

static int __init st7565_init(void)
	{
	// allocate a buffer and zero it out
		rx_buffer = kmalloc(BUFFER_SIZE,  GFP_KERNEL);
		memset(rx_buffer, 0, BUFFER_SIZE);

// register a character device
if (alloc_chrdev_region(&first, 0, 1, "st7565") < 0)
		{
			return -1;
		}
		if ((cl = class_create(THIS_MODULE, "chardrv")
) == NULL)
{
	unregister_chrdev_region(first, 1);
	return -1;
}
if (device_create(cl, NULL, first, NULL, "st7565") == NULL)
{
	class_destroy(cl);
	unregister_chrdev_region(first, 1);
	return -1;
}
cdev_init(&c_dev, &pugs_fops);
if (cdev_add(&c_dev, first, 1) == -1)
{
	device_destroy(cl, first);
	class_destroy(cl);
	unregister_chrdev_region(first, 1);
	return -1;
}

// request access to GPIO, set them all as outputs (initially low)
int err, i = 0;
for(i = 0; i < st7565_gpio_pin_info.num_pins; i++) {
	err = gpio_request(st7565_gpio_pins[i].gpio, st7565_gpio_pins[i].name);
	if(err) {
		printk("Could not get access to GPIO %i, error code: %i\n", st7565_gpio_pins[i].gpio, err);
	}
	err = gpio_direction_output(st7565_gpio_pins[i].gpio, 0);
	if(err) {
		printk("Could not set value of GPIO %i, error code: %i\n", st7565_gpio_pins[i].gpio, err);
	}
}

// initialize display
st7565_init_lcd();

lcd_intro();

lcd_ascii5x7_string(0,10,"not connected...");
// ready to go!
printk("st7565 registered!\n");

return 0;
}

static void __exit st7565_exit(void)
{
lcd_ascii5x7_string(0,1,"shutting down...");
// release buffer
if (rx_buffer) {
	kfree(rx_buffer);
}

// release GPIO
int i = 0;
for(i = 0; i < st7565_gpio_pin_info.num_pins; i++) {
	gpio_free(st7565_gpio_pins[i].gpio);
}

// unregister character device
cdev_del(&c_dev);
device_destroy(cl, first);
class_destroy(cl);
unregister_chrdev_region(first, 1);
printk("st7565 unregistered\n");
}

void st7565_deinit(void) {
#ifdef DEBUG
printk("deiniting...\n");
#endif
lcd_clear();
gpio_set_value(ST7565_AP, 1);
gpio_set_value(ST7565_RST, 0);
}

void st7565_init_lcd(void) {
printk("configuring st7565\n");
gpio_set_value(ST7565_CS, 0);
/**/
//gpio_set_value(ST7565_CS, 0);
/**/

// set color pin
gpio_set_value(ST7565_AP, 1);
gpio_set_value(ST7565_RST, 0);
gpio_set_value(ST7565_RST, 1);

lcd_transfer_data(0xe2, 0); //Internal reset

lcd_transfer_data(0xa2, 0); //Sets the LCD drive voltage bias ratio
//A2: 1/9 bias
//A3: 1/7 bias (ST7565V)

lcd_transfer_data(0xa0, 0); //Sets the display RAM address SEG output correspondence
//A0: normal
//A1: reverse

lcd_transfer_data(0xc8, 0);	   //#Select COM output scan direction
//C0~C7: normal direction
//C8~CF: reverse direction

lcd_transfer_data(0xa4, 0);	   //#Display all points ON/OFF
//A4: normal display
//A5: all points ON

lcd_transfer_data(0xa6, 0);   //#Sets the LCD display normal/reverse
//A6: normal
//A7: reverse

lcd_transfer_data(0x2F, 0);   //#select internal power supply operating mode
//28~2F: Operating mode

lcd_transfer_data(0x40, 0);   //#Display start line set
//40~7F Display start address

lcd_transfer_data(0x20, 0);	//#V5 voltage regulator internal resistor ratio set(contrast)
//20~27 small~large

lcd_transfer_data(0x81, 0);	   //#Electronic volume mode set
//81: Set the V5 output voltage

lcd_transfer_data(0x30, 0);	   //#Electronic volume register set
//00~3F: electronic volume register

lcd_transfer_data(0xaf, 0);  //#Display ON/OFF
//AF: ON
//AE: OFF
/**/
//gpio_set_value(ST7565_CS, 1);
/**/
lcd_clear();
printk("st7565 configured\n");
}

void lcd_ascii5x7_string(unsigned int xPos, unsigned int yPos,
	unsigned char * str) {
gpio_set_value(ST7565_CS, 0);
if (str) {
	int i = 0;
	for (i = 0; i < strlen(str); i++) {
		lcd_ascii5x7(xPos, yPos + (i * 6), str[i]);
	}
}
gpio_set_value(ST7565_CS, 1);
}

void lcd_ascii5x7(unsigned int xPos, unsigned int yPos, unsigned char c) {
c -= 32; // adjust for the fact that our table of characters is not really ASCII
lcd_set_page(xPos, yPos);
unsigned int i = 0;
for (i = 0; i < 5; i++) {
	lcd_transfer_data(font5x7[i + (c * 5)], 1);
}
}

void lcd_clear(void) {
unsigned int i = 0;
for (i = 0; i < LCD_PAGES; i++) {
	lcd_set_page(i, 0);
	unsigned int j = 0;
	for (j = 0; j < LCD_COLS; j++) {
		lcd_transfer_data(0x0, 1);
	}
}
}

void lcd_set_page(unsigned char page, unsigned int column) {
unsigned int lsb = column & 0x0f;
unsigned int msb = column & 0xf0;
msb = msb >> 4;
msb = msb | 0x10;
page = page | 0xb0;
lcd_transfer_data(page, 0);
lcd_transfer_data(msb, 0);
lcd_transfer_data(lsb, 0);
}

void lcd_transfer_data(unsigned char value, unsigned int A0) {
gpio_set_value(ST7565_CS, 0);
gpio_set_value(ST7565_CLK, 1);
if (A0) {
	gpio_set_value(ST7565_A0, 1);
} else {
	gpio_set_value(ST7565_A0, 0);
}
lcd_byte(value);
gpio_set_value(ST7565_CS, 1);
}

void lcd_byte(unsigned char bits) {
unsigned char tmp = bits;
int i = 0;
for (i = 0; i < 8; i++) {
	gpio_set_value(ST7565_CLK, 0);
	if (tmp & 0x80) {
		gpio_set_value(ST7565_SI, 1);
	} else {
		gpio_set_value(ST7565_SI, 0);
	}
	tmp = (tmp << 1);
	gpio_set_value(ST7565_CLK, 1);
}
}

void lcd_write_banner(void) {
gpio_set_value(ST7565_CS, 0);
// this prints the startup banner
// first printed page -> don't know where this offset is comming from!
lcd_set_page(5, 50);
// l
lcd_transfer_data(0x7f, 1);
lcd_transfer_data(0x80, 1);
lcd_transfer_data(0x80, 1);
lcd_transfer_data(0x80, 1);

// i
lcd_transfer_data(0x88, 1);
lcd_transfer_data(0xFA, 1);
lcd_transfer_data(0x88, 1);
lcd_transfer_data(0x88, 1);

// t
lcd_transfer_data(0x88, 1);
lcd_transfer_data(0x7F, 1);
lcd_transfer_data(0x88, 1);
lcd_transfer_data(0x88, 1);

// t
lcd_transfer_data(0x88, 1);
lcd_transfer_data(0x7F, 1);
lcd_transfer_data(0x88, 1);
lcd_transfer_data(0x88, 1);
lcd_transfer_data(0x80, 1);

// l
lcd_transfer_data(0x7f, 1);
lcd_transfer_data(0x80, 1);
lcd_transfer_data(0x80, 1);
lcd_transfer_data(0x80, 1);

// e
lcd_transfer_data(0x70, 1);
lcd_transfer_data(0xA8, 1);
lcd_transfer_data(0xA8, 1);
lcd_transfer_data(0xB0, 1);
lcd_transfer_data(0x80, 1);
lcd_transfer_data(0x80, 1);
lcd_transfer_data(0x80, 1);

// first printed main page
// R
lcd_set_page(7, 30);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0x87, 1);
lcd_transfer_data(0x87, 1);
lcd_transfer_data(0x87, 1);
lcd_transfer_data(0x87, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFE, 1);
lcd_transfer_data(0x7C, 1);

lcd_transfer_data(0x00, 1);

// O
lcd_transfer_data(0xFE, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0x07, 1);
lcd_transfer_data(0x07, 1);
lcd_transfer_data(0x07, 1);
lcd_transfer_data(0x07, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFE, 1);

lcd_transfer_data(0x00, 1);

// O
lcd_transfer_data(0xFE, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0x07, 1);
lcd_transfer_data(0x07, 1);
lcd_transfer_data(0x07, 1);
lcd_transfer_data(0x07, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFE, 1);

lcd_transfer_data(0x00, 1);

// T
lcd_transfer_data(0x07, 1);
lcd_transfer_data(0x07, 1);
lcd_transfer_data(0x07, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0x07, 1);
lcd_transfer_data(0x07, 1);
lcd_transfer_data(0x07, 1);

lcd_transfer_data(0x00, 1);

// b
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xC0, 1);

lcd_transfer_data(0x00, 1);

// o
lcd_transfer_data(0xC0, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xC0, 1);

lcd_transfer_data(0x00, 1);

// t
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0x00, 1);

// second printed main page
// R
lcd_set_page(0, 30);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0x07, 1);
lcd_transfer_data(0x07, 1);
lcd_transfer_data(0x07, 1);
lcd_transfer_data(0x07, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFE, 1);

lcd_transfer_data(0x00, 1);

// O
lcd_transfer_data(0x7F, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0x7F, 1);

lcd_transfer_data(0x00, 1);

// O
lcd_transfer_data(0x7F, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0x7F, 1);

lcd_transfer_data(0x00, 1);

// O
lcd_transfer_data(0x00, 1);
lcd_transfer_data(0x00, 1);
lcd_transfer_data(0x00, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0x00, 1);
lcd_transfer_data(0x00, 1);
lcd_transfer_data(0x00, 1);

lcd_transfer_data(0x00, 1);

// b
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0x7F, 1);

lcd_transfer_data(0x00, 1);

// o
lcd_transfer_data(0x7F, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0x7F, 1);

lcd_transfer_data(0x00, 1);

// t
lcd_transfer_data(0x00, 1);
lcd_transfer_data(0x00, 1);
lcd_transfer_data(0x7F, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xFF, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xE0, 1);
lcd_transfer_data(0xE0, 1);
gpio_set_value(ST7565_CS, 1);
}

module_init(st7565_init);
module_exit(st7565_exit);
MODULE_LICENSE("GPL")
;
