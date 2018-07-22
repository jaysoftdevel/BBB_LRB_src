/*
 * lcd.h
 *
 *  Created on: Dec 23, 2014
 *  Author: done
 */
#include <stdbool.h>

#ifndef LCD_H_
#define LCD_H_

#define DEBUG

#define IOCTL_LCD_TEXT_MODE _IOW(6, 0, unsigned long)
#define IOCTL_LCD_WORKING_MODE _IOW(6, 1, unsigned long)
#define IOCTL_LCD_PIXEL_MODE _IOW(6, 2, unsigned long)
#define IOCTL_LCD_CLEAR_ALL _IOW(6, 3, unsigned long)
#define IOCTL_LCD_SETUP_WORKING_MODE _IOW(6,4, unsigned long)
#define IOCTL_LCD_INTRO _IOW(6,5, unsigned long)

#define LCD_COLS 128
#define LCD_PAGES 8
#define TOKENSIZE 6

extern void write_banner(void);

typedef struct{
	unsigned char distFrontLeft;	
	unsigned char distFrontCenter;
	unsigned char distFrontRight;
	unsigned char distRearLeft;
	unsigned char distRearRight;
}DistanceSensors;

typedef struct{
	unsigned int ping;
	bool connectionStatus;
}ConnectionStatus;

typedef struct{
	unsigned char positionLeft;
	unsigned char positionRight;
}MotorStatus;

typedef struct{
	DistanceSensors distanceSensors;
	ConnectionStatus connectionStatus;
	MotorStatus motorStatus;
	char currentLoad;
}DisplayData;
#endif /* LCD_H_ */
