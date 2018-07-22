// Define the entry point of the program
.origin 0
.entrypoint START

// Address of the io controller for GPIO1 and GPIO2
#define GPIO1 0x4804C000
#define GPIO2 0x481AC000
#define GPIO3 0x481AE000

// Offset address for the output enable register of the gpio controller
#define GPIO_OE 0x134

// Offset address for the data in/out register of the gpio controller
#define GPIO_DATAIN 0x138
#define GPIO_CLEARDATAOUT 0x190
#define GPIO_SETDATAOUT 0x194

// PRU interrupt for PRU0 and PRU1
#define PRU0_ARM_INTERRUPT 19
#define PRU1_ARM_INTERRUPT 20

// gpio2[11] P8_42
#define BIT_ECHO 0x0B

// gpio2[14] P8_37
#define BIT_TRIGGER 0x0E

#define delay r0
#define roundtrip r4


START:

	// Clear the STANDBY_INIT bit in the SYSCFG register
	// otherwise the PRU will not be able to write outside the PRU memory space
	// and to the Beaglebone pins
	LBCO r0, C4, 4, 4
	CLR r0, r0, 4
	SBCO r0, C4, 4, 4
	
	// Make constant 24 (c24) point to the beginning of PRU1 data ram
	MOV r0, 0x00000000
	MOV r1, 0x24020
	SBBO r0, r1, 0, 4
	
	// Enable trigger as output and echo as input (clear BIT_TRIGGER and set BIT_ECHO of output enable)
	MOV r3, GPIO2 | GPIO_OE
	LBBO r2, r3, 0, 4
	CLR r2, BIT_TRIGGER
	SET r2, BIT_ECHO
	SBBO r2, r3, 0, 4

TRIGGER:

	// Fire the sonar
	// Set trigger pin to high
	MOV r2, 1<<BIT_TRIGGER
	MOV r3, GPIO2 | GPIO_SETDATAOUT
	SBBO r2, r3, 0, 4
	
	// Delay 10 microseconds (200 MHz / 2 instructions = 10 ns per loop, 10 us = 1000 loops) 
	MOV delay, 1000
TRIGGER_DELAY:
	SUB delay, delay, 1
	QBNE TRIGGER_DELAY, delay, 0
	
	// Set trigger pin to low
	MOV r2, 1<<BIT_TRIGGER
	MOV r3, GPIO2 | GPIO_CLEARDATAOUT
	SBBO r2, r3, 0, 4
	
	// Wait for BIT_ECHO to go high, i.e. wait for the echo cycle to start
	MOV r3, GPIO2 | GPIO_DATAIN
WAIT_ECHO:
	// Read the GPIO until BIT_ECHO goes high
	LBBO r2, r3, 0, 4
	QBBC WAIT_ECHO, r2, BIT_ECHO
	
	// roundtrip measures the echo duration in microseconds, resolution is 1us
	MOV roundtrip, 0

SAMPLE_ECHO:

	// Delay 1 microsecond (adjusted because it takes time to query the GPIO pin)
	MOV delay, 76
SAMPLE_ECHO_DELAY:
	SUB delay, delay, 1
	QBNE SAMPLE_ECHO_DELAY, delay, 0
	
	// Add 1us to the roundtrip counter
	ADD roundtrip, roundtrip, 1
	
	// Read GPIO until BIT_ECHO goes low
	LBBO r2, r3, 0, 4
	QBBS SAMPLE_ECHO, r2, BIT_ECHO

	// Echo is complete
	// Store the microsecond count in the PRU's data ram so C program can read it
	SBCO roundtrip, c24, 0x10, 4
	
	// Trigger the PRU1 interrupt (C program gets the event)
	MOV r31.b0, PRU1_ARM_INTERRUPT+16
	
	// Delay to allow sonar to stop resonating and sound burst to decay in environment
	MOV delay, 3000000
RESET_DELAY:
	SUB delay, delay, 1
	QBNE RESET_DELAY, delay, 0
	
	// Jump back to triggering the sonar
	JMP TRIGGER
	
	HALT

