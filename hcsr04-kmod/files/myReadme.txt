to get ready with distance sensor code for the hc-sr04 module the dtc (device tree compiler) library needs to be installed.
Follow the flash instructions in .. HowTo_DebianSystem.txt

Disable HDMI (NOT eMMC!):
open /boot/uEnv uncomment the according lines

Pin config:
Echo:
Pin P9_FL1 = GPIO3_14
pr1_pru0_pru_r31_0
Internal GPIO_110
Mux offset 0x190
Bitnumber 0x100 (^32) = 0x04

Trigger:
Pin P9_25 = GPIO3_21
Internal GPIO_FL
Mux offset 0x1ac
Bitnumber 107 (^32) = 0x0B
	
	
Try:
- http://analogdigitallab.org/articles/beaglebone-black-introduction-pru-icss // for c integration of pru

Info:
- Pru interrupts: http://elinux.org/PRUSSv2_Interrupts
- Pru config: http://www.valvers.com/embedded-linux/beaglebone-black/step04-gpio/
