################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../st7565.c \
../st7565.mod.c 

O_SRCS += \
../st7565.mod.o \
../st7565.o 

OBJS += \
./st7565.o \
./st7565.mod.o 

C_DEPS += \
./st7565.d \
./st7565.mod.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	arm-linux-gnueabi-gcc -I/media/Data2/Projects/linux-headers-3.14.22-ti-r31/include -I/media/Data2/Projects/linux-headers-3.14.22-ti-r31/arch/arm/include -I/media/Data2/Projects/linux-headers-3.14.22-ti-r31/arch/arm/include/generated/asm -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


