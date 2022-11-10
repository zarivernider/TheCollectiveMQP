################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../test.c 

C_DEPS += \
./test.d 

OBJS += \
./test.o 


# Each subdirectory must supply rules for building sources it contributes
test.o: ../test.c subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -I/opt/poky/1.8/sysroots/cortexa8hf-vfp-neon-poky-linux-gnueabi/usr/include -O0 -g3 -Wall -c -march=armv7-a -mfloat-abi=hard -mfpu=neon -mtune=cortex-a8 --sysroot=/opt/poky/1.8/sysroots/cortexa8hf-vfp-neon-poky-linux-gnueabi -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean--2e-

clean--2e-:
	-$(RM) ./test.d ./test.o

.PHONY: clean--2e-

