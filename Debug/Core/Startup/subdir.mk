################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32l4r5zitx.s 

S_DEPS += \
./Core/Startup/startup_stm32l4r5zitx.d 

OBJS += \
./Core/Startup/startup_stm32l4r5zitx.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/startup_stm32l4r5zitx.o: ../Core/Startup/startup_stm32l4r5zitx.s
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -c -x assembler-with-cpp -MMD -MP -MF"Core/Startup/startup_stm32l4r5zitx.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"
