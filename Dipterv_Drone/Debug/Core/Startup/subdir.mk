################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32f405rgtx.s 

OBJS += \
./Core/Startup/startup_stm32f405rgtx.o 

S_DEPS += \
./Core/Startup/startup_stm32f405rgtx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Drone/Dipterv_Drone/Drivers/BMI088/Inc" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Drone/Dipterv_Drone/Debug/Drivers/BMM150" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Drone/Dipterv_Drone/Drivers/ESC" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Drone/Dipterv_Drone/Drivers/OrientationFilter" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Drone/Dipterv_Drone/Drivers/OriIMU" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Drone/Dipterv_Drone/Drivers/ahrs" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Drone/Dipterv_Drone/Drivers/Altitude_filter" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Drone/Dipterv_Drone/Drivers/HCSR04" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Drone/Dipterv_Drone/Drivers/bmm150_API" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32f405rgtx.d ./Core/Startup/startup_stm32f405rgtx.o

.PHONY: clean-Core-2f-Startup

