################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/flight/imu.c 

OBJS += \
./Drivers/flight/imu.o 

C_DEPS += \
./Drivers/flight/imu.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/flight/%.o Drivers/flight/%.su: ../Drivers/flight/%.c Drivers/flight/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Dipterv_Drone/Drivers/BMI088/Inc" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Dipterv_Drone/Drivers/BMP388" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Dipterv_Drone/Drivers/BMM150" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Dipterv_Drone/Drivers/ESC" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Dipterv_Drone/Drivers/OrientationFilter" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Dipterv_Drone/Drivers/OriIMU" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Dipterv_Drone/Drivers/ahrs" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Dipterv_Drone/Drivers/Altitude_filter" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Dipterv_Drone/Drivers/drivers" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Dipterv_Drone/Drivers/flight" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Dipterv_Drone/Drivers/rx" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Dipterv_Drone/Drivers/pg" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Dipterv_Drone/Drivers/telemetry" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Dipterv_Drone/Drivers/common" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-flight

clean-Drivers-2f-flight:
	-$(RM) ./Drivers/flight/imu.d ./Drivers/flight/imu.o ./Drivers/flight/imu.su

.PHONY: clean-Drivers-2f-flight

