################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/rx/crsf.c \
../Drivers/rx/rx.c 

OBJS += \
./Drivers/rx/crsf.o \
./Drivers/rx/rx.o 

C_DEPS += \
./Drivers/rx/crsf.d \
./Drivers/rx/rx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/rx/%.o Drivers/rx/%.su: ../Drivers/rx/%.c Drivers/rx/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Dipterv_Drone/Drivers/BMI088/Inc" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Dipterv_Drone/Drivers/BMP388" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Dipterv_Drone/Drivers/BMM150" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Dipterv_Drone/Drivers/ESC" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Dipterv_Drone/Drivers/OrientationFilter" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Dipterv_Drone/Drivers/OriIMU" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Dipterv_Drone/Drivers/ahrs" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Dipterv_Drone/Drivers/Altitude_filter" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Dipterv_Drone/Drivers/rx" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Dipterv_Drone/Drivers/pg" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Dipterv_Drone/Drivers/telemetry" -I"C:/Users/Andris/STM32CubeIDE/workspace_1.9.0/Dipterv_Drone/Drivers/HCSR04" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-rx

clean-Drivers-2f-rx:
	-$(RM) ./Drivers/rx/crsf.d ./Drivers/rx/crsf.o ./Drivers/rx/crsf.su ./Drivers/rx/rx.d ./Drivers/rx/rx.o ./Drivers/rx/rx.su

.PHONY: clean-Drivers-2f-rx

