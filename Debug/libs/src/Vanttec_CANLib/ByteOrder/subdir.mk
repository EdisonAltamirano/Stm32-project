################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../libs/src/Vanttec_CANLib/ByteOrder/inet.cpp 

OBJS += \
./libs/src/Vanttec_CANLib/ByteOrder/inet.o 

CPP_DEPS += \
./libs/src/Vanttec_CANLib/ByteOrder/inet.d 


# Each subdirectory must supply rules for building sources it contributes
libs/src/Vanttec_CANLib/ByteOrder/%.o libs/src/Vanttec_CANLib/ByteOrder/%.su: ../libs/src/Vanttec_CANLib/ByteOrder/%.cpp libs/src/Vanttec_CANLib/ByteOrder/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Z0180064/Vanttec/software/Stm32-project/libs/vanttec_CANLib/src/Vanttec_CANLib" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/Z0180064/Vanttec/software/Stm32-project/libs/vanttec_CANLib/src/Vanttec_CANLib_Linux" -I"C:/Users/Z0180064/Vanttec/software/Stm32-project/libs" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-libs-2f-src-2f-Vanttec_CANLib-2f-ByteOrder

clean-libs-2f-src-2f-Vanttec_CANLib-2f-ByteOrder:
	-$(RM) ./libs/src/Vanttec_CANLib/ByteOrder/inet.d ./libs/src/Vanttec_CANLib/ByteOrder/inet.o ./libs/src/Vanttec_CANLib/ByteOrder/inet.su

.PHONY: clean-libs-2f-src-2f-Vanttec_CANLib-2f-ByteOrder

