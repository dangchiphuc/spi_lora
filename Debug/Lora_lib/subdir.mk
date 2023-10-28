################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Lora_lib/LoRa.c 

OBJS += \
./Lora_lib/LoRa.o 

C_DEPS += \
./Lora_lib/LoRa.d 


# Each subdirectory must supply rules for building sources it contributes
Lora_lib/%.o Lora_lib/%.su Lora_lib/%.cyclo: ../Lora_lib/%.c Lora_lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/dangc/OneDrive/Desktop/LoRa/LoRa" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Lora_lib

clean-Lora_lib:
	-$(RM) ./Lora_lib/LoRa.cyclo ./Lora_lib/LoRa.d ./Lora_lib/LoRa.o ./Lora_lib/LoRa.su

.PHONY: clean-Lora_lib

