################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F0xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
cv03_F030_HAL.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
cv03_F030_HAL.list \

OBJCOPY_BIN += \
cv03_F030_HAL.bin \


# All Target
all: cv03_F030_HAL.elf secondary-outputs

# Tool invocations
cv03_F030_HAL.elf: $(OBJS) $(USER_OBJS) A:\SKOLA\Studium_MPC\1_rocnik_zimni\MPC-MKS\Cviceni\BPC-MKS\cv03_F030_HAL\STM32F030R8TX_FLASH.ld
	arm-none-eabi-gcc -o "cv03_F030_HAL.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"A:\SKOLA\Studium_MPC\1_rocnik_zimni\MPC-MKS\Cviceni\BPC-MKS\cv03_F030_HAL\STM32F030R8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="cv03_F030_HAL.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

cv03_F030_HAL.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "cv03_F030_HAL.list"
	@echo 'Finished building: $@'
	@echo ' '

cv03_F030_HAL.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "cv03_F030_HAL.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
