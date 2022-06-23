################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
REAL_TIME_OPERATİNG_SYSTEM.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
REAL_TIME_OPERATİNG_SYSTEM.list \

OBJCOPY_BIN += \
REAL_TIME_OPERATİNG_SYSTEM.bin \


# All Target
all: REAL_TIME_OPERATİNG_SYSTEM.elf secondary-outputs

# Tool invocations
REAL_TIME_OPERATİNG_SYSTEM.elf: $(OBJS) $(USER_OBJS) C:\Users\MUSTAFA\STM32CubeIDE\workspace_1.3.0\HCSR04MOTORCONTROL\STM32F407VGTX_FLASH.ld
	arm-none-eabi-gcc -o "REAL_TIME_OPERATİNG_SYSTEM.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\MUSTAFA\STM32CubeIDE\workspace_1.3.0\HCSR04MOTORCONTROL\STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="REAL_TIME_OPERATİNG_SYSTEM.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

REAL_TIME_OPERATİNG_SYSTEM.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "REAL_TIME_OPERATİNG_SYSTEM.list"
	@echo 'Finished building: $@'
	@echo ' '

REAL_TIME_OPERATİNG_SYSTEM.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "REAL_TIME_OPERATİNG_SYSTEM.bin"
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
