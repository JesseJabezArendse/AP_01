cmake_minimum_required(VERSION 3.22)

project(NucleoMX_IKS02A1_VL53L1A1 C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_FLAGS_INIT "${CMAKE_C_FLAGS_INIT} -Wall -fdata-sections -ffunction-sections")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

if(NOT CMAKE_OBJCOPY)
    set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
endif()
if(NOT CMAKE_SIZE)
    set(CMAKE_SIZE arm-none-eabi-size)
endif()

set(MCU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${MCU_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${MCU_FLAGS}")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Og -g3")

set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/stm32f411retx_flash.ld)

set(C_DEFS
    USE_HAL_DRIVER
    STM32F411xE
)

set(C_INCLUDES
    Core/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include
    Drivers/BSP/Components/Common
    Drivers/BSP/Components/iis2dlpc
    Drivers/BSP/Components/iis2mdc
    Drivers/BSP/Components/ism330dhcx
    Drivers/BSP/IKS02A1
    X-CUBE-MEMS1/Target
)

set(C_SOURCES
    Core/Src/main.c
    Core/Src/IKS02A1_Simulink.c
    Core/Src/VL53L1A1_Simulink.c
    Core/Src/VL53L1X_api.c
    Core/Src/VL53L1X_calibration.c
    Core/Src/X-NUCLEO-53L1A1.c
    Core/Src/vl53l1_platform.c
    Core/Src/stm32f4xx_nucleo_bus.c
    Core/Src/stm32f4xx_it.c
    Core/Src/stm32f4xx_hal_msp.c
    Drivers/BSP/Components/iis2dlpc/iis2dlpc_reg.c
    Drivers/BSP/Components/iis2dlpc/iis2dlpc.c
    Drivers/BSP/Components/iis2mdc/iis2mdc_reg.c
    Drivers/BSP/Components/iis2mdc/iis2mdc.c
    Drivers/BSP/Components/ism330dhcx/ism330dhcx_reg.c
    Drivers/BSP/Components/ism330dhcx/ism330dhcx.c
    Drivers/BSP/IKS02A1/iks02a1_motion_sensors.c
    Drivers/BSP/IKS02A1/iks02a1_motion_sensors_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    Core/Src/system_stm32f4xx.c
    Core/Src/sysmem.c
    Core/Src/syscalls.c
)

set(ASM_SOURCES
    startup_stm32f411xe.s
)

add_executable(${PROJECT_NAME}.elf ${C_SOURCES} ${ASM_SOURCES})

set_target_properties(${PROJECT_NAME}.elf PROPERTIES
    LINKER_LANGUAGE C
)

target_include_directories(${PROJECT_NAME}.elf PRIVATE ${C_INCLUDES})

target_compile_definitions(${PROJECT_NAME}.elf PRIVATE ${C_DEFS})

target_link_options(${PROJECT_NAME}.elf PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -specs=nano.specs
    "-T${LINKER_SCRIPT}"
    "-Wl,-Map=${CMAKE_BINARY_DIR}/bin/${PROJECT_NAME}.map,--cref"
    -Wl,--gc-sections
)

target_link_libraries(${PROJECT_NAME}.elf PRIVATE c m nosys)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${PROJECT_NAME}.elf>
            $<TARGET_FILE_DIR:${PROJECT_NAME}.elf>/${PROJECT_NAME}.hex
    COMMENT "Generating ${PROJECT_NAME}.hex"
)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary -S
            $<TARGET_FILE:${PROJECT_NAME}.elf>
            $<TARGET_FILE_DIR:${PROJECT_NAME}.elf>/${PROJECT_NAME}.bin
    COMMENT "Generating ${PROJECT_NAME}.bin"
)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
    COMMENT "Size of ${PROJECT_NAME}.elf"
)

add_custom_target(clean-all
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_clean.cmake
)
