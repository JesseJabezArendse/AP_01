# ARM GCC Toolchain file for STM32F411
# This file sets up the cross-compilation environment for ARM Cortex-M4

# Specify the cross compiler
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR cortex-m4)

# Specify the toolchain
set(TOOLCHAIN_PREFIX arm-none-eabi-)
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_AR ${TOOLCHAIN_PREFIX}ar)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}objdump)
set(CMAKE_SIZE ${TOOLCHAIN_PREFIX}size)

# Find the ARM tools
find_program(CMAKE_C_COMPILER ${CMAKE_C_COMPILER})
find_program(CMAKE_CXX_COMPILER ${CMAKE_CXX_COMPILER})
find_program(CMAKE_ASM_COMPILER ${CMAKE_ASM_COMPILER})
find_program(CMAKE_AR ${CMAKE_AR})
find_program(CMAKE_OBJCOPY ${CMAKE_OBJCOPY})
find_program(CMAKE_OBJDUMP ${CMAKE_OBJDUMP})
find_program(CMAKE_SIZE ${CMAKE_SIZE})

# Set language settings
set(CMAKE_C_COMPILER_WORKS TRUE)
set(CMAKE_CXX_COMPILER_WORKS TRUE)
set(CMAKE_ASM_COMPILER_WORKS TRUE)

# Skip compiler check
set(CMAKE_SKIP_LINK_EXECUTABLE_PLATFORM_CHECK TRUE)
