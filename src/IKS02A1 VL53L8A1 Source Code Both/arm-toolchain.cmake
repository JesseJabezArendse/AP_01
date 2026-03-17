# ARM GCC Toolchain file for STM32F411
# This file sets up the cross-compilation environment for ARM Cortex-M4

# Disable compiler checks BEFORE any language declarations
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Specify the cross compiler
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR cortex-m4)
set(CMAKE_CROSSCOMPILING TRUE)

# Specify the toolchain - use full paths or environment detection
set(TOOLCHAIN_PREFIX arm-none-eabi-)
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_AR ${TOOLCHAIN_PREFIX}ar)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}objdump)
set(CMAKE_SIZE ${TOOLCHAIN_PREFIX}size)

# Set compiler IDs to skip detection
set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_C_COMPILER_ID_RUN 1)
set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID_RUN 1)
set(CMAKE_CXX_COMPILER_FORCED TRUE)
set(CMAKE_ASM_COMPILER_ID GNU)
set(CMAKE_ASM_COMPILER_ID_RUN 1)
set(CMAKE_ASM_COMPILER_FORCED TRUE)

# Force compiler to work
set(CMAKE_C_COMPILER_WORKS 1 CACHE INTERNAL "")
set(CMAKE_CXX_COMPILER_WORKS 1 CACHE INTERNAL "")
set(CMAKE_ASM_COMPILER_WORKS 1 CACHE INTERNAL "")

# Skip all compiler checks
set(CMAKE_SKIP_LINK_EXECUTABLE_PLATFORM_CHECK TRUE)
set(CMAKE_SKIP_RPATH TRUE)
set(CMAKE_SYSTEM_IGNORE_PATH /usr/lib /usr/include)

# Avoid compiler tests
string(APPEND CMAKE_C_FLAGS_INIT " -x c")
string(APPEND CMAKE_CXX_FLAGS_INIT " -x c++")
