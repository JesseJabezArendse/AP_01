###############################################################################
# ARM None-EABI Toolchain File
# Cross-platform toolchain configuration for STM32 development
###############################################################################

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Try to find ARM toolchain
set(ARM_TOOLCHAIN_PREFIX "arm-none-eabi")

# Set executable suffix based on host platform
if(WIN32)
    set(TOOLCHAIN_SUFFIX ".exe")
else()
    set(TOOLCHAIN_SUFFIX "")
endif()

# Search paths for ARM toolchain (in order of priority)
set(TOOLCHAIN_SEARCH_PATHS
    # Local tools directory
    "${CMAKE_CURRENT_LIST_DIR}/../tools/arm-gcc-toolchain/bin"
    "${CMAKE_CURRENT_LIST_DIR}/../tools/arm-gnu-toolchain/bin"
    # STM32CubeCLT installation (common on macOS/Linux)
    "/opt/ST/STM32CubeCLT_1.18.0/GNU-tools-for-STM32/bin"
    "/opt/ST/STM32CubeCLT/GNU-tools-for-STM32/bin"
    # System PATH
    ENV PATH
)

# Find ARM GCC compiler
find_program(ARM_GCC
    NAMES ${ARM_TOOLCHAIN_PREFIX}-gcc${TOOLCHAIN_SUFFIX}
    PATHS ${TOOLCHAIN_SEARCH_PATHS}
    NO_DEFAULT_PATH
)

if(NOT ARM_GCC)
    # Fallback to system PATH
    find_program(ARM_GCC
        NAMES ${ARM_TOOLCHAIN_PREFIX}-gcc${TOOLCHAIN_SUFFIX}
    )
endif()

if(NOT ARM_GCC)
    message(FATAL_ERROR 
        "ARM toolchain not found. Please install arm-none-eabi-gcc.\n"
        "  macOS: brew install --cask gcc-arm-embedded or install STM32CubeCLT\n"
        "  Linux: sudo apt install gcc-arm-none-eabi or install STM32CubeCLT\n"
        "  Windows: Install STM32CubeCLT or download from ARM website"
    )
endif()

# Get toolchain directory
get_filename_component(ARM_TOOLCHAIN_DIR ${ARM_GCC} DIRECTORY)
message(STATUS "Found ARM toolchain: ${ARM_TOOLCHAIN_DIR}")

# Set compilers and tools
set(CMAKE_C_COMPILER ${ARM_TOOLCHAIN_DIR}/${ARM_TOOLCHAIN_PREFIX}-gcc${TOOLCHAIN_SUFFIX} CACHE FILEPATH "C compiler")
set(CMAKE_CXX_COMPILER ${ARM_TOOLCHAIN_DIR}/${ARM_TOOLCHAIN_PREFIX}-g++${TOOLCHAIN_SUFFIX} CACHE FILEPATH "C++ compiler")
set(CMAKE_ASM_COMPILER ${ARM_TOOLCHAIN_DIR}/${ARM_TOOLCHAIN_PREFIX}-gcc${TOOLCHAIN_SUFFIX} CACHE FILEPATH "ASM compiler")

# Set binutils
set(CMAKE_AR ${ARM_TOOLCHAIN_DIR}/${ARM_TOOLCHAIN_PREFIX}-ar${TOOLCHAIN_SUFFIX} CACHE FILEPATH "Archiver")
set(CMAKE_OBJCOPY ${ARM_TOOLCHAIN_DIR}/${ARM_TOOLCHAIN_PREFIX}-objcopy${TOOLCHAIN_SUFFIX} CACHE FILEPATH "Objcopy tool")
set(CMAKE_OBJDUMP ${ARM_TOOLCHAIN_DIR}/${ARM_TOOLCHAIN_PREFIX}-objdump${TOOLCHAIN_SUFFIX} CACHE FILEPATH "Objdump tool")
set(CMAKE_SIZE ${ARM_TOOLCHAIN_DIR}/${ARM_TOOLCHAIN_PREFIX}-size${TOOLCHAIN_SUFFIX} CACHE FILEPATH "Size tool")
set(CMAKE_RANLIB ${ARM_TOOLCHAIN_DIR}/${ARM_TOOLCHAIN_PREFIX}-ranlib${TOOLCHAIN_SUFFIX} CACHE FILEPATH "Ranlib tool")
set(CMAKE_STRIP ${ARM_TOOLCHAIN_DIR}/${ARM_TOOLCHAIN_PREFIX}-strip${TOOLCHAIN_SUFFIX} CACHE FILEPATH "Strip tool")

# Don't search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Prevent CMake from testing the toolchain
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)
set(CMAKE_ASM_COMPILER_WORKS 1)

# Set the CMAKE_TRY_COMPILE_TARGET_TYPE to STATIC_LIBRARY to avoid linker issues during try_compile
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)






