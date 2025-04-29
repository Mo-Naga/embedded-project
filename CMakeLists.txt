cmake_minimum_required(VERSION 3.10)

# Project Name
project(AVR_Robot C)

# MCU configuration
set(MCU atmega328p)
set(F_CPU 16000000UL)
set(BAUD 9600)

# Compiler/flags
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_C_COMPILER avr-gcc)
set(CMAKE_C_FLAGS "-Wall -Os -DF_CPU=${F_CPU} -mmcu=${MCU}")

# Source files
set(SOURCES
    main.c
    motor.c
    ultrasonic.c
)

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Executable target (ELF)
add_executable(${PROJECT_NAME}.elf ${SOURCES})

# HEX file
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND avr-objcopy -O ihex -R .eeprom ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMENT "Generating HEX file"
)

# Size info
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND avr-size --format=avr --mcu=${MCU} ${PROJECT_NAME}.elf
    COMMENT "Displaying size"
)
