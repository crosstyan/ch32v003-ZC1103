cmake_minimum_required(VERSION 3.20)
set(CMAKE_TOOLCHAIN_FILE
        ${CMAKE_CURRENT_LIST_DIR}/xpack-riscv-none-elf-gcc.cmake)
project(simple)
enable_language(ASM)
enable_language(C)
enable_language(CXX)

set(FUN_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/fun)
file(GLOB FUN_SRC ${FUN_DIRECTORY}/ch32v003fun/*.c)
include_directories(${FUN_DIRECTORY}/ch32v003fun)
include_directories(${FUN_DIRECTORY}/extralibs)

set(LIB_PRINTF_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/libs/printf)
set(LIB_PRINTF_SRC ${LIB_PRINTF_DIRECTORY}/src/printf/printf.c)
include_directories(${LIB_PRINTF_DIRECTORY}/src)
include_directories(${LIB_PRINTF_DIRECTORY}/src/printf)

#set(ZC1103_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/driver/zc1103)
#set(ZC1103_SRC ${ZC1103_DIRECTORY}/src/rfsystem.cpp)
#include_directories(${ZC1103_DIRECTORY}/inc)

set(LLCC68_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/driver/llcc68)
set(LLCC68_SRC
        ${LLCC68_DIRECTORY}/src/llcc68.cpp
        ${LLCC68_DIRECTORY}/src/Hal.cpp
        ${LLCC68_DIRECTORY}/src/Module.cpp)
include_directories(${LLCC68_DIRECTORY}/inc)

set(CNL_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/libs/cnl)
include_directories(${CNL_DIRECTORY}/include)

#set(RAIDOLIB_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/libs/RadioLib)
#include_directories(${RAIDOLIB_DIRECTORY}/src
#        ${RAIDOLIB_DIRECTORY}/src/modules/LLCC68)
#set(RAIDOLIB_SRC
#        ${RAIDOLIB_DIRECTORY}/src/modules/LLCC68/LLCC68.cpp
#        ${RAIDOLIB_DIRECTORY}/src/modules/SX126x/SX126x.cpp
#        ${RAIDOLIB_DIRECTORY}/src/Module.cpp
#        ${RAIDOLIB_DIRECTORY}/src/Hal.cpp)

set(APP_SRC src/main.cpp src/system_tick.cpp src/utils.cpp src/exti.cpp src/message_wrapper.cpp)
set(GPIO_SRC src/driver/gpio/gpio.cpp)
include_directories(src/inc src/driver/gpio)

set(GIT_DIR_LOOKUP_POLICY ALLOW_LOOKING_ABOVE_CMAKE_SOURCE_DIR)
set(ETL_DIR libs/etl)
add_subdirectory(${ETL_DIR})
include_directories(${ETL_DIR}/include)

set(NANOPB_DIR libs/nanopb)
set(NANOPB_SRC ${NANOPB_DIR}/pb_common.c ${NANOPB_DIR}/pb_decode.c ${NANOPB_DIR}/pb_encode.c)
include_directories(${NANOPB_DIR})

set(NANOPB_GEN_DIR libs/pb_gen)
set(NANOPB_GEN_SRC ${NANOPB_GEN_DIR}/simple.pb.c)
include_directories(${NANOPB_GEN_DIR})

set(LED_SRC src/driver/led/led.cpp src/driver/llcc68/src/llcc68.cpp src/driver/llcc68/inc/radio_sx126x.h)
include_directories(src/driver/led)

add_executable(${CMAKE_PROJECT_NAME}.elf
        ${LLCC68_SRC}
        ${FUN_SRC}
        ${APP_SRC}
        ${LIB_PRINTF_SRC}
        ${NANOPB_SRC}
        ${LED_SRC}
        ${GPIO_SRC}
        ${NANOPB_GEN_SRC}
        ${ZC1103_SRC})
add_compile_definitions(
        SYSTICK_USE_HCLK=1
        PRINTF_ALIAS_STANDARD_FUNCTION_NAMES_HARD=1
)

set_target_properties(${CMAKE_PROJECT_NAME}.elf PROPERTIES
        CXX_STANDARD 23
        CXX_STANDARD_REQUIRED ON
        LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/vendor/Ld/Link.ld
        LINK_FLAGS "-T ${CMAKE_CURRENT_SOURCE_DIR}/vendor/Ld/Link.ld")

target_link_libraries(${CMAKE_PROJECT_NAME}.elf
        PRIVATE
        etl::etl)

# Generate the binary file
add_custom_target(
        bin ALL
        ${TOOLCHAIN_PREFIX}/bin/${TOOLCHAIN}-objcopy -Obinary
        "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.elf"
        "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin"
        DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.elf)

# Echo the size Information
add_custom_target(
        size ALL
        ${TOOLCHAIN_PREFIX}/bin/${TOOLCHAIN}-size "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.elf"
        DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.elf)

# flash with minichlink
add_custom_target(
        flash COMMAND
        minichlink -w ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin flash -b
        DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin)

#add_custom_target(
#        monitor
#        COMMAND minichlink -b -T)
