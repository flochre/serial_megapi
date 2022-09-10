cmake_minimum_required(VERSION 3.9)
project(serial_megapi VERSION 1.0 DESCRIPTION "lib to communicate with the megapi over serial")
include(GNUInstallDirs)

set(package serial_megapi)

# find_package(crypt REQUIRED)
# find_package(RT REQUIRED)
# find_package(Threads REQUIRED)
# find_package(wiringPi REQUIRED)

add_compile_options(-Wall)
add_compile_options(-O3)
add_compile_options(-Winline)
add_compile_options(-pipe)

add_library(${PROJECT_NAME} SHARED 
    src/serial_megapi.c
)

add_executable(serialTest examples/serialTest.c)
target_link_libraries(serialTest ${PROJECT_NAME} wiringPi wiringPiDev pthread m crypt rt)

file(GLOB_RECURSE PUBLIC_INCLUDE_FILES "include/*.h*")

target_include_directories(${PROJECT_NAME} PRIVATE .)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_include_directories(serialTest PUBLIC include)
target_include_directories(${PROJECT_NAME} PRIVATE src)