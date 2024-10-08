# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(CHIP_TARGET "esp32s3")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32-S3_Temperature_and_Humidity_Monitoring_System)

find_package(cjson REQUIRED)
include_directories(${CJSON_INCLUDE_DIRS})
add_executable(server server.c)
target_link_libraries(server cjson mhd pq)