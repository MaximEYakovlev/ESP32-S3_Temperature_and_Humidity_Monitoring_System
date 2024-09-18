# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/maxim/esp/esp-idf/components/bootloader/subproject"
  "/home/maxim/ESP32-S3_SHT40_webserver/build/bootloader"
  "/home/maxim/ESP32-S3_SHT40_webserver/build/bootloader-prefix"
  "/home/maxim/ESP32-S3_SHT40_webserver/build/bootloader-prefix/tmp"
  "/home/maxim/ESP32-S3_SHT40_webserver/build/bootloader-prefix/src/bootloader-stamp"
  "/home/maxim/ESP32-S3_SHT40_webserver/build/bootloader-prefix/src"
  "/home/maxim/ESP32-S3_SHT40_webserver/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/maxim/ESP32-S3_SHT40_webserver/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/maxim/ESP32-S3_SHT40_webserver/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
