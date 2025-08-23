# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/alif-ilhan/esp/v5.5/esp-idf/components/bootloader/subproject"
  "/home/alif-ilhan/Desktop/WRO-FUTURE-ENGINEERS-2025/src/ESP32S3/build/bootloader"
  "/home/alif-ilhan/Desktop/WRO-FUTURE-ENGINEERS-2025/src/ESP32S3/build/bootloader-prefix"
  "/home/alif-ilhan/Desktop/WRO-FUTURE-ENGINEERS-2025/src/ESP32S3/build/bootloader-prefix/tmp"
  "/home/alif-ilhan/Desktop/WRO-FUTURE-ENGINEERS-2025/src/ESP32S3/build/bootloader-prefix/src/bootloader-stamp"
  "/home/alif-ilhan/Desktop/WRO-FUTURE-ENGINEERS-2025/src/ESP32S3/build/bootloader-prefix/src"
  "/home/alif-ilhan/Desktop/WRO-FUTURE-ENGINEERS-2025/src/ESP32S3/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/alif-ilhan/Desktop/WRO-FUTURE-ENGINEERS-2025/src/ESP32S3/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/alif-ilhan/Desktop/WRO-FUTURE-ENGINEERS-2025/src/ESP32S3/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
