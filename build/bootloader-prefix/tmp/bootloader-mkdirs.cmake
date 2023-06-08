# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/kev/esp/esp-idf/components/bootloader/subproject"
  "/home/kev/eclipse-workspace/Proyecto_ARQUI/build/bootloader"
  "/home/kev/eclipse-workspace/Proyecto_ARQUI/build/bootloader-prefix"
  "/home/kev/eclipse-workspace/Proyecto_ARQUI/build/bootloader-prefix/tmp"
  "/home/kev/eclipse-workspace/Proyecto_ARQUI/build/bootloader-prefix/src/bootloader-stamp"
  "/home/kev/eclipse-workspace/Proyecto_ARQUI/build/bootloader-prefix/src"
  "/home/kev/eclipse-workspace/Proyecto_ARQUI/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/kev/eclipse-workspace/Proyecto_ARQUI/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/kev/eclipse-workspace/Proyecto_ARQUI/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
