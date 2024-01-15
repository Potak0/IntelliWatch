# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/Espressif/442/frameworks/esp-idf-v4.4.2/components/ulp/cmake"
  "D:/ESP_Project/IntelliWatch/build/esp-idf/main/ulp_main"
  "D:/ESP_Project/IntelliWatch/build/esp-idf/main/ulp_main-prefix"
  "D:/ESP_Project/IntelliWatch/build/esp-idf/main/ulp_main-prefix/tmp"
  "D:/ESP_Project/IntelliWatch/build/esp-idf/main/ulp_main-prefix/src/ulp_main-stamp"
  "D:/ESP_Project/IntelliWatch/build/esp-idf/main/ulp_main-prefix/src"
  "D:/ESP_Project/IntelliWatch/build/esp-idf/main/ulp_main-prefix/src/ulp_main-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/ESP_Project/IntelliWatch/build/esp-idf/main/ulp_main-prefix/src/ulp_main-stamp/${subDir}")
endforeach()
