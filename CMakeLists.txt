# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP_ICM42688)

# ULP RISC-V 程序构建
set(ULP_SRCS
    "ulp/ulp_main.c"
)
set(EXTRA_COMPONENT_DIRS "")
