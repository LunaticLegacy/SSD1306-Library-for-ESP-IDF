cmake_minimum_required(VERSION 3.16)

message(STATUS "IDF Environment Path: $ENV{IDF_PATH}")

set(PYTHON_EXECUTABLE "G:/ESP-IDF/Tools/python_env/idf5.4_py3.11_env/Scripts/python.exe")
include(g:/ESP-IDF/IDF/v5.4/esp-idf/tools/cmake/project.cmake)

# 只包含实际使用的库
set(LUNALIBS ${CMAKE_CURRENT_LIST_DIR}/lunalibs)

set(EXTRA_COMPONENT_DIRS 
    ${LUNALIBS}/st7789_luna 
)

include_directories(${CMAKE_SOURCE_DIR}/lunalibs)

set(CMAKE_CPP_STANDARD 20)
set(CMAKE_CPP_STANDARD_REQUIRED ON)

project(Screen)