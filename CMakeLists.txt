cmake_minimum_required(VERSION 3.16)

message(STATUS "IDF Environment Path: $ENV{IDF_PATH}")

set(PYTHON_EXECUTABLE "E:/ESP-IDF/Tools/python_env/idf5.4_py3.11_env/Scripts")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_LIST_DIR}/components)
include_directories(${CMAKE_SOURCE_DIR}/components)

set(CMAKE_CPP_STANDARD 20)
set(CMAKE_CPP_STANDARD_REQUIRED ON)

project(Screen)
