cmake_minimum_required(VERSION 3.15)
project(concord VERSION 0.1.0 LANGUAGES CXX)

project(multiverse)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()
cmake_policy(SET CMP0074 NEW)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(FetchContent)


fetchcontent_declare(
    muli
    GIT_REPOSITORY https://github.com/bresilla/Muli
)
FetchContent_MakeAvailable(muli)
include_directories(${muli_SOURCE_DIR}/include)


add_library(multiverse INTERFACE)

target_include_directories(multiverse INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(multiverse INTERFACE muli)
