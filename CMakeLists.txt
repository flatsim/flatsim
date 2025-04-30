cmake_minimum_required(VERSION 3.15)

project(multiverse VERSION 0.1.1 LANGUAGES CXX)

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

set(IMGUI_VERSION 1.91.8)
FetchContent_Declare(
    imgui
    URL https://github.com/ocornut/imgui/archive/refs/tags/v${IMGUI_VERSION}-docking.tar.gz
)
FetchContent_MakeAvailable(imgui)
add_library(imgui STATIC 
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp)
target_include_directories(imgui INTERFACE ${imgui_SOURCE_DIR})


add_library(multiverse INTERFACE)

target_include_directories(multiverse INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(multiverse INTERFACE muli)

install(
    DIRECTORY include/
    DESTINATION include
)
include_directories(include)

set(exe
  src/main.cpp
)

set(lib
  src/world/world.cpp
)

set(exec_names)

foreach(src_file IN LISTS exe)
  get_filename_component(exec_name "${src_file}" NAME_WE)
  add_executable(${exec_name} "${src_file}")
  foreach(lib_file IN LISTS lib)
    target_sources(${exec_name} PRIVATE "${lib_file}")
  endforeach()
  target_link_libraries(${exec_name} multiverse)
  install(TARGETS ${exec_name} DESTINATION bin)
  list(APPEND exec_names ${exec_name})
endforeach()
