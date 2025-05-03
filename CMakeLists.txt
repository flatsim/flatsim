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

set(params
    -Wno-unused-parameter
    -Wno-unused-variable
    -Wno-reorder
    -Wno-strict-aliasing
    -Wno-unused-but-set-variable
)

set(MULI_BUILD_DEMO OFF)
fetchcontent_declare(
    muli
    # GIT_REPOSITORY https://github.com/bresilla/Muli
    GIT_REPOSITORY https://github.com/Sopiro/Muli
)
FetchContent_MakeAvailable(muli)
include_directories(${muli_SOURCE_DIR}/include)

FetchContent_Declare(
  concord
  GIT_REPOSITORY https://github.com/smolfetch/concord.git
  GIT_TAG        main
)
FetchContent_MakeAvailable(concord)

FetchContent_Declare(
  pigment
  GIT_REPOSITORY https://github.com/smolfetch/pigment.git
  GIT_TAG        main
)
FetchContent_MakeAvailable(pigment)

FetchContent_Declare(
    rerun_sdk 
    URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip
)
FetchContent_MakeAvailable(rerun_sdk)


add_library(multiverse INTERFACE)

target_include_directories(multiverse INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(multiverse INTERFACE muli)
target_link_libraries(multiverse INTERFACE concord::concord)
target_link_libraries(multiverse INTERFACE pigment::pigment)


install(
    DIRECTORY include/
    DESTINATION include
)
include_directories(include)

set(exe
  src/mvs.cpp
)

set(lib
    src/multiverse/robot/driver.cpp
    src/multiverse/robot/sensor.cpp
    src/multiverse/robot.cpp
    src/multiverse/world.cpp
    src/multiverse/simulator.cpp
)

set(dependencies
  muli
  concord::concord
  pigment::pigment
  rerun_sdk
)

set(exec_names)


foreach(src_file IN LISTS exe)
  get_filename_component(exec_name "${src_file}" NAME_WE)
  add_executable(${exec_name} "${src_file}")
  foreach(lib_file IN LISTS lib)
    target_compile_options(${exec_name} PRIVATE ${params})
    target_sources(${exec_name} PRIVATE "${lib_file}")
  endforeach()
  target_link_libraries(${exec_name} ${dependencies})
  install(TARGETS ${exec_name} DESTINATION bin)
  list(APPEND exec_names ${exec_name})
endforeach()


foreach(exec IN LISTS exec_names)
  file(REMOVE "${CMAKE_CURRENT_LIST_DIR}/.execs")
  file(WRITE "${CMAKE_CURRENT_LIST_DIR}/.execs")
  get_filename_component(exec_name "${exec}" NAME)
  file(APPEND "${CMAKE_CURRENT_LIST_DIR}/.execs" "${CMAKE_CURRENT_LIST_DIR}/build/${exec_name}\n")
endforeach()

install(TARGETS
  ${exec_names}
  DESTINATION lib/${PROJECT_NAME}
)
