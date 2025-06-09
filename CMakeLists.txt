cmake_minimum_required(VERSION 3.15)
set(project_name multiverse)
project(${project_name} VERSION 0.1.1 LANGUAGES CXX)

cmake_policy(SET CMP0074 NEW)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(FetchContent)

# Prevent duplicate dependencies 
set(FETCHCONTENT_QUIET FALSE)
set(FETCHCONTENT_UPDATES_DISCONNECTED ON)

# Build configuration options
option(MULTIVERSE_BUILD_TESTS "Build tests" OFF)
option(MULTIVERSE_BUILD_EXAMPLES "Build examples" ON)

set(params
    -Wno-unused-parameter
    -Wno-unused-variable
    -Wno-reorder
    -Wno-strict-aliasing
    -Wno-unused-but-set-variable
)

# --------------------------------------------------------------------------------------------------
# External Dependencies
set(ext_deps)

# Muli Physics Engine
set(MULI_BUILD_DEMO OFF)
FetchContent_Declare(muli 
    GIT_REPOSITORY https://github.com/Sopiro/Muli
    # Note: No GIT_TAG specified to use default branch
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(muli)
if(NOT muli_POPULATED)
    message(FATAL_ERROR "Failed to fetch muli dependency")
endif()
include_directories(${muli_SOURCE_DIR}/include)
list(APPEND ext_deps muli)

# Rerun Visualization SDK
FetchContent_Declare(rerun_sdk 
    URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip
)
FetchContent_MakeAvailable(rerun_sdk)
if(NOT rerun_sdk_POPULATED)
    message(FATAL_ERROR "Failed to fetch rerun_sdk dependency")
endif()
list(APPEND ext_deps rerun_sdk)

# Logging Library
FetchContent_Declare(spdlog 
    GIT_REPOSITORY https://github.com/gabime/spdlog.git 
    GIT_TAG v1.15.0
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(spdlog)
list(APPEND ext_deps spdlog::spdlog)

# Concord Library
FetchContent_Declare(concord 
    GIT_REPOSITORY https://github.com/smolfetch/concord.git 
    GIT_TAG 1.2.0
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(concord)
list(APPEND ext_deps concord::concord)

# Pigment Library
FetchContent_Declare(pigment 
    GIT_REPOSITORY https://github.com/smolfetch/pigment.git 
    GIT_TAG main
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(pigment)
list(APPEND ext_deps pigment::pigment)

# Entropy Library
FetchContent_Declare(entropy 
    GIT_REPOSITORY https://github.com/smolfetch/entropy.git 
    GIT_TAG develop
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(entropy)
list(APPEND ext_deps entropy::entropy)

# Geoson Library
FetchContent_Declare(geoson 
    GIT_REPOSITORY https://github.com/smolfetch/geoson.git 
    GIT_TAG develop
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(geoson)
list(APPEND ext_deps geoson::geoson)

# Geotiv Library
FetchContent_Declare(geotiv 
    GIT_REPOSITORY https://github.com/smolfetch/geotiv.git 
    GIT_TAG develop
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(geotiv)
list(APPEND ext_deps geotiv::geotiv)

# Farmtrax Library
FetchContent_Declare(farmtrax 
    GIT_REPOSITORY https://github.com/smolfetch/farmtrax.git 
    GIT_TAG main
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(farmtrax)
list(APPEND ext_deps farmtrax::farmtrax)


# --------------------------------------------------------------------------------------------------
install(
    DIRECTORY include/
    DESTINATION include
)
include_directories(include)

# --------------------------------------------------------------------------------------------------
set(int_deps
    src/multiverse/robot/chasis/karosserie.cpp
    src/multiverse/robot/chasis/wheel.cpp
    src/multiverse/robot/chasis/hitch.cpp
    src/multiverse/robot/chasis/chasis.cpp
    src/multiverse/robot/sensor.cpp
    src/multiverse/robot/sensors/gps_sensor.cpp
    src/multiverse/robot.cpp
    src/multiverse/world/layer.cpp
    src/multiverse/world.cpp
    src/multiverse/simulator.cpp
)

set(exe
  src/mvs.cpp
)

set(exec_names)
foreach(src_file IN LISTS exe)
  get_filename_component(exec_name "${src_file}" NAME_WE)
  add_executable(${exec_name} "${src_file}")
  foreach(lib_file IN LISTS int_deps)
    target_compile_options(${exec_name} PRIVATE ${params})
    target_sources(${exec_name} PRIVATE "${lib_file}")
  endforeach()
  target_link_libraries(${exec_name} ${ext_deps})
  install(TARGETS ${exec_name} DESTINATION bin)
  list(APPEND exec_names ${exec_name})
endforeach()


# Generate .execs file with all executable paths
file(REMOVE "${CMAKE_CURRENT_LIST_DIR}/.execs")
file(WRITE "${CMAKE_CURRENT_LIST_DIR}/.execs" "")
foreach(exec IN LISTS exec_names)
  get_filename_component(exec_name "${exec}" NAME)
  file(APPEND "${CMAKE_CURRENT_LIST_DIR}/.execs" "${CMAKE_CURRENT_LIST_DIR}/build/${exec_name}\n")
endforeach()

install(TARGETS
  ${exec_names}
  DESTINATION lib/${PROJECT_NAME}
)
