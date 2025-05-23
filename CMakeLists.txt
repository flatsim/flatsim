cmake_minimum_required(VERSION 3.15)
set(project_name multiverse)
project(${project_name} VERSION 0.1.1 LANGUAGES CXX)

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

# --------------------------------------------------------------------------------------------------
set(ext_deps)

set(MULI_BUILD_DEMO OFF)
fetchcontent_declare( muli GIT_REPOSITORY https://github.com/Sopiro/Muli )
FetchContent_MakeAvailable(muli)
include_directories(${muli_SOURCE_DIR}/include)
list(APPEND ext_deps muli)

FetchContent_Declare(concord GIT_REPOSITORY https://github.com/smolfetch/concord.git GIT_TAG main)
FetchContent_MakeAvailable(concord)
list(APPEND ext_deps concord::concord)

FetchContent_Declare(pigment GIT_REPOSITORY https://github.com/smolfetch/pigment.git GIT_TAG main)
FetchContent_MakeAvailable(pigment)
list(APPEND ext_deps pigment::pigment)

FetchContent_Declare(entropy GIT_REPOSITORY https://github.com/smolfetch/entropy.git GIT_TAG main)
FetchContent_MakeAvailable(entropy)
list(APPEND ext_deps entropy::entropy)

FetchContent_Declare(geoson GIT_REPOSITORY https://github.com/smolfetch/geoson.git GIT_TAG main)
FetchContent_MakeAvailable(geoson)
list(APPEND ext_deps geoson::geoson)

FetchContent_Declare(geotiv GIT_REPOSITORY https://github.com/smolfetch/geotiv.git GIT_TAG main)
FetchContent_MakeAvailable(geotiv)
list(APPEND ext_deps geotiv::geotiv)

FetchContent_Declare(farmtrax GIT_REPOSITORY https://github.com/smolfetch/farmtrax.git GIT_TAG main)
FetchContent_MakeAvailable(farmtrax)
list(APPEND ext_deps farmtrax::farmtrax)

FetchContent_Declare(rerun_sdk URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)
list(APPEND ext_deps rerun_sdk)

FetchContent_Declare(spdlog GIT_REPOSITORY https://github.com/gabime/spdlog.git GIT_TAG v1.15.0)
FetchContent_MakeAvailable(spdlog)
list(APPEND ext_deps spdlog::spdlog)

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
