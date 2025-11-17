cmake_minimum_required(VERSION 3.15)

# set(CMAKE_C_COMPILER gcc-13)
# set(CMAKE_CXX_COMPILER g++-13)

set(project_name flatsim)
project(${project_name} VERSION 0.1.0 LANGUAGES CXX)

add_compile_options(-Wall -Wextra -Wpedantic)
cmake_policy(SET CMP0074 NEW)
cmake_policy(SET CMP0135 NEW)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Performance optimizations
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type")
# Removed -flto to prevent excessive memory usage during compilation
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG -ffast-math" CACHE STRING "Release flags")
set(CMAKE_C_FLAGS_RELEASE "-O3 -march=native -DNDEBUG -ffast-math" CACHE STRING "Release flags")
set(params
    -Wno-reorder
    -Wno-unused-variable
    -Wno-unused-but-set-variable
    -Wno-unused-parameter
)

set(link_options
    "LINKER:--no-as-needed"
)

option(RERUN_DOWNLOAD_AND_BUILD_ARROW "Download and build arrow" OFF)
set(HAS_RERUN ON)

string(TOUPPER ${project_name} project_name_upper)
option(${project_name_upper}_BUILD_EXAMPLES "Build examples" OFF)
option(${project_name_upper}_ENABLE_TESTS "Enable tests" OFF)
option(${project_name_upper}_ENABLE_RUST "Enable Rust components" OFF)
include(FetchContent)

# --------------------------------------------------------------------------------------------------
# Rust Integration with Corrosion
if(${project_name_upper}_ENABLE_RUST)
    message(STATUS "Enabling Rust components via Corrosion")
    FetchContent_Declare(Corrosion GIT_REPOSITORY https://github.com/corrosion-rs/corrosion.git GIT_TAG v0.5)
    FetchContent_MakeAvailable(Corrosion)
    corrosion_import_crate(MANIFEST_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Cargo.toml CRATES flatsim-ffi)
endif()

# --------------------------------------------------------------------------------------------------
# External Dependencies
set(ext_deps)

# Muli Physics Engine
set(MULI_BUILD_DEMO OFF)
FetchContent_Declare(muli GIT_REPOSITORY https://github.com/Sopiro/Muli GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(muli)
include_directories(${muli_SOURCE_DIR}/include)
list(APPEND ext_deps muli)

find_package(rerun_sdk REQUIRED)
find_package(TBB REQUIRED)
find_package(ZeroMQ REQUIRED)
find_package(cppzmq REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(Boost REQUIRED COMPONENTS json)

list(APPEND ext_deps rerun_sdk zmq cppzmq spdlog::spdlog Boost::json TBB::tbb)

FetchContent_Declare(concord GIT_REPOSITORY https://github.com/onlyhead/concord.git GIT_TAG 2.3.2)
FetchContent_MakeAvailable(concord)
list(APPEND ext_deps concord::concord)

FetchContent_Declare(pigment GIT_REPOSITORY https://github.com/onlyhead/pigment.git GIT_TAG 0.3.1)
FetchContent_MakeAvailable(pigment)
list(APPEND ext_deps pigment::pigment)

FetchContent_Declare(entropy GIT_REPOSITORY https://github.com/onlyhead/entropy.git GIT_TAG 1.0.0)
FetchContent_MakeAvailable(entropy)
list(APPEND ext_deps entropy::entropy)

FetchContent_Declare(zoneout GIT_REPOSITORY https://github.com/onlyhead/zoneout.git GIT_TAG 1.1.0)
FetchContent_MakeAvailable(zoneout)
list(APPEND ext_deps zoneout::zoneout)

FetchContent_Declare(farmtrax GIT_REPOSITORY https://github.com/onlyhead/farmtrax.git GIT_TAG 1.0.1)
FetchContent_MakeAvailable(farmtrax)
list(APPEND ext_deps farmtrax::farmtrax)

# FetchContent_Declare(navcon GIT_REPOSITORY https://github.com/onlyhead/navcon.git GIT_TAG 0.1.6)
# FetchContent_MakeAvailable(navcon)
add_subdirectory(xtra/navcon navcon)
list(APPEND ext_deps navcon::navcon)

# --------------------------------------------------------------------------------------------------
add_library(${project_name} INTERFACE)
# Allow users to link via `${project_name}::${project_name}`
add_library(${project_name}::${project_name} ALIAS ${project_name})
target_include_directories(${project_name} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include
)
include_directories(include)

# --------------------------------------------------------------------------------------------------
include(GNUInstallDirs)

#Install headers
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

#Install and export the INTERFACE target
install(TARGETS ${project_name} EXPORT ${project_name}Targets)

#Install the export file so that `find_package(${project_name})` works
install(EXPORT ${project_name}Targets
  FILE ${project_name}Targets.cmake
  NAMESPACE ${project_name}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${project_name}
)

#Also export it for in‐source FetchContent usage
export(EXPORT ${project_name}Targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${project_name}Targets.cmake"
  NAMESPACE ${project_name}::
)


# --------------------------------------------------------------------------------------------------
file(GLOB_RECURSE internal_deps
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

# Create a static library from internal sources to avoid recompiling for each executable
add_library(${project_name}_internal STATIC ${internal_deps})
target_compile_options(${project_name}_internal PRIVATE ${params})
target_link_libraries(${project_name}_internal PUBLIC ${ext_deps})
target_include_directories(${project_name}_internal PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link Rust FFI library if enabled
if(${project_name_upper}_ENABLE_RUST)
    target_link_libraries(${project_name}_internal PUBLIC flatsim_ffi)
    # Include CXX bridge headers
    target_include_directories(${project_name}_internal PUBLIC 
        ${CMAKE_BINARY_DIR}/cargo/build/${Rust_CARGO_TARGET}/cxxbridge
    )
endif()

# --------------------------------------------------------------------------------------------------
if(${project_name_upper}_BUILD_EXAMPLES)
  file(GLOB exe CONFIGURE_DEPENDS examples/*.cpp)
  # ----------------------------------------------
  FetchContent_Declare(cli11 GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git GIT_TAG v2.5.0)
  FetchContent_MakeAvailable(cli11)
  list(APPEND ext_deps CLI11::CLI11)

  set(exec_names)
  foreach(src_file IN LISTS exe)
    get_filename_component(exec_name "${src_file}" NAME_WE)
    add_executable(${exec_name} "${src_file}")
    target_compile_options(${exec_name} PRIVATE ${params})
    target_compile_options(${exec_name} PRIVATE ${CURL_CFLAGS_OTHER})
    # Link against the static library instead of compiling all sources again
    target_link_libraries(${exec_name} ${project_name}_internal ${ext_deps})
    target_link_options(${exec_name} PRIVATE ${link_options})
    target_include_directories(${exec_name} PRIVATE ${CURL_INCLUDE_DIRS})
    install(TARGETS ${exec_name} DESTINATION bin)
    list(APPEND exec_names ${exec_name})
  endforeach()
  # ----------------------------------------------
  foreach(exec IN LISTS exec_names)
    file(REMOVE "${CMAKE_CURRENT_LIST_DIR}/.execs")
    file(WRITE "${CMAKE_CURRENT_LIST_DIR}/.execs")
    get_filename_component(exec_name "${exec}" NAME)
    file(APPEND "${CMAKE_CURRENT_LIST_DIR}/.execs" "${CMAKE_CURRENT_LIST_DIR}/build/${exec_name}\n")
  endforeach()
  # ----------------------------------------------
  install(TARGETS
    ${exec_names}
    DESTINATION lib/${PROJECT_NAME}
  )
endif()


# --------------------------------------------------------------------------------------------------
if(${project_name_upper}_ENABLE_TESTS)
  enable_testing()
  add_definitions(-DENABLE_DOCTEST_IN_LIBRARY)
  include(FetchContent)
  FetchContent_Declare(DocTest GIT_REPOSITORY "https://github.com/onqtam/doctest")
  FetchContent_MakeAvailable(DocTest)

  file(GLOB test_src CONFIGURE_DEPENDS test/*.cpp)

  foreach(src_file IN LISTS test_src)
    get_filename_component(test_name "${src_file}" NAME_WE)
    add_executable(${test_name} "${src_file}")
    target_compile_options(${test_name} PRIVATE ${params})
    # Link against the static library instead of compiling all sources again
    target_link_libraries(${test_name} ${project_name}_internal ${ext_deps} doctest_with_main)
    target_link_options(${test_name} PRIVATE ${link_options})
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()

