cmake_minimum_required(VERSION 3.5)
project(slsim)

set(CMAKE_CXX_STANDARD 11)

# set CMAKE_BUILD_TYPE=Release for cmake-gui
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# An option supported by the GCC and Clang compilers \
# can instruct the compiler to optimize for the CPU architecture \
# of the host currently running CMake.
if(BUILD_WITH_MARCH_NATIVE)
  add_compile_options(-march=native)
  set(CMAKE_C_FLAGS "-march=native ${CMAKE_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "-march=native ${CMAKE_CXX_FLAGS}")
endif()

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# set output path
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib) # STATIC
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib) # SHARED
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(SOURCE_DIR ${PROJECT_SOURCE_DIR}/src)
file(GLOB SOURCE_FILES 
    "${SOURCE_DIR}/*.cpp"
    "${SOURCE_DIR}/*.hpp"
    "${SOURCE_DIR}/*.c"
    "${SOURCE_DIR}/*.h"
)

include_directories(${PROJECT_SOURCE_DIR} 
                    ${PROJECT_SOURCE_DIR}/src
                    ${EIGEN3_INCLUDE_DIR} 
                    ${OpenCV_INCLUDE_DIRS})

add_executable(data_gen src/main/data_gen.cpp ${SOURCE_FILES})
target_link_libraries(data_gen ${OpenCV_LIBS})
