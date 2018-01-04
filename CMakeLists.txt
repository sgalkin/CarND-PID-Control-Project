project(pid)
cmake_minimum_required (VERSION 3.5)

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.9)
    message(FATAL_ERROR "GCC version must be at least 4.9!")
  endif()
endif()

add_definitions(-std=c++11)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -Werror -O3 -g")

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  execute_process(COMMAND brew --prefix openssl
    OUTPUT_VARIABLE OPENSSL_ROOT_DIR)
  string(STRIP ${OPENSSL_ROOT_DIR} OPENSSL_ROOT_DIR)
endif()

find_package(ZLIB REQUIRED)

find_package(OpenSSL REQUIRED)
include_directories(${OPENSSL_INCLUDE_DIR})

find_library(UWS_LIB uWS)
if(NOT UWS_LIB)
  message(FATAL_ERROR "uWS library not found")
endif()

find_library(UV_LIB uv)
if(NOT UV_LIB)
  message(FATAL_ERROR "uv library not found")
endif()

add_subdirectory(src)
add_subdirectory(test)
