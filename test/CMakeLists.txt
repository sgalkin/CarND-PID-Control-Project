set(TEST_NAME "Test")
set(test main.cpp)

set(sources
  ${CMAKE_SOURCE_DIR}/src/twiddle.h
  ${CMAKE_SOURCE_DIR}/src/controller.h
  ${CMAKE_SOURCE_DIR}/src/controller.cpp
  )

set(tests
  test_twiddle.cpp
  test_controller.cpp
  )

include_directories(${CMAKE_SOURCE_DIR}/src)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-for-loop-analysis")
add_executable(${TEST_NAME} ${tests} ${sources} ${test})
set_target_properties(
  ${CMAKE_PROJECT_NAME}
  PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

add_custom_target(check
  DEPENDS ${TEST_NAME}
  COMMAND ./${TEST_NAME} --use-colour no
  )

enable_testing()
add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
