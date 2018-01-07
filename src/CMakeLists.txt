set(main main.cpp)
set(sources
  application.h
  measurement.h
  protocol.h
  protocol.cpp
  controller.h
  controller.cpp
  twiddle.h
  configuration.h
  configuration.cpp
  processor.h
  filter.h
  filter.cpp
  )

add_executable(${CMAKE_PROJECT_NAME} ${sources} ${main})
target_link_libraries(${CMAKE_PROJECT_NAME}
  ${ZLIB_LIBRARIES}
  ${OPENSSL_LIBRARIES}
  ${UV_LIB}
  ${UWS_LIB}
  ${CMAKE_THREAD_LIBS_INIT}
  )
set_target_properties(
  ${CMAKE_PROJECT_NAME}
  PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
  )
