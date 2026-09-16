add_executable(robotctl main.cc)
target_link_libraries(robotctl PRIVATE robot_core)
target_include_directories(robotctl PRIVATE "${ROBOT_GENERATED_INCLUDE_DIR}")
if(ROBOT_ENABLE_GRPC)
  target_link_libraries(robotctl PRIVATE robot_proto)
  target_compile_definitions(robotctl PRIVATE ROBOT_HAS_GRPC=1)
endif()
robot_enable_warnings(robotctl)

set_target_properties(robotctl PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
install(TARGETS robotctl RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
