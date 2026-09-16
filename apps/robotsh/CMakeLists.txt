# robotsh composes the framework directly (driver + CommandExecutor) and is
# fully independent of the robot-manager daemon, its gRPC transport and api/.
add_executable(robotsh main.cc)
target_link_libraries(robotsh PRIVATE robot_core robot_config)
target_include_directories(robotsh PRIVATE "${ROBOT_GENERATED_INCLUDE_DIR}")

if(ROBOT_ENABLE_MOCK)
  target_link_libraries(robotsh PRIVATE robot_mock)
  target_include_directories(robotsh PRIVATE
    "${PROJECT_SOURCE_DIR}/src/adapters/mock")
endif()
if(ROBOT_ENABLE_EFORT)
  target_link_libraries(robotsh PRIVATE robot_efort)
  target_include_directories(robotsh PRIVATE
    "${PROJECT_SOURCE_DIR}/src/adapters/efort")
endif()
robot_enable_warnings(robotsh)
set_target_properties(robotsh PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
install(TARGETS robotsh RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
