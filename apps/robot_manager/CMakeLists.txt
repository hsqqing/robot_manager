add_subdirectory(robotctl)

if(ROBOT_ENABLE_GRPC)
  add_subdirectory(transport)
endif()

add_executable(robot-manager main.cc)
target_link_libraries(robot-manager PRIVATE robot_core robot_config)
target_include_directories(robot-manager PRIVATE
  "${ROBOT_GENERATED_INCLUDE_DIR}")

if(ROBOT_ENABLE_MOCK)
  target_link_libraries(robot-manager PRIVATE robot_mock)
  target_include_directories(robot-manager PRIVATE
    "${PROJECT_SOURCE_DIR}/src/adapters/mock")
endif()
if(ROBOT_ENABLE_EFORT)
  target_link_libraries(robot-manager PRIVATE robot_efort)
  target_include_directories(robot-manager PRIVATE
    "${PROJECT_SOURCE_DIR}/src/adapters/efort")
  set_target_properties(
    robot-manager
    PROPERTIES
      BUILD_RPATH "${EFORT_SDK_ROOT}/lib"
      INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}/robot-manager")
endif()
if(ROBOT_ENABLE_GRPC)
  target_link_libraries(robot-manager PRIVATE robot_grpc)
  target_compile_definitions(robot-manager PRIVATE ROBOT_HAS_GRPC=1)
endif()
robot_enable_warnings(robot-manager)
set_target_properties(robot-manager PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")

install(TARGETS robot-manager RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
