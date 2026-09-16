if(NOT ROBOT_ENABLE_MOCK)
  message(FATAL_ERROR "ROBOT_BUILD_EXAMPLES currently requires ROBOT_ENABLE_MOCK=ON")
endif()

add_executable(robot_basic_move basic_move.cc)
target_link_libraries(robot_basic_move PRIVATE robot_core robot_mock)
target_include_directories(robot_basic_move PRIVATE
  "${PROJECT_SOURCE_DIR}/src/adapters/mock")
robot_enable_warnings(robot_basic_move)
