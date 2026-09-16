if(NOT ROBOT_ENABLE_MOCK)
  message(FATAL_ERROR "ROBOT_BUILD_TESTS requires ROBOT_ENABLE_MOCK=ON")
endif()

add_executable(robot_unit_tests unit/robot_unit_tests.cc)
target_link_libraries(robot_unit_tests PRIVATE robot_core robot_config robot_mock)
target_include_directories(robot_unit_tests PRIVATE
  "${PROJECT_SOURCE_DIR}/src/adapters/mock")
robot_enable_warnings(robot_unit_tests)
add_test(NAME robot_unit_tests COMMAND robot_unit_tests)
set_tests_properties(robot_unit_tests PROPERTIES
  WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}")
