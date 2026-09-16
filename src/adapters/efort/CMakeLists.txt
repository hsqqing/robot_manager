find_package(EfortSdk REQUIRED)
add_library(
  robot_efort
  efort_error_mapper.cc
  efort_driver.cc)
target_include_directories(
  robot_efort
  PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(robot_efort PUBLIC robot_core PRIVATE EfortSdk::EfortSdk)
target_compile_definitions(robot_efort PUBLIC ROBOT_HAS_EFORT=1)
robot_enable_warnings(robot_efort)
