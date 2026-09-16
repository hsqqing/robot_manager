# Each executable owns its composition and optional transport dependencies.
add_subdirectory(robot_manager)
add_subdirectory(robotctl)

install(FILES "${ROBOT_GENERATED_INCLUDE_DIR}/robot/version.h"
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/robot)
install(DIRECTORY "${PROJECT_SOURCE_DIR}/config/"
  DESTINATION ${CMAKE_INSTALL_SYSCONFDIR}/robot-manager)
install(FILES "${PROJECT_SOURCE_DIR}/deploy/systemd/robot-manager.service"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/systemd/system)

if(ROBOT_ENABLE_EFORT)
  install(
    FILES
      "${EFORT_SDK_ROOT}/lib/libEftSdk.so"
      "${EFORT_SDK_ROOT}/lib/liblog4cpp.so"
      "${EFORT_SDK_ROOT}/lib/librlibcpp.bcc.so.1"
      "${EFORT_SDK_ROOT}/lib/librlibcpp.tool.so.1"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/robot-manager)
endif()
