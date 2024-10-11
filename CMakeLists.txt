cmake_minimum_required(VERSION 3.8)
project(agri_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


# Find the controlcan library
set(LIBCONTROL_PATH "${PROJECT_SOURCE_DIR}/hardware/include")  # Adjust this path if necessary
find_library(CONTROL_CAN_LIB NAMES controlcan PATHS ${LIBCONTROL_PATH} NO_DEFAULT_PATH)

add_library(
  diffdrive_agribot
  SHARED
  hardware/diffbot_system.cpp
  hardware/canalystii.cpp
)

# Link the controlcan library
target_link_libraries(diffdrive_agribot PUBLIC ${CONTROL_CAN_LIB})

target_compile_features(diffdrive_agribot PUBLIC cxx_std_17)
target_include_directories(diffdrive_agribot PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/diffdrive_agribot>
)
ament_target_dependencies(
  diffdrive_agribot PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(diffdrive_agribot PRIVATE "DIFFDRIVE_AGRIBOT_BUILDING_DLL")

# # Custom command to create symbolic link for Gazebo meshes
# set(INPUT_MESHES_DIR ${CMAKE_CURRENT_SOURCE_DIR})
# set(OUTPUT_MESHES_SYMLINK $ENV{HOME}/.gazebo/models/${PROJECT_NAME})
# execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${INPUT_MESHES_DIR} ${OUTPUT_MESHES_SYMLINK})

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface diffdrive_agribot.xml)


# INSTALL
install(
  DIRECTORY src launch rviz worlds maps config
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY hardware/include/
  DESTINATION include/diffdrive_agribot
)
install(TARGETS diffdrive_agribot
  EXPORT export_diffdrive_agribot
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install python modules
ament_python_install_package(${PROJECT_NAME})

# Install python executables
install(PROGRAMS
    python_nodes/gps_waypoint_logger.py
    python_nodes/interactive_waypoint_follower.py
    python_nodes/logged_waypoint_follower.py
    python_nodes/joy_gps_adapter.py
    DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
