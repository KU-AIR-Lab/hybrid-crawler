cmake_minimum_required(VERSION 3.10)
project(crawler_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointSetpoint.msg"
  "action/Locomotion.action"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

# --- ADD THESE LINES TO OMIT THE WARNING ---
# These variables are only used by the legacy 'catkin' build system and 
# cause warnings in ROS 2's 'ament_cmake' when passed implicitly by colcon.
unset(CATKIN_INSTALL_INTO_PREFIX_ROOT)
unset(CATKIN_SYMLINK_INSTALL)
# ---------------------------------------------

ament_package()
