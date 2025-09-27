cmake_minimum_required(VERSION 3.8)
project(robot_actions_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rcl_action REQUIRED)

# Generate action interfaces
find_package(rosidl_default_generators REQUIRED)

set(action_files
  "action/Ask.action"
  "action/GetAllRooms.action"
  "action/GetCurrentLocation.action"
  "action/GetReachableLocationsAroundObject.action"
  "action/GoTo.action"
  "action/IsInRoom.action"
  "action/Pick.action"
  "action/PickUp.action"
  "action/Place.action"
  "action/PutDown.action"
  "action/PutIntoBasket.action"
  "action/RetrieveFromBasket.action"
  "action/Say.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  DEPENDENCIES std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()