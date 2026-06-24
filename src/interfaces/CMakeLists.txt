cmake_minimum_required(VERSION 3.8)
project(interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TubeHeight.msg"
  "msg/TubeState.msg"
  "msg/RobotStatus.msg"
  "srv/StartTask.srv"
  "srv/StopTask.srv"
  "srv/RequestRecheck.srv"
  "srv/MoveToPose.srv"
  "srv/GripperControl.srv"
  "srv/CurrentTubeState.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
