cmake_minimum_required(VERSION 3.5)
project(object_detection)

# Default standards
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------
# Dependencies
# -----------------------------
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# -----------------------------
# Generate Messages
# -----------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/InferenceResult.msg"
  "msg/Yolov8Inference.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# -----------------------------
# Install Python Nodes
# -----------------------------
install(
  PROGRAMS
    scripts/trolley_image_detector.py
    scripts/yolov8_ros2_pt.py
    scripts/yolov8_ros2_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------
# Install Other Resources
# -----------------------------
install(
  DIRECTORY
    launch
    worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
