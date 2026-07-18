cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

find_package(ament_cmake REQUIRED)
#find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


#add_executable(serial_sender_node src/serial_sender.cpp)
#ament_target_dependencies(serial_sender_node rclcpp std_msgs)

#add_executable(ik_node src/ik_node.cpp)
#ament_target_dependencies(ik_node rclcpp std_msgs geometry_msgs)

#install(TARGETS ik_node serial_sender_node
# DESTINATION lib/${PROJECT_NAME}
#)

install(PROGRAMS
  src/serial_sender.py
  src/ik_node.py
  src/camera_node.py
  src/color_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
