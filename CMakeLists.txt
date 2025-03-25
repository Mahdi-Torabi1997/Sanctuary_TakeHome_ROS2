cmake_minimum_required(VERSION 3.8)
project(cpp_pkg)

# Required compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Publisher node: uses FK and the manipulator code.
add_executable(publisher_node 
  src/publisher_node.cpp 
  src/ForwardKinematics.cpp 
  src/RRRManipulator.cpp
)
ament_target_dependencies(publisher_node rclcpp example_interfaces Eigen3)

add_executable(subscriber_node 
  src/subscriber_node.cpp 
  src/InverseKinematics.cpp 
  src/RRRManipulator.cpp
)
ament_target_dependencies(subscriber_node rclcpp example_interfaces Eigen3)



install(TARGETS
  publisher_node
  subscriber_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
