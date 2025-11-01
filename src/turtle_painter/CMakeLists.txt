cmake_minimum_required(VERSION 3.8)
project(turtle_painter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(turtle_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(paint src/paint.cpp)
add_executable(go_to_goal src/turtle_go_to_goal_action_server.cpp)

ament_target_dependencies(paint rclcpp turtlesim geometry_msgs rclcpp_action turtle_interfaces std_srvs)
ament_target_dependencies(go_to_goal rclcpp rclcpp_action geometry_msgs turtlesim turtle_interfaces)

install(TARGETS
	paint
	go_to_goal
	DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
