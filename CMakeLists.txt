cmake_minimum_required(VERSION 3.8)
project(sort_mpc_nav)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

add_executable(pedestrian_sim src/pedestrian_sim.cpp)
ament_target_dependencies(pedestrian_sim 
  rclcpp
  geometry_msgs
  visualization_msgs
)

install(TARGETS
  pedestrian_sim
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  sort_mpc_nav/sort_tracker.py
  sort_mpc_nav/mpc_planner.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()