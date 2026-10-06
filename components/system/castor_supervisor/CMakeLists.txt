cmake_minimum_required(VERSION 3.8)
project(castor_supervisor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yasmin REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(castor_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(supervisor src/supervisor_node.cpp)
target_include_directories(supervisor PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(supervisor rclcpp yasmin behaviortree_cpp castor_interfaces)

# Temporary mission state machine: take-off, MARL to a goal, hold, land.
add_executable(mission src/mission_node.cpp)
target_include_directories(mission PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(mission rclcpp castor_interfaces geometry_msgs nav_msgs)

install(TARGETS supervisor mission DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY trees DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rules test/test_rules.cpp)
  target_include_directories(test_rules PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_mission test/test_mission.cpp)
  target_include_directories(test_mission PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

ament_package()
