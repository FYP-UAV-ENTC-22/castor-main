cmake_minimum_required(VERSION 3.8)
project(castor_vehicle_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(castor_interfaces REQUIRED)

add_executable(vehicle_interface src/vehicle_interface_node.cpp)
target_include_directories(vehicle_interface PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(vehicle_interface PRIVATE
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  ${px4_msgs_TARGETS}
  ${castor_interfaces_TARGETS})

install(TARGETS vehicle_interface DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frames test/test_frames.cpp)
  target_include_directories(test_frames PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

ament_package()
