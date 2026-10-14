cmake_minimum_required(VERSION 3.8)
project(castor_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(world_frame src/world_frame_node.cpp)
target_include_directories(world_frame PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(world_frame PRIVATE rclcpp::rclcpp ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS})

install(TARGETS world_frame DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_datum test/test_datum.cpp)
  target_include_directories(test_datum PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

ament_package()
