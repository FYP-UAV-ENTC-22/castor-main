cmake_minimum_required(VERSION 3.8)
project(castor_policy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(castor_interfaces REQUIRED)

# ONNX Runtime is optional so the package still builds on a machine without it.
# The planning image installs the prebuilt release under /opt/castor/third_party.
set(ONNXRUNTIME_ROOT "/opt/castor/third_party/onnxruntime" CACHE PATH "ONNX Runtime prebuilt release")
find_path(ORT_INCLUDE_DIR onnxruntime_cxx_api.h PATHS "${ONNXRUNTIME_ROOT}/include" NO_DEFAULT_PATH)
find_library(ORT_LIBRARY onnxruntime PATHS "${ONNXRUNTIME_ROOT}/lib" NO_DEFAULT_PATH)

add_executable(policy_runner src/policy_runner_node.cpp)
target_include_directories(policy_runner PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(policy_runner PRIVATE rclcpp::rclcpp ${castor_interfaces_TARGETS})

if(ORT_INCLUDE_DIR AND ORT_LIBRARY)
  message(STATUS "castor_policy: ONNX Runtime from ${ONNXRUNTIME_ROOT}")
  target_compile_definitions(policy_runner PRIVATE CASTOR_HAVE_ORT=1)
  target_include_directories(policy_runner PRIVATE ${ORT_INCLUDE_DIR})
  target_link_libraries(policy_runner PRIVATE ${ORT_LIBRARY})
  get_filename_component(ORT_LIB_DIR "${ORT_LIBRARY}" DIRECTORY)
  set_target_properties(policy_runner PROPERTIES INSTALL_RPATH "${ORT_LIB_DIR}")
else()
  message(WARNING "castor_policy: ONNX Runtime not found under ${ONNXRUNTIME_ROOT}; building without model loading")
  target_compile_definitions(policy_runner PRIVATE CASTOR_HAVE_ORT=0)
endif()

install(TARGETS policy_runner DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_team test/test_team.cpp)
  target_include_directories(test_team PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

ament_package()
