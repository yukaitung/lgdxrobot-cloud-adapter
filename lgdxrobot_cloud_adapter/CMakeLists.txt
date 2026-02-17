cmake_minimum_required(VERSION 3.8)
project(lgdxrobot_cloud_adapter)

set(CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_NAME lgdxrobot_cloud_adapter)
set(PROTO_NAME lgdxrobot2_agent_proto)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#
# Node
#
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

add_executable(${EXECUTABLE_NAME} 
  src/main.cpp
  src/CloudAdapter.cpp
)
ament_target_dependencies(${EXECUTABLE_NAME}  
  rclcpp
)

install(TARGETS ${EXECUTABLE_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()
