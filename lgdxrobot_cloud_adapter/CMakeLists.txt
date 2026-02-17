cmake_minimum_required(VERSION 3.8)
project(lgdxrobot_cloud_adapter)

set(CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_NAME lgdxrobot_cloud_adapter)
set(PROTO_NAME lgdxrobot2_agent_proto)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#
# gRPC
#
find_package(Protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)

add_library(${PROTO_NAME} 
  proto/RobotClientsService.proto
)
target_link_libraries(${PROTO_NAME} 
  PUBLIC
  protobuf::libprotobuf
  gRPC::grpc
  gRPC::grpc++
)
target_include_directories(${PROTO_NAME} PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

get_target_property(grpc_cpp_plugin_location gRPC::grpc_cpp_plugin LOCATION)
protobuf_generate(TARGET ${PROTO_NAME} LANGUAGE cpp)
protobuf_generate(TARGET ${PROTO_NAME} LANGUAGE grpc GENERATE_EXTENSIONS .grpc.pb.h .grpc.pb.cc PLUGIN "protoc-gen-grpc=${grpc_cpp_plugin_location}")

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
target_link_libraries(${EXECUTABLE_NAME} 
  ${PROTO_NAME}
  "${cpp_typesupport_target}"
)
ament_target_dependencies(${EXECUTABLE_NAME}  
  rclcpp
)

install(TARGETS ${EXECUTABLE_NAME} DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROTO_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

ament_package()
