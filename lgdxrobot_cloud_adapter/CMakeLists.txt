cmake_minimum_required(VERSION 3.8)
project(lgdxrobot_cloud_adapter)

set(CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_NAME lgdxrobot_cloud_adapter)
set(PROTO_NAME lgdxrobot_cloud_adapter_proto)

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
target_include_directories(${PROTO_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<INSTALL_INTERFACE:include>
)

get_target_property(grpc_cpp_plugin_location gRPC::grpc_cpp_plugin LOCATION)
protobuf_generate(TARGET ${PROTO_NAME} LANGUAGE cpp)
protobuf_generate(TARGET ${PROTO_NAME} LANGUAGE grpc GENERATE_EXTENSIONS .grpc.pb.h .grpc.pb.cc PLUGIN "protoc-gen-grpc=${grpc_cpp_plugin_location}")

#
# Node
#
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(include)

add_library(lgdxrobot_cloud_component SHARED
  src/CloudAdapter.cpp
  src/RobotStatus.cpp
)
target_link_libraries(lgdxrobot_cloud_component
  ${PROTO_NAME}
  "${cpp_typesupport_target}"
)
ament_target_dependencies(lgdxrobot_cloud_component
  rclcpp
  rclcpp_components
)

rclcpp_components_register_node(lgdxrobot_cloud_component
  PLUGIN "LGDXRobotCloud::CloudAdapter"
  EXECUTABLE lgdxrobot_cloud_adapter_node
)

ament_export_targets(export_lgdxrobot_cloud_proto)
install(TARGETS ${PROTO_NAME} 
  EXPORT export_lgdxrobot_cloud_proto
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
ament_export_targets(export_lgdxrobot_cloud_component)
install(TARGETS lgdxrobot_cloud_component
  EXPORT export_lgdxrobot_cloud_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
