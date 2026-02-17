#include "lgdxrobot_cloud_adapter/CloudAdapter.hpp"

CloudAdapter::CloudAdapter() : Node("lgdxrobot_cloud_adapter_node")
{}

void CloudAdapter::Initalise()
{
  RCLCPP_INFO(this->get_logger(), "Initialising Cloud Adapter");
}