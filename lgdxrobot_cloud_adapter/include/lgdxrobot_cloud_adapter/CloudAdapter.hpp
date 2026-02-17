#ifndef CLOUD_ADAPTER_HPP
#define CLOUD_ADAPTER_HPP

#include "rclcpp/rclcpp.hpp"

class CloudAdapter : public rclcpp::Node
{
  public:
    CloudAdapter();

    void Initalise();
};

#endif // CLOUD_ADAPTER_HPP