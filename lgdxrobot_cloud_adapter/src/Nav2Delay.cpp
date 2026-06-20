#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("nav2_delay");

  node->declare_parameter("nav2_delay_enable", false);
  bool delay = node->get_parameter("nav2_delay_enable").as_bool();
  if (delay)
  {
    RCLCPP_INFO(node->get_logger(), "Delay Nav2 until the cloud adapter is ready.");
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "Start Nav2 immediately.");
    rclcpp::shutdown();
    return 0;
  }

  auto server = node->create_service<std_srvs::srv::Empty>("cloud/nav2_delay",
    [node](const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
      RCLCPP_INFO(node->get_logger(), "Starting Nav2.");
      rclcpp::shutdown();
    });

  rclcpp::spin(node);
  return 0;
}
