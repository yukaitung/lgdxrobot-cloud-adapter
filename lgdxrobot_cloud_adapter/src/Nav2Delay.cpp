#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr g_node = nullptr;
  g_node = rclcpp::Node::make_shared("nav2_delay");

  g_node->declare_parameter("nav2_delay_enable", false);
  bool delay = g_node->get_parameter("nav2_delay_enable").as_bool();
  if (delay)
  {
    RCLCPP_INFO(g_node->get_logger(), "Delay Nav2 until the cloud adapter is ready.");
  }
  else
  {
    RCLCPP_INFO(g_node->get_logger(), "Do not delay Nav2.");
    rclcpp::shutdown();
    g_node = nullptr;
    return 0;
  }

  auto server = g_node->create_service<std_srvs::srv::Empty>("cloud/nav2_delay",
    [g_node](const std::shared_ptr<std_srvs::srv::Empty::Request>,
      std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
      RCLCPP_INFO(g_node->get_logger(), "Start Nav2");
      rclcpp::shutdown();
    });

  rclcpp::spin(g_node);
  rclcpp::shutdown();
  g_node = nullptr;
  return 0;
}
