#include <fstream>

#include "lgdxrobot_cloud_adapter/SaveMap.hpp"

void SaveMap::SaveRoute(const std::string& route, rclcpp::Logger logger)
{
  std::ofstream file("route.geojson", std::ios::out | std::ios::trunc);
  if (!file.is_open())
  {
    RCLCPP_ERROR(logger, "Unable to write route.geojson");
    return;
  }
  file << route;
  file.close();
}
