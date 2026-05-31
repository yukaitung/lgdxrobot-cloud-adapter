#ifndef SAVE_MAP_HPP
#define SAVE_MAP_HPP

#include "rclcpp/rclcpp.hpp"

class SaveMap
{
  public:
    static void SaveRoute(const std::string& route, rclcpp::Logger logger);
    static void SaveMapData(const std::string& mapStringBytes);
    static void SaveKeepoutMask(const std::string& keepoutMaskStringBytes);
    static void SaveSpeedMask(const std::string& speedMaskStringBytes);
    SaveMap() = delete;
};

#endif // SAVE_MAP_HPP