#ifndef SAVE_MAP_HPP
#define SAVE_MAP_HPP

#include <string>
#include "rclcpp/rclcpp.hpp"

struct MapParameters
{
  int mapWidth;
  int mapHeight;
  double resolution;
  double originX;
  double originY;
  double originRotation;
};

class SaveMap
{
  private:
    static void SaveYaml(const std::string &fileName, const MapParameters &parameters, const bool isScale, rclcpp::Logger logger);
    static void SavePgm(const std::string &fileName, const int width, const int height, 
      const std::string &mapStringBytes, rclcpp::Logger logger);
  public:
    static void SaveRoute(const std::string &route, rclcpp::Logger logger);
    static void SaveMapData(const MapParameters &parameters, const std::string& mapStringBytes, rclcpp::Logger logger);
    static void SaveKeepoutMask(const MapParameters &parameters, const std::string& mapStringBytes, rclcpp::Logger logger);
    static void SaveSpeedMask(const MapParameters &parameters, const std::string& mapStringBytes, rclcpp::Logger logger);
    SaveMap() = delete;
};

#endif // SAVE_MAP_HPP