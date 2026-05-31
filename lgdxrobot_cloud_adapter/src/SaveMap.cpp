#include <fstream>

#include "lgdxrobot_cloud_adapter/SaveMap.hpp"

void SaveMap::SaveYaml(const std::string &fileName, const MapParameters &parameters, const bool isScale, rclcpp::Logger logger)
{
  std::ofstream file(fileName + ".yaml", std::ios::out | std::ios::trunc);
  if (!file.is_open())
  {
    RCLCPP_ERROR(logger, "Unable to write %s", fileName.c_str());
    return;
  }
  file << "image: " << fileName << ".pgm" << std::endl;
  if (isScale)
  {
    file << "mode: scale" << std::endl;
  }
  else
  {
    file << "mode: trinary" << std::endl;
  }
  file << "resolution: " << parameters.resolution << std::endl;
  file << "origin: [" << parameters.originX << ", " << parameters.originY << ", " << parameters.originRotation << "]" << std::endl;
  file << "negate: 0" << std::endl;
  if (isScale)
  {
    file << "occupied_thresh: 1.0" << std::endl;
    file << "free_thresh: 0.0" << std::endl;
  }
  else
  {
    file << "occupied_thresh: 0.65" << std::endl;
    file << "free_thresh: 0.25" << std::endl;
  }
  file.close();
}

void SaveMap::SavePgm(const std::string &fileName, const int width, const int height, 
  const std::string &mapStringBytes, rclcpp::Logger logger)
{
  std::ofstream file(fileName + ".pgm", std::ios::out | std::ios::trunc | std::ios::binary);
  if (!file.is_open())
  {
    RCLCPP_ERROR(logger, "Unable to write %s.pgm", fileName.c_str());
    return;
  }
  file << "P5" << std::endl;
  file << width << " " << height << std::endl;
  file << "255" << std::endl;
  if (mapStringBytes.empty())
  {
    // Make a empty map
    for (int i = 0; i < width * height; i++)
    {
      char byte = 255;
      file << byte;
    }
  }
  else
  {
    file.write(mapStringBytes.c_str(), mapStringBytes.size());
  }
  file.close();
}

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

void SaveMap::SaveMapData(const MapParameters &parameters, const std::string& mapStringBytes, rclcpp::Logger logger)
{
  SaveYaml("map", parameters, false, logger);
  SavePgm("map", parameters.mapWidth, parameters.mapHeight, mapStringBytes, logger);
}

void SaveMap::SaveKeepoutMask(const MapParameters &parameters, const std::string& mapStringBytes, rclcpp::Logger logger)
{
  
  SaveYaml("keepout", parameters, false, logger);
  SavePgm("keepout", parameters.mapWidth, parameters.mapHeight, mapStringBytes, logger);
}

void SaveMap::SaveSpeedMask(const MapParameters &parameters, const std::string& mapStringBytes, rclcpp::Logger logger)
{
  SaveYaml("speed", parameters, true, logger);
  SavePgm("speed", parameters.mapWidth, parameters.mapHeight, mapStringBytes, logger);
}
