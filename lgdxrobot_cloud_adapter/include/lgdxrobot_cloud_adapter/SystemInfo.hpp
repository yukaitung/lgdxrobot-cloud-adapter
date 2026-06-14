#ifndef SYSTEM_INFO_HPP
#define SYSTEM_INFO_HPP

#include <string>
#include <unordered_map>
#include <utility>

#include "rclcpp/rclcpp.hpp"

class SystemInfo
{
  private:
    rclcpp::Logger logger_;

    int cacheSwapTotal = 0;
    int cacheSwapUsed = 0;

    std::string ReadFile(const char *filename);
    std::string RunCommand(const char *command);

  public:
    SystemInfo(rclcpp::Node::SharedPtr node);

    // System Info
    std::unordered_map<std::string, std::string> GetCpu();
    std::string GetMotherboardName();
    std::string GetMotherboardSerialNumber();
    int GetMemory();
    std::string GetGpu();
    std::string GetOs();

    // Monitoring
    float GetCpuUsage();
    std::pair<int, int> GetMemoryUsage();
    std::pair<int, int> GetSwapUsage();
    std::pair<int, int> GetDiskUsage();
};

#endif // SYSTEM_INFO_HPP