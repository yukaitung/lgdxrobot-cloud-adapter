#include <boost/algorithm/string.hpp>
#include <vector>
#include <fstream>

#include "lgdxrobot_cloud_adapter/SystemInfo.hpp"

using namespace std;
using namespace boost;

SystemInfo::SystemInfo(rclcpp::Node::SharedPtr node) : logger_(node->get_logger())
{}

std::string SystemInfo::ReadFile(const char *filename)
{
  std::ifstream file(filename, std::ios::in);
  if (!file.is_open()) 
  {
    RCLCPP_ERROR(logger_, "Failed to open file: %s", filename);
    return "";
  }
  string content((istreambuf_iterator<char>(file)), istreambuf_iterator<char>());
  return content;
}

std::string SystemInfo::RunCommand(const char *command)
{
  const int BUFFER_SIZE = 128;
  char buffer[BUFFER_SIZE];
  string result;
  FILE* pipe = popen(command, "r");
  if (!pipe) 
  {
    RCLCPP_ERROR(logger_, "Failed to run command: %s", command);
    return "";
  }
  try 
  {
    while (fgets(buffer, BUFFER_SIZE, pipe) != nullptr) 
    {
      result += buffer;
    }
  } catch (...) 
  {
    pclose(pipe);
    RCLCPP_ERROR(logger_, "The command %s threw an exception.", command);
    return "";
  }
  pclose(pipe);
  return result;
}

std::unordered_map<std::string, std::string> SystemInfo::GetCpu()
{
  unordered_map<string, string> cpuInfo;

  string output = RunCommand("lscpu");
  
  // Process the output
  vector<string> lines;
  split(lines, output, is_any_of("\n"));
  for (auto line : lines)
  {
    vector<string> keyValue;
    split(keyValue, line, is_any_of(":"));
    if (keyValue.size() == 2)
    {
      trim(keyValue[1]);
      cpuInfo[keyValue[0]] = keyValue[1];
    }
  }

  return cpuInfo;
}

std::string SystemInfo::GetMotherboardName()
{
  string vendor = ReadFile("/sys/class/dmi/id/board_vendor");
  string model = ReadFile("/sys/class/dmi/id/board_name");
  if (!vendor.empty() && !model.empty())
  {
    // Vendor - Model
    return vendor + " - " + model;
  }
  else
  {
    // Vendor or Model
    return vendor + model;
  }
}

string SystemInfo::GetMotherboardSerialNumber()
{
  return ReadFile("/sys/class/dmi/id/board_serial");
}

int SystemInfo::GetMemory()
{
  int memoryTotal;

  string output = RunCommand("free --mega | tr -s ' '");
  
  // Process the output
  vector<string> lines;
  split(lines, output, is_any_of("\n"));
  for (auto line : lines)
  {
    vector<string> values;
    split(values, line, is_any_of(" "));
    
    if (values.size() > 0)
    {
      if (values[0] == "Mem:")
      {
        memoryTotal = stoi(values[1]);
        break;
      }
    }
  }

  return memoryTotal;
}

std::string SystemInfo::GetGpu()
{
  string output = RunCommand("lspci | grep ' VGA '");
  if (output.empty()) 
  {
    return "";
  }

  const string toFind = "VGA compatible controller: ";
  size_t start = output.find(toFind);
  if (start == string::npos)
  {
    return "";
  }
  size_t end = output.find(" (", start);
  if (end == string::npos)
  {
    end = output.length();
  }
  string name = output.substr(start + toFind.length(), end - start - toFind.length());
  return name;
}

std::string SystemInfo::GetOs()
{
  string output = ReadFile("/etc/os-release");
  if (output.empty())
  {
    return "";
  }

  const string toFind = "PRETTY_NAME=\"";
  size_t start = output.find(toFind);
  if (start == string::npos)
  {
    return "";
  }
  size_t end = output.find("\n", start);
  if (end == string::npos)
  {
    end = output.length();
  }
  string name = output.substr(start + toFind.length(), end - start - toFind.length() - 1);
  return name;
}

float SystemInfo::GetCpuUsage()
{
  float cpuUsage = 0.0;
  string output = RunCommand("mpstat | awk '/all/ {print $NF}'");
  cpuUsage = stof(output);
  return cpuUsage;
}

std::pair<int, int> SystemInfo::GetMemoryUsage()
{
  int memoryTotal, memoryUsed;

  string output = RunCommand("free --mega | tr -s ' '");
  
  // Process the output
  vector<string> lines;
  split(lines, output, is_any_of("\n"));
  for (auto line : lines)
  {
    vector<string> values;
    split(values, line, is_any_of(" "));
    
    if (values.size() > 0)
    {
      if (values[0] == "Mem:")
      {
        memoryTotal = stoi(values[1]);
        memoryUsed = stoi(values[2]);
      }
      else if (values[0] == "Swap:")
      {
        cacheSwapTotal = stoi(values[1]);
        cacheSwapUsed = stoi(values[2]);
      }
    }
  }

  return {memoryTotal, memoryUsed};
}

std::pair<int, int> SystemInfo::GetSwapUsage()
{
  return {cacheSwapTotal, cacheSwapUsed};
}

std::pair<int, int> SystemInfo::GetDiskUsage()
{
  int diskTotal, diskUsed;
  string output = RunCommand("df -m | grep '^/dev/' | tr -s ' '");

  // Process the output
  vector<string> lines;
  split(lines, output, is_any_of("\n"));
  for (auto line : lines)
  {
    vector<string> values;
    split(values, line, is_any_of(" "));
    
    if (values.size() >= 3)
    {
      diskTotal += stoi(values[1]);
      diskUsed += stoi(values[2]);
    }
  }

  return {diskTotal, diskUsed};
}