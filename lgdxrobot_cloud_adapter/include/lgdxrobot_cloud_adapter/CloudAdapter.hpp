#ifndef CLOUD_ADAPTER_HPP
#define CLOUD_ADAPTER_HPP

#include <string>

#include "grpc/grpc.h"
#include "grpcpp/channel.h"
#include "grpcpp/client_context.h"
#include "grpcpp/create_channel.h"
#include "grpcpp/security/credentials.h"
#include "lgdxrobot_cloud_msgs/msg/auto_task.hpp"
#include "lgdxrobot_cloud_msgs/msg/robot_data.hpp"
#include "lgdxrobot_cloud_msgs/srv/auto_task_abort.hpp"
#include "lgdxrobot_cloud_msgs/srv/auto_task_next.hpp"
#include "lgdxrobot_cloud_msgs/srv/mcu_sn.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "proto/RobotClientsService.grpc.pb.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "std_msgs/msg/bool.hpp"

#include "RobotStatus.hpp"
#include "Map.hpp"
#include "Navigation.hpp"
#include "Exchange.hpp"

namespace LGDXRobotCloud
{

struct CloudErrorRetryData
{
  std::string mcuSerialNumber;
  RobotClientsNextToken nextToken;
  RobotClientsAbortToken abortToken;
};

class CloudAdapter : public rclcpp::Node
{
  private:
    const int kGrpcWaitSec = 5;

    // Modules
    std::unique_ptr<Map> map;
    std::unique_ptr<Navigation> navigation;
    std::unique_ptr<IExchange> exchangeStream;

    std::shared_ptr<CloudSignals> cloudSignals;
    std::shared_ptr<NavigationSignals> navigationSignals;

    // ROS
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr cloudRetryTimer;
    rclcpp::TimerBase::SharedPtr autoTaskPublisherTimer;
    rclcpp::TimerBase::SharedPtr cloudExchangeTimer;
    rclcpp::TimerBase::SharedPtr softwareEmergencyStopPublisherTimer;
    rclcpp::Publisher<lgdxrobot_cloud_msgs::msg::AutoTask>::SharedPtr autoTaskPublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr softwareEmergencyStopPublisher;
    rclcpp::Service<lgdxrobot_cloud_msgs::srv::McuSn>::SharedPtr mcuSerialNumberService;
    rclcpp::Service<lgdxrobot_cloud_msgs::srv::AutoTaskNext>::SharedPtr autoTaskNextService;
    rclcpp::Service<lgdxrobot_cloud_msgs::srv::AutoTaskAbort>::SharedPtr autoTaskAbortService;
    rclcpp::Subscription<lgdxrobot_cloud_msgs::msg::RobotData>::SharedPtr robotDataSubscription;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSubscription;
    std::shared_ptr<tf2_ros::TransformListener> tfListener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tfBuffer;

    // Robot Data
    RobotStatus::StateMachine robotStatus = RobotStatus::Offline();
    bool hasMcuSn = false;
    bool isSlam = false;
    bool pauseTaskAssignment = false;
    lgdxrobot_cloud_msgs::msg::RobotData robotData;
    std::shared_ptr<RobotClientsAutoTaskNavProgress> navProgress;
    std::vector<double> batteries = {0.0, 0.0};
    RobotClientsRobotCriticalStatus criticalStatus;
    // Robot Data: SLAM
    bool mapHasUpdated = false;
    bool overwriteGoal = false;
    RobotClientsMapData mapData;

    // Exchange
    RobotClientsData exchangeRobotData;
    RobotClientsNextToken exchangeNextToken;
    RobotClientsAbortToken exchangeAbortToken;
    RobotClientsSlamStatus exchangeSlamStatus = RobotClientsSlamStatus::SlamIdle;
    RobotClientsMapData exchangeMapData;

    // AutoTask
    lgdxrobot_cloud_msgs::msg::AutoTask currentTask;
    std::vector<RobotClientsPath> navigationPaths;
    std::size_t navigationProgress = 0;
    
    // Cloud
    std::shared_ptr<grpc::Channel> grpcChannel;
    std::unique_ptr<RobotClientsService::Stub> grpcRealtimeStub;
    std::unique_ptr<RobotClientsService::Stub> grpcStub;
    std::shared_ptr<grpc::CallCredentials> accessToken;
    CloudErrorRetryData cloudErrorRetryData;

    void Initalise();

    // Greet
    std::string GreetReadCertificate(const char *filename);
    #ifdef __linux__ 
    std::string GreetSetMotherBoardSN();
    #endif
    void GreetSetSystemInfo(RobotClientsSystemInfo *info);
    void Greet(std::string mcuSN);

    void ExchangeProcessData();
    void CloudExchange();
    void SlamExchange();
    void OnSlamMapUpdate(const nav_msgs::msg::OccupancyGrid &msg);

    void CloudAutoTaskNext();
    void CloudAutoTaskAbort(RobotClientsAbortReason reason);
    void OnNextExchange();
    void OnHandleClouldExchange(const RobotClientsResponse *response);
    void OnHandleSlamExchange(const RobotClientsSlamCommands *response);

    void NavigationStart();
    void OnNavigationDone();
    void OnNavigationAborted();
    void OnNavigationStuck();
    void OnNavigationCleared();

    void TryExitCriticalStatus();
    void HandleError();
    void OnErrorOccured();
    void Shutdown();

  public:
    CloudAdapter(const rclcpp::NodeOptions &options);
};

}

#endif // CLOUD_ADAPTER_HPP