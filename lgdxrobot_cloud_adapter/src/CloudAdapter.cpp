#include <chrono>
#include <random>
#include <fstream>

#include "hwinfo/hwinfo.h"
#include "hwinfo/utils/unit.h"

#include <rclcpp_components/register_node_macro.hpp>
#include "lgdxrobot_cloud_adapter/CloudAdapter.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "lgdxrobot_cloud_adapter/SaveMap.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_msgs/srv/set_initial_pose.hpp"

using namespace std::chrono_literals;

namespace LGDXRobotCloud
{
  
CloudAdapter::CloudAdapter(const rclcpp::NodeOptions &options) : Node("lgdxrobot_cloud_adapter_node", options)
{
  timer = this->create_wall_timer(std::chrono::microseconds(1), [this]() {this->Initalise();});
}

CloudAdapter::~CloudAdapter()
{
  Shutdown();
}

void CloudAdapter::Initalise()
{
  timer->cancel();

  // Parameters
  auto cloudSlamEnableParam = rcl_interfaces::msg::ParameterDescriptor{};
  cloudSlamEnableParam.description = "Enable SLAM Mode.";
  this->declare_parameter("slam_enable", false, cloudSlamEnableParam);
  auto cloudAddressParam = rcl_interfaces::msg::ParameterDescriptor{};
  cloudAddressParam.description = "Address of LGDXRobot Cloud.";
  this->declare_parameter("address", "", cloudAddressParam);
  auto cloudRootCertParam = rcl_interfaces::msg::ParameterDescriptor{};
  cloudRootCertParam.description = "Path to server root certificate.";
  this->declare_parameter("root_cert", "", cloudRootCertParam);
  auto cloudClientKeyParam = rcl_interfaces::msg::ParameterDescriptor{};
  cloudClientKeyParam.description = "Path to client's private key.";
  this->declare_parameter("client_key", "", cloudClientKeyParam);
  auto cloudClientCertParam = rcl_interfaces::msg::ParameterDescriptor{};
  cloudClientCertParam.description = "Path to client's certificate chain.";
  this->declare_parameter("client_cert", "", cloudClientCertParam);

  isSlam = this->get_parameter("slam_enable").as_bool();
  // Components
  cloudSignals = std::make_shared<CloudSignals>();
  navigationSignals = std::make_shared<NavigationSignals>();
  navProgress = std::make_shared<RobotClientsAutoTaskNavProgress>();
  openNavigation = std::make_shared<OpenNavigation>(shared_from_this(), navigationSignals, navProgress);
  routeNavigation = std::make_unique<RouteNavigation>(shared_from_this(), navigationSignals, openNavigation);
  if (isSlam)
  {
    map = std::make_unique<Map>(shared_from_this());
  }

  navigationSignals->Done.connect(boost::bind(&CloudAdapter::OnNavigationDone, this));
  navigationSignals->Stuck.connect(boost::bind(&CloudAdapter::OnNavigationStuck, this));
  navigationSignals->Cleared.connect(boost::bind(&CloudAdapter::OnNavigationCleared, this));
  navigationSignals->Abort.connect(boost::bind(&CloudAdapter::OnNavigationAborted, this));

  cloudSignals->StreamError.connect(boost::bind(&CloudAdapter::OnErrorOccured, this));
  if (isSlam)
  {
    cloudSignals->NextExchange.connect(boost::bind(&CloudAdapter::OnNextExchange, this));
    cloudSignals->HandleSlamExchange.connect(boost::bind(&CloudAdapter::OnHandleSlamExchange, this, boost::placeholders::_1));
  }
  else
  {
    cloudSignals->NextExchange.connect(boost::bind(&CloudAdapter::OnNextExchange, this));
    cloudSignals->HandleExchange.connect(boost::bind(&CloudAdapter::OnHandleClouldExchange, this, boost::placeholders::_1));
  }

  // ROS
  nav2DelayClient = this->create_client<std_srvs::srv::Empty>("cloud/nav2_delay");
  tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
  if (isSlam)
  {
    cloudExchangeTimer = this->create_wall_timer(std::chrono::milliseconds(200), 
      std::bind(&CloudAdapter::SlamExchange, this));
    cloudExchangeTimer->cancel();

    mapSubscription = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 
      rclcpp::SensorDataQoS().reliable(), 
      std::bind(&CloudAdapter::OnSlamMapUpdate, this, std::placeholders::_1));
  }
  else
  {
    cloudExchangeTimer = this->create_wall_timer(std::chrono::milliseconds(500), 
      std::bind(&CloudAdapter::CloudExchange, this));
    cloudExchangeTimer->cancel();

    // Subscribers
    robotDataSubscription = this->create_subscription<lgdxrobot_cloud_msgs::msg::RobotData>("cloud/robot_data", 
      rclcpp::SensorDataQoS().reliable(),
      [this](const std::shared_ptr<lgdxrobot_cloud_msgs::msg::RobotData> msg)
      {
        if (msg->hardware_emergency_stop_enabled)
        {
          criticalStatus.set_hardwareemergencystop(true);
          robotStatus.EnterCritical();
        }
        else
        {
          criticalStatus.set_hardwareemergencystop(false);
          TryExitCriticalStatus();
        }
        if (msg->batteries_percentage.size() > 0)
        {
          batteries[0] = msg->batteries_percentage[0];
        }
        if (msg->batteries_percentage.size() > 1)
        {
          batteries[1] = msg->batteries_percentage[1];
        }
      });
    // Topics
    softwareEmergencyStopPublisher = this->create_publisher<std_msgs::msg::Bool>("cloud/software_emergency_stop", 
      rclcpp::SensorDataQoS().reliable());
    softwareEmergencyStopPublisherTimer = this->create_wall_timer(std::chrono::milliseconds(100),
      [this]()
      {
        auto message = std_msgs::msg::Bool();
        message.data = criticalStatus.softwareemergencystop();
        softwareEmergencyStopPublisher->publish(message);
      });
    autoTaskPublisher = this->create_publisher<lgdxrobot_cloud_msgs::msg::AutoTask>("cloud/auto_task", 
      rclcpp::SensorDataQoS().reliable());
    autoTaskPublisherTimer = this->create_wall_timer(std::chrono::milliseconds(100), 
      [this]()
      {
        autoTaskPublisher->publish(currentTask);
      });

    // Services
    autoTaskNextService = this->create_service<lgdxrobot_cloud_msgs::srv::AutoTaskNext>("auto_task_next",
      [this](const std::shared_ptr<lgdxrobot_cloud_msgs::srv::AutoTaskNext::Request> request,
        std::shared_ptr<lgdxrobot_cloud_msgs::srv::AutoTaskNext::Response> response) 
      {
        if (!currentTask.next_token.empty() && 
            request->task_id == currentTask.task_id &&
            request->next_token == currentTask.next_token)
        {
          CloudAutoTaskNext();
          response->success = true;
        }
        else
        {
          response->success = false;
        }
      });
    autoTaskAbortService = this->create_service<lgdxrobot_cloud_msgs::srv::AutoTaskAbort>("auto_task_abort",
      [this](const std::shared_ptr<lgdxrobot_cloud_msgs::srv::AutoTaskAbort::Request> request,
        std::shared_ptr<lgdxrobot_cloud_msgs::srv::AutoTaskAbort::Response> response)
      {
        if (!currentTask.next_token.empty() && 
            request->task_id == currentTask.task_id &&
            request->next_token == currentTask.next_token)
        {
          CloudAutoTaskAbort(RobotClientsAbortReason::Robot);
          response->success = true;
        }
        else
        {
          response->success = false;
        }
      });
  }

  // Cloud Initalise
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(1000, 6000);
  int cloudRetryWait = dis(gen);
  RCLCPP_INFO(this->get_logger(), "LGDXRobot Cloud is enabled, the break for reconnection to the cloud is %d ms.", cloudRetryWait);
  cloudRetryTimer = this->create_wall_timer(std::chrono::milliseconds(cloudRetryWait), 
    std::bind(&CloudAdapter::HandleError, this));
  cloudRetryTimer->cancel();
  
  // Connect to Cloud
  std::string serverAddress = this->get_parameter("address").as_string();
  std::string rootCertPath = this->get_parameter("root_cert").as_string();
  std::string clientKeyPath = this->get_parameter("client_key").as_string();
  std::string clientCertPath = this->get_parameter("client_cert").as_string();
  std::string rootCert = GreetReadCertificate(rootCertPath.c_str());
  std::string clientKey = GreetReadCertificate(clientKeyPath.c_str());
  std::string clientCert = GreetReadCertificate(clientCertPath.c_str());
  grpc::SslCredentialsOptions sslOptions = {rootCert, clientKey, clientCert};

  grpcChannel = grpc::CreateChannel(serverAddress, grpc::SslCredentials(sslOptions));
  grpcStub = RobotClientsService::NewStub(grpcChannel);
  accessToken = grpc::AccessTokenCredentials("");

  Greet();
}

std::tuple<double, double, double> CloudAdapter::GetCurrentPosition()
{
  try
  {
    geometry_msgs::msg::TransformStamped t;
    t = tfBuffer->lookupTransform("base_link", "map", tf2::TimePointZero);

    tf2::Quaternion q(
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    double x = -(t.transform.translation.x * cos(yaw) + t.transform.translation.y * sin(yaw));
    double y = -(-t.transform.translation.x * sin(yaw) + t.transform.translation.y * cos(yaw));

    return {x, y, yaw};
  }
  catch (const tf2::TransformException &ex) 
  {
    return {0.0, 0.0, 0.0};
  }
}

std::string CloudAdapter::GreetReadCertificate(const char *filename)
{
  std::ifstream file(filename, std::ios::in);
  if (file.is_open())
  {
    std::stringstream ss;
		ss << file.rdbuf();
		file.close();
		return ss.str();
  }
  return {};
}

#ifdef __linux__ 
std::string CloudAdapter::GreetSetMotherBoardSN()
{
  std::ifstream file("/sys/class/dmi/id/board_serial", std::ios::in);
  std::string serialNumber;
  if (file.is_open())
  {
    std::getline(file, serialNumber);
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Unable to read motherboard serial number.");
  }
  return serialNumber;
}
#endif

void CloudAdapter::GreetSetSystemInfo(RobotClientsSystemInfo *info)
{
  hwinfo::MainBoard main_board;
  info->set_motherboard(main_board.name());
  #ifdef __linux__ 
    info->set_motherboardserialnumber(GreetSetMotherBoardSN());
  #else
    info->set_motherboardserialnumber(main_board.serialNumber());
  #endif
  const auto cpus = hwinfo::getAllCPUs();
  if (cpus.size() > 0)
  {
    hwinfo::CPU cpu = cpus.at(0);
    info->set_cpu(cpu.modelName());
  }
  else
  {
    info->set_cpu("");
  }
  hwinfo::OS os;
  info->set_os(os.name());
  info->set_is32bit(os.is32bit());
  info->set_islittleendian(os.isLittleEndian());
  const auto gpus = hwinfo::getAllGPUs();
  if (gpus.size() > 0)
  {
    hwinfo::GPU gpu = gpus.at(0);
    info->set_gpu(gpu.name());
  }
  hwinfo::Memory memory;
  info->set_rammib(hwinfo::unit::bytes_to_MiB(memory.total_Bytes()));
}

void CloudAdapter::Greet()
{
  RCLCPP_INFO(this->get_logger(), "Connecting to the cloud.");
  
  grpc::ClientContext *context = new grpc::ClientContext();
  auto deadline = std::chrono::system_clock::now() + std::chrono::seconds(kGrpcWaitSec);
  context->set_deadline(deadline);

  RobotClientsGreet *request = new RobotClientsGreet();
  RobotClientsSystemInfo *systemInfo = new RobotClientsSystemInfo();
  GreetSetSystemInfo(systemInfo);
  request->set_allocated_systeminfo(systemInfo);

  RobotClientsGreetResponse *response = new RobotClientsGreetResponse();
  grpcStub->async()->Greet(context, request, response, [context, request, response, this](grpc::Status status)
  {
    if (status.ok()) 
    {
      accessToken = grpc::AccessTokenCredentials(response->accesstoken());
      if (grpcRealtimeStub == nullptr)
      {
        grpcRealtimeStub = RobotClientsService::NewStub(grpcChannel);
      }
      if (isSlam)
      {
        RCLCPP_INFO(this->get_logger(), "Connected to the cloud, start SLAM data exchange.");
        exchangeStream = std::make_unique<SlamExchangeType>(grpcRealtimeStub.get(), accessToken, cloudSignals);
      }
      else
      {
        std::string route = response->mapinfo().route();
        if (!route.empty())
        {
          SaveMap::SaveRoute(route, this->get_logger());
          isRouteNavigation = true;
        }
        MapParameters mapParameters;
        mapParameters.mapWidth = response->mapinfo().mapwidth();
        mapParameters.mapHeight = response->mapinfo().mapheight();
        mapParameters.resolution = response->mapinfo().resolution();
        mapParameters.originX = response->mapinfo().originx();
        mapParameters.originY = response->mapinfo().originy();
        mapParameters.originRotation = response->mapinfo().originrotation();
        SaveMap::SaveMapData(mapParameters, response->mapinfo().map(), this->get_logger());
        SaveMap::SaveKeepoutMask(mapParameters, response->mapinfo().keepoutmask(), this->get_logger());
        SaveMap::SaveSpeedMask(mapParameters, response->mapinfo().speedmask(), this->get_logger());

        // Start Nav2 after loading maps
        RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 to be ready.");
        while (!nav2DelayClient->wait_for_service(1s)) {
          if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            delete context;
            delete request;
            delete response;
            return;
          }
        }
        nav2DelayClient->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
        WaitForNav2();
        RCLCPP_INFO(this->get_logger(), "Nav2 is ready.");

        RCLCPP_INFO(this->get_logger(), "Connected to the cloud, start data exchange.");
        robotStatus.ConnnectedCloud();
        exchangeStream = std::make_unique<CloudExchangeType>(grpcRealtimeStub.get(), accessToken, cloudSignals);
      }

      // Start the timer to exchange data
      cloudExchangeTimer->reset();
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Unable to connect the cloud, will try again.");
      OnErrorOccured();
    }
    delete context;
    delete request;
    delete response;
  });
}

void CloudAdapter::WaitForNav2()
{
  auto client = this->create_client<lifecycle_msgs::srv::GetState>("bt_navigator/get_state");
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
  }
  bool isReady = false;
  while (!isReady)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    client->async_send_request(request, [this, &isReady](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future)
    {
      auto result = future.get();
      if (result->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        isReady = true;
      }
    });
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

void CloudAdapter::ExchangeProcessData()
{
  if (isSlam)
  {
    exchangeRobotData.set_robotstatus(robotStatus.GetStatus() == RobotClientsRobotStatus::Critical ? RobotClientsRobotStatus::Critical : RobotClientsRobotStatus::Paused);
  }
  else
  {
    exchangeRobotData.set_robotstatus(robotStatus.GetStatus());
  }
  exchangeRobotData.mutable_criticalstatus()->CopyFrom(criticalStatus);
  auto exchangeBatteries = exchangeRobotData.mutable_batteries();
  exchangeBatteries->Clear();
  exchangeBatteries->Reserve(batteries.size());
  for (size_t i = 0; i < batteries.size(); i++)
  {
    exchangeBatteries->AddAlreadyReserved(batteries[i]);
  }
  auto [x, y, rotation] = GetCurrentPosition();
  exchangeRobotData.mutable_position()->set_x(x);
  exchangeRobotData.mutable_position()->set_y(y);
  exchangeRobotData.mutable_position()->set_rotation(rotation);
  exchangeRobotData.mutable_navprogress()->CopyFrom(*navProgress);
  exchangeRobotData.set_pausetaskassignment(pauseTaskAssignment);
}

void CloudAdapter::CloudExchange()
{
  if (!cloudExchangeTimer->is_canceled())
    cloudExchangeTimer->cancel();

  ExchangeProcessData();

  if (exchangeStream != nullptr)
  {
    exchangeStream->SendMessage(exchangeRobotData, exchangeNextToken, exchangeAbortToken);
  }

  // Clear the token to indicate that the exchange is done
  exchangeNextToken.Clear();
  exchangeAbortToken.Clear();
  // Don't reset the cloudExchangeTimer here
}

void CloudAdapter::SlamExchange()
{
  if (!cloudExchangeTimer->is_canceled())
  cloudExchangeTimer->cancel();

  ExchangeProcessData();

  exchangeMapData.Clear();
  if (mapHasUpdated)
  {
    exchangeMapData.CopyFrom(mapData);
    mapHasUpdated = false;
  }
  
  if (exchangeStream != nullptr)
  {
    exchangeStream->SendMessage(exchangeSlamStatus, exchangeRobotData, exchangeMapData);
  }
  // Don't reset the slamExchangeTimer here
}

void CloudAdapter::OnSlamMapUpdate(const nav_msgs::msg::OccupancyGrid &msg)
{
  // Compare the map with the current map
  int incomingSize = (int)msg.data.size();
  if (incomingSize == mapData.data_size())
  {
    bool same = true;
    for (size_t i = 0; i < msg.data.size(); i++)
    {
      if ((int32_t)msg.data[i] != mapData.data(i))
      {
        same = false;
        break;
      }
    }
    if (same)
    {
      return;
    }
  }

  // Update the map
  mapData.set_resolution(msg.info.resolution);
  mapData.set_width(msg.info.width);
  mapData.set_height(msg.info.height);
  // Origin
  auto origin = mapData.mutable_origin();
  origin->set_x(msg.info.origin.position.x);
  origin->set_y(msg.info.origin.position.y);
  tf2::Quaternion q(
    msg.info.origin.orientation.x,
    msg.info.origin.orientation.y,
    msg.info.origin.orientation.z,
    msg.info.origin.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  origin->set_rotation(yaw);
  // Map Data
  auto md = mapData.mutable_data();
  md->Clear();
  md->Reserve(incomingSize);
  for (int i = 0; i < incomingSize; i++)
  {
    md->AddAlreadyReserved(msg.data[i]);
  }
  mapHasUpdated = true;
}

void CloudAdapter::CloudAutoTaskNext()
{
  if (!currentTask.next_token.empty())
  {
    // Setup the next token for the exchange
    RCLCPP_INFO(this->get_logger(), "AutoTask advances to next progress.");
    exchangeNextToken.set_taskid(currentTask.task_id);
    exchangeNextToken.set_nexttoken(currentTask.next_token);
  }
}

void CloudAdapter::CloudAutoTaskAbort(RobotClientsAbortReason reason)
{
  if (!currentTask.next_token.empty())
  {
    // Setup the next token for the exchange
    robotStatus.TaskAborting();
    RCLCPP_INFO(this->get_logger(), "AutoTask will be aborted.");
    exchangeAbortToken.set_taskid(currentTask.task_id);
    exchangeAbortToken.set_nexttoken(currentTask.next_token);
    exchangeAbortToken.set_abortreason(reason);
  }
}

void CloudAdapter::OnNextExchange()
{
  cloudExchangeTimer->reset();
}

void CloudAdapter::OnHandleClouldExchange(const RobotClientsResponse *response)
{
  // Handle AutoTask
  if (response->has_task())
  {
    RobotClientsAutoTask task = response->task();
    currentTask.task_id = task.taskid();
    currentTask.task_name = task.taskname();
    currentTask.task_progress_id = task.taskprogressid();
    currentTask.task_progress_name = task.taskprogressname();
    currentTask.next_token = task.nexttoken();
    if (currentTask.task_progress_id == 3)
    {
      RCLCPP_INFO(this->get_logger(), "AutoTask Id: %d completed.", task.taskid());
      robotStatus.TaskCompleted();
    }
    else if (currentTask.task_progress_id == 4)
    {
      RCLCPP_INFO(this->get_logger(), "AutoTask Id: %d aborted.", task.taskid());
      if (isRouteNavigation)
      {
        routeNavigation->Abort();
      }
      else
      {
        openNavigation->Abort();
      }
      robotStatus.TaskAborted();
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Received AutoTask Id: %d, Progress: %d", task.taskid(), task.taskprogressid());
      if (task.paths_size())
      {
        RCLCPP_INFO(this->get_logger(), "This task has %d waypoint(s).", task.paths_size());
        navigationPath.clear();
        navigationPath.assign(task.paths().begin(), task.paths().end());
        navigationProgress = 0;
        NavigationStart();
      }
      robotStatus.TaskAssigned();
    }
  }

  // Handle Robot Commands
  if (response->has_commands())
  {
    auto commands = response->commands();
    if (commands.has_aborttask() && commands.aborttask() == true)
    {
      CloudAutoTaskAbort(RobotClientsAbortReason::UserApi);
    }
    if (commands.has_softwareemergencystopenable() && commands.softwareemergencystopenable() == true)
    {
      RCLCPP_INFO(this->get_logger(), "Enabling software emergency stop");
      criticalStatus.set_softwareemergencystop(true);
      robotStatus.EnterCritical();
    }
    if (commands.has_softwareemergencystopdisable() && commands.softwareemergencystopdisable() == true)
    {
      RCLCPP_INFO(this->get_logger(), "Disabling software emergency stop");
      criticalStatus.set_softwareemergencystop(false);
      TryExitCriticalStatus();
    }
    if (commands.has_pausetaskassignmentenable() && commands.pausetaskassignmentenable() == true)
    {
      RCLCPP_INFO(this->get_logger(), "Pausing task Assignment");
      pauseTaskAssignment = true;
      robotStatus.PauseTaskAssignment();
    }
    if (commands.has_pausetaskassignmentdisable() && commands.pausetaskassignmentdisable() == true)
    {
      RCLCPP_INFO(this->get_logger(), "Resuming task Assignment");
      pauseTaskAssignment = false;
      robotStatus.ResumeTaskAssignment();
    }
  }
}

void CloudAdapter::OnHandleSlamExchange(const RobotClientsSlamCommands *respond)
{
  if (respond->has_setgoal())
  {
    if (exchangeSlamStatus == RobotClientsSlamStatus::SlamRunning)
    {
      overwriteGoal = true;
    }
    double x = respond->setgoal().x();
    double y = respond->setgoal().y();
    double rotation = respond->setgoal().rotation();
    RCLCPP_INFO(this->get_logger(), "A new goal is set: %fm, %fm, %frad", x, y, rotation);
    nav_msgs::msg::Goals goals;
    goals.header.frame_id = "map";
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(rotation);
    goals.goals.push_back(pose);
    openNavigation->Start(goals);
    exchangeSlamStatus = RobotClientsSlamStatus::SlamRunning;
  }
  if (respond->has_abortgoal() && respond->abortgoal() == true)
  {
    RCLCPP_INFO(this->get_logger(), "Aborting the current goal");
    openNavigation->Abort();
  }
  if (respond->has_softwareemergencystopenable() && respond->softwareemergencystopenable() == true)
  {
    RCLCPP_INFO(this->get_logger(), "Enabling software emergency stop");
    criticalStatus.set_softwareemergencystop(true);
    robotStatus.EnterCritical();
  }
  if (respond->has_softwareemergencystopdisable() && respond->softwareemergencystopdisable() == true)
  {
    RCLCPP_INFO(this->get_logger(), "Disabling software emergency stop");
    criticalStatus.set_softwareemergencystop(false);
    TryExitCriticalStatus();
  }
  if (respond->has_savemap() && respond->savemap() == true)
  {
    RCLCPP_INFO(this->get_logger(), "Saving the map");
    map->Save();
  }
  if (respond->has_refreshmap() && respond->refreshmap() == true)
  {
    RCLCPP_INFO(this->get_logger(), "Refreshing the map");
    mapHasUpdated = true;
  }
  if (respond->has_abortslam() && respond->abortslam() == true)
  {
    RCLCPP_INFO(this->get_logger(), "Aborting the current SLAM");
    openNavigation->Abort();
    Shutdown();
  }
  if (respond->has_completeslam() && respond->completeslam() == true)
  {
    openNavigation->Abort();
    RCLCPP_INFO(this->get_logger(), "Completing the current SLAM and saving the map with 5 seconds blocking");
    map->Save();
    std::this_thread::sleep_for(std::chrono::seconds(5));
    Shutdown();
  }
}

void CloudAdapter::NavigationStart()
{
  if (navigationProgress < navigationPath.size())
  {
    if (isRouteNavigation)
    {
      // For route navigation, we plan a path from the current position to the next waypoint
      auto [x, y, rotation] = GetCurrentPosition();
      geometry_msgs::msg::PoseStamped current;
      current.header.frame_id = "map";
      current.pose.position.x = x;
      current.pose.position.y = y;
      current.pose.position.z = 0.0;
      current.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(rotation);
      const RobotClientsDof w2 = navigationPath.at(navigationProgress);
      geometry_msgs::msg::PoseStamped goal;
      goal.header.frame_id = "map";
      goal.pose.position.x = w2.x();
      goal.pose.position.y = w2.y();
      goal.pose.position.z = 0.0;
      goal.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(w2.rotation());
      routeNavigation->Start(current, goal);
      // Plan path to next waypoint
      navigationProgress++;
    }
    else
    {
      // For normal navigation, we can just ask NAV2 to follow a path
      nav_msgs::msg::Goals goals;
      goals.header.frame_id = "map";
      for (auto &waypoint : navigationPath)
      {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = waypoint.x();
        pose.pose.position.y = waypoint.y();
        pose.pose.position.z = 0.0;
        pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(waypoint.rotation());
        goals.goals.push_back(pose);
      }
      openNavigation->Start(goals);
      // go to next step
      navigationProgress = navigationPath.size();
    }
  }
  else
  {
    CloudAutoTaskNext();
  }
}

void CloudAdapter::OnNavigationDone()
{
  if (isSlam)
  {
    exchangeSlamStatus = RobotClientsSlamStatus::SlamSuccess;
  }
  else
  {
    NavigationStart();
  }
}

void CloudAdapter::OnNavigationStuck()
{
  if (!isSlam) 
  {
    robotStatus.NavigationStuck();
  }
  // Do nothing is SLAM
}

void CloudAdapter::OnNavigationCleared()
{
  if (!isSlam) 
  {
    robotStatus.NavigationCleared();
  }
  // Do nothing is SLAM
}

void CloudAdapter::OnNavigationAborted()
{
  if (isSlam)
  {
    if (overwriteGoal)
    {
      // Not showing aborted because the goal was overwritten
      overwriteGoal = false;
    }
    else
    {
      exchangeSlamStatus = RobotClientsSlamStatus::SlamAborted;
    }
  }
  else
  {
    CloudAutoTaskAbort(RobotClientsAbortReason::NavStack);
  }
}

void CloudAdapter::TryExitCriticalStatus()
{
  // Ensure no unreslovable status
  if (criticalStatus.hardwareemergencystop() == true)
  {
    RCLCPP_ERROR(this->get_logger(), "Unresolvable critical status, will not exit.");
    return;
  }
  robotStatus.ExitCritical();
}

void CloudAdapter::HandleError()
{
  cloudRetryTimer->cancel();
  Greet();
  // set hasError to false when greet success
}

void CloudAdapter::OnErrorOccured()
{
  if (cloudRetryTimer->is_canceled())
  {
    RCLCPP_ERROR(this->get_logger(), "Cloud error occured.");
    cloudRetryTimer->reset();
  }
}

void CloudAdapter::Shutdown()
{
  if (cloudExchangeTimer != nullptr && !cloudExchangeTimer->is_canceled())
  {
    cloudExchangeTimer->cancel();
  }

  if (exchangeStream != nullptr)
  {
    exchangeStream->Shutdown();
    exchangeStream->AwaitCompletion();
  }
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(LGDXRobotCloud::CloudAdapter)