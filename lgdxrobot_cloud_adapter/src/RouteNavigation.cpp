#include "lgdxrobot_cloud_adapter/RouteNavigation.hpp"

using namespace std::chrono_literals;

RouteNavigation::RouteNavigation(rclcpp::Node::SharedPtr node, 
    std::shared_ptr<NavigationSignals> navigationSignalsPtr,
    std::shared_ptr<OpenNavigation> openNavigationPtr
 ) : logger_(node->get_logger())
{
  navigationSignals = navigationSignalsPtr;
  openNavigation = openNavigationPtr;
  computeAndTrackRouteActionClient = rclcpp_action::create_client<ComputeAndTrackRouteAction>(
    node,
    "compute_and_track_route");
}

void RouteNavigation::Response(const GoalHandle::SharedPtr &goalHandle)
{
  if (!goalHandle)
  {
    RCLCPP_ERROR(logger_, "computeAndTrackRoute goal was rejected by server, the task will be aborted.");
    navigationSignals->Abort();
  }
}

void RouteNavigation::Feedback(GoalHandle::SharedPtr, 
  const std::shared_ptr<const ComputeAndTrackRouteAction::Feedback> feedback)
{
  using namespace std::placeholders;

  if (feedback->current_edge_id != currentEdgeId)
  {
    RCLCPP_INFO(logger_, "Current edge: %d, last node: %d next node: %d", feedback->current_edge_id, feedback->last_node_id, feedback->next_node_id);
    currentEdgeId = feedback->current_edge_id;
  }
  if (feedback->rerouted)
  {
    RCLCPP_INFO(logger_, "Getting a new route.");
    
    nav_msgs::msg::Goals goals;
    goals.header.frame_id = "map";
    goals.goals = feedback->path.poses;
    for (auto &pose : goals.goals)
    {
      pose.header.frame_id = "map";
    }
    openNavigation->Start(goals);
  }
}

void RouteNavigation::Result(const GoalHandle::WrappedResult &result)
{
  RCLCPP_INFO(logger_, "computeAndTrackRoute completed.");
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      navigationSignals->Done();
      break;
    case rclcpp_action::ResultCode::ABORTED:
    case rclcpp_action::ResultCode::CANCELED:
    default:
      navigationSignals->Abort();
      return;
  }
}

void RouteNavigation::Start(geometry_msgs::msg::PoseStamped &waypoint1, geometry_msgs::msg::PoseStamped &waypoint2)
{
  using namespace std::placeholders;

  while (!computeAndTrackRouteActionClient->wait_for_action_server(5s))
  {
    RCLCPP_ERROR(logger_, "navThroughPoses action server is not available yet.");
  }

  auto goal = ComputeAndTrackRouteAction::Goal();
  goal.start = waypoint1;
  goal.goal = waypoint2;
  goal.use_poses = true;

  auto goalOption = rclcpp_action::Client<ComputeAndTrackRouteAction>::SendGoalOptions();
  goalOption.goal_response_callback = std::bind(&RouteNavigation::Response, this, _1);
  goalOption.feedback_callback = std::bind(&RouteNavigation::Feedback, this, _1, _2);
  goalOption.result_callback = std::bind(&RouteNavigation::Result, this, _1);
  computeAndTrackRouteActionClient->async_send_goal(goal, goalOption);
}

void RouteNavigation::Abort()
{
  if (computeAndTrackRouteActionClient->wait_for_action_server())
  {
    // The NAV2 stack is running, cancel the goal
    auto cancelResult = computeAndTrackRouteActionClient->async_cancel_all_goals(
      [this](auto response)
      {
        if (response)
        {
          RCLCPP_INFO(logger_, "Navigation aborted.");
        }
        else
        {
          RCLCPP_ERROR(logger_, "Navigation abort failed.");
        }
      }
    );
  }
  navigationSignals->Abort();
}
