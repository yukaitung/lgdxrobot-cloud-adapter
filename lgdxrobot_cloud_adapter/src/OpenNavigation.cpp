#include "lgdxrobot_cloud_adapter/OpenNavigation.hpp"

using namespace std::chrono_literals;

OpenNavigation::OpenNavigation(rclcpp::Node::SharedPtr node, 
    std::shared_ptr<NavigationSignals> navigationSignalsPtr,
    std::shared_ptr<RobotClientsAutoTaskNavProgress> navProgressPtr
 ) : logger_(node->get_logger())
{
  navigationSignals = navigationSignalsPtr;
  navProgress = navProgressPtr;

  navThroughPosesActionClient = rclcpp_action::create_client<NavigateThroughPosesAction>(
    node,
    "navigate_through_poses");
	planSubscription = node->create_subscription<nav_msgs::msg::Path>("plan",
		rclcpp::SensorDataQoS().reliable(),
    std::bind(&OpenNavigation::PlanCallback, this, std::placeholders::_1));
}

void OpenNavigation::Response(const GoalHandle::SharedPtr &goalHandle)
{
  if (!goalHandle)
  {
    RCLCPP_ERROR(logger_, "navThroughPoses goal was rejected by server, the task will be aborted.");
    navigationSignals->Abort();
  }
}

void OpenNavigation::Feedback(GoalHandle::SharedPtr, 
  const std::shared_ptr<const NavigateThroughPosesAction::Feedback> feedback)
{
  lastNavProgress = *navProgress;
  navProgress->set_eta(rclcpp::Duration(feedback->estimated_time_remaining).seconds());
  navProgress->set_recoveries(feedback->number_of_recoveries);
  navProgress->set_distanceremaining(feedback->distance_remaining);
  navProgress->set_waypointsremaining(feedback->number_of_poses_remaining);
  
  if (!isStuck)
  {
    // Determine if the robot is stuck by
    // 1. Recoveries is increasing
    // 2. Eta is 0
    if (navProgress->recoveries() > lastNavProgress.recoveries() && 
        navProgress->eta() == 0)
    {
      RCLCPP_INFO(logger_, "The robot is stuck.");
      isStuck = true;
      navigationSignals->Stuck();
    }
  }
  else
  {
    // Determine if the robot is cleared by
    // 1. Recoveries is unchanged
    // 2. Eta is not 0 and decreasing
    // 3. distanceRemaining is decreasing
    if (navProgress->recoveries() == lastNavProgress.recoveries() &&
        navProgress->eta() < lastNavProgress.eta() &&
        navProgress->eta() > 0 &&
        navProgress->distanceremaining() < lastNavProgress.distanceremaining())
    {
      
      RCLCPP_INFO(logger_, "The robot is cleared.");
      isStuck = false;
      navigationSignals->Cleared();
    }
  }
}

void OpenNavigation::Result(const GoalHandle::WrappedResult &result)
{
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
	navProgress->clear_plan();
  isStuck = false;
}

void OpenNavigation::PlanCallback(const nav_msgs::msg::Path &msg)
{
  navProgress->clear_plan();
  int size = msg.poses.size();
	for (int i = 0; i < size; i += kPlanSample)
	{
		auto *item = navProgress->add_plan();
		item->set_x(msg.poses[i].pose.position.x);
		item->set_y(msg.poses[i].pose.position.y);
	}
  if (size % (kPlanSample + 1) != 0)
  {
    auto *item = navProgress->add_plan();
    item->set_x(msg.poses[size - 1].pose.position.x);
    item->set_y(msg.poses[size - 1].pose.position.y);
  }
}

void OpenNavigation::Start(nav_msgs::msg::Goals &goals)
{
  using namespace std::placeholders;

  while (!navThroughPosesActionClient->wait_for_action_server(5s))
  {
    RCLCPP_ERROR(logger_, "navThroughPoses action server is not available yet.");
  }

  auto goal = NavigateThroughPosesAction::Goal();
  goal.poses = goals;

  auto goalOption = rclcpp_action::Client<NavigateThroughPosesAction>::SendGoalOptions();
  goalOption.goal_response_callback = std::bind(&OpenNavigation::Response, this, _1);
  goalOption.feedback_callback = std::bind(&OpenNavigation::Feedback, this, _1, _2);
  goalOption.result_callback = std::bind(&OpenNavigation::Result, this, _1);
  navThroughPosesActionClient->async_send_goal(goal, goalOption);
}

void OpenNavigation::Abort()
{
  if (navThroughPosesActionClient->wait_for_action_server())
  {
    // The NAV2 stack is running, cancel the goal
    auto cancelResult = navThroughPosesActionClient->async_cancel_all_goals(
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
  navProgress->clear_plan();
  isStuck = false;
}
