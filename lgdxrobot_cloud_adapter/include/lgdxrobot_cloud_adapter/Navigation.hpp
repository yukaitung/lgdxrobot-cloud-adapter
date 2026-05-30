#ifndef NAV_THROUGH_POSES_HPP
#define NAV_THROUGH_POSES_HPP

#include <vector>

#include "Signals/NavigationSignals.hpp"

#include "nav_msgs/msg/goals.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "proto/RobotClientsService.grpc.pb.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"

class Navigation
{
  using NavigateThroughPosesAction = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateThroughPosesAction>;

  private:
    rclcpp::Logger logger_;

    const int kPlanSample = 10;
    bool isStuck = false;
    std::shared_ptr<NavigationSignals> navigationSignals;
    RobotClientsAutoTaskNavProgress lastNavProgress;
    std::shared_ptr<RobotClientsAutoTaskNavProgress> navProgress;

    rclcpp_action::Client<NavigateThroughPosesAction>::SharedPtr navThroughPosesActionClient;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr planSubscription;

    void Response(const GoalHandle::SharedPtr &goalHandle);
    void Feedback(GoalHandle::SharedPtr, const std::shared_ptr<const NavigateThroughPosesAction::Feedback> feedback);
    void Result(const GoalHandle::WrappedResult &result);
    void PlanCallback(const nav_msgs::msg::Path &msg);

  public:
    Navigation(rclcpp::Node::SharedPtr node,
      std::shared_ptr<NavigationSignals> navigationSignalsPtr,
      std::shared_ptr<RobotClientsAutoTaskNavProgress> navProgressPtr);
    void Start(nav_msgs::msg::Goals &goals);
    void Abort();
};

#endif // NAV_THROUGH_POSES_HPP