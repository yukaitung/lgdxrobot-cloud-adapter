#ifndef ROUTE_NAVIGATION_HPP
#define ROUTE_NAVIGATION_HPP

#include <vector>

#include "Signals/NavigationSignals.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/compute_and_track_route.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"

#include "OpenNavigation.hpp"

class RouteNavigation
{
  using ComputeAndTrackRouteAction = nav2_msgs::action::ComputeAndTrackRoute;
  using NavigateThroughPosesAction = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ComputeAndTrackRouteAction>;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<NavigateThroughPosesAction>;

  private:
    rclcpp::Logger logger_;
    std::shared_ptr<NavigationSignals> navigationSignals;
    std::shared_ptr<OpenNavigation> openNavigation;
    rclcpp_action::Client<ComputeAndTrackRouteAction>::SharedPtr computeAndTrackRouteActionClient;
    int currentEdgeId = -1;
    bool haveNavigationGoal = false;

    void Response(const GoalHandle::SharedPtr &goalHandle);
    void Feedback(GoalHandle::SharedPtr, const std::shared_ptr<const ComputeAndTrackRouteAction::Feedback> feedback);
    void Result(const GoalHandle::WrappedResult &result);

  public:
    RouteNavigation(rclcpp::Node::SharedPtr node,
      std::shared_ptr<NavigationSignals> navigationSignalsPtr,
      std::shared_ptr<OpenNavigation> openNavigationPtr);
    void Start(geometry_msgs::msg::PoseStamped &waypoint1, geometry_msgs::msg::PoseStamped &waypoint2);
    void Abort();
};

#endif // ROUTE_NAVIGATION_HPP