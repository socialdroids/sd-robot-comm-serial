#ifndef INCLUDE_INCLUDE_NAVIGATION_TESTER_HPP_
#define INCLUDE_INCLUDE_NAVIGATION_TESTER_HPP_

#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class NavigationTester
{
public:
    NavigationTester();
    ~NavigationTester();

    void recordPoses(bool _status);
    void start();
    void stop();

    bool is_recording() const;
    bool is_running() const;

    std::string status() const;

    void add_goal(geometry_msgs::msg::PoseStamped& _goal);

private:
    using WaypointFollowerGoalHandle =
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>;

    bool record_poses_;
    bool running_;
    // Timeout value when waiting for action servers to respnd
    std::chrono::milliseconds server_timeout_;

    rclcpp::Node::SharedPtr node_;

    std::vector<geometry_msgs::msg::PoseStamped> acummulated_poses_;

    // The NavigateToPose action client
    rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr
        waypoint_follower_action_client_;
    // Goal-related state
    nav2_msgs::action::FollowWaypoints::Goal waypoint_follower_goal_;
    WaypointFollowerGoalHandle::SharedPtr waypoint_follower_goal_handle_;

    // Navigation action feedback subscribers
    rclcpp::Subscription<
        nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>::SharedPtr
        navigation_feedback_sub_;
    rclcpp::Subscription<
        nav2_msgs::action::NavigateToPose::Impl::GoalStatusMessage>::SharedPtr
        navigation_goal_status_sub_;
};

#endif // INCLUDE_INCLUDE_NAVIGATION_TESTER_HPP_
