#ifndef INCLUDE_INCLUDE_NAVIGATION_TESTER_HPP_
#define INCLUDE_INCLUDE_NAVIGATION_TESTER_HPP_

#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <nlohmann/json.hpp>
#include <fstream>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using json = nlohmann::json;

class NavigationTester
{
public:
    NavigationTester(rclcpp::Node::SharedPtr _parent, std::string _share_dir);
    ~NavigationTester();

    void recordPoses(bool _status);
    void start();
    void stop();

    bool is_recording() const;
    bool is_running() const;

    std::string status() const;

    void add_goal(geometry_msgs::msg::PoseStamped& _goal);
    geometry_msgs::msg::PoseStamped last_goal();

    int remaining_poses();
    float eta();
    float remaining_distance();
    int total_time();
    int recoveries();
    std::string navigation_status();
    std::string get_filename();

private:
    using WaypointFollowerGoalHandle =
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>;

    void navigate_feedback_callback(const nav2_msgs::action::NavigateToPose::
                                        Impl::FeedbackMessage::SharedPtr msg);
    void navigate_status_callback(
        const action_msgs::msg::GoalStatusArray::SharedPtr msg);

    void waypoints_callback(
        const visualization_msgs::msg::MarkerArray::SharedPtr msg);

    bool read_waypoints_file();

    std::string share_dir_;
    bool record_poses_;
    bool running_;

    geometry_msgs::msg::Pose last_pose_;
    int rem_poses_;
    float eta_;
    float rem_distance_;
    int total_time_;
    int recoveries_;
    std::string nav_status_;

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
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
        waypoints_sub_;
    rclcpp::Subscription<
        nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>::SharedPtr
        navigation_feedback_sub_;
    rclcpp::Subscription<
        nav2_msgs::action::NavigateToPose::Impl::GoalStatusMessage>::SharedPtr
        navigation_goal_status_sub_;
};

#endif // INCLUDE_INCLUDE_NAVIGATION_TESTER_HPP_
