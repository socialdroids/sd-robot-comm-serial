#include "navigation_tester.hpp"
#include <string>

NavigationTester::NavigationTester() : server_timeout_(100)
{
    record_poses_ = false;
    running_ = false;
    node_ = std::make_shared<rclcpp::Node>("nav_tester");
    acummulated_poses_.clear();

    waypoint_follower_action_client_ =
        rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
            node_, "follow_waypoints");
    waypoint_follower_goal_ = nav2_msgs::action::FollowWaypoints::Goal();

    // create action feedback subscribers
    navigation_feedback_sub_ = node_->create_subscription<
        nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>(
        "navigate_to_pose/_action/feedback", rclcpp::SystemDefaultsQoS(),
        [this](const nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage::
                   SharedPtr msg) {
            // navigation_feedback_indicator_->setText(getNavToPoseFeedbackLabel(msg->feedback));
            std::string aux = std::string(
                "ETA: " +
                std::to_string(
                    rclcpp::Duration(msg->feedback.estimated_time_remaining)
                        .seconds()) +
                " s"
                "Distance remaining: " +
                std::to_string(msg->feedback.distance_remaining) +
                " m"
                "Time taken: " +
                std::to_string(
                    rclcpp::Duration(msg->feedback.navigation_time).seconds()) +
                " s"
                "Recoveries: " +
                std::to_string(msg->feedback.number_of_recoveries) + "");
            RCLCPP_INFO(node_->get_logger(), "[GOAL FEEDBACK] %s", aux.c_str());
        });

    // create action goal status subscribers
    navigation_goal_status_sub_ =
        node_->create_subscription<action_msgs::msg::GoalStatusArray>(
            "navigate_to_pose/_action/status", rclcpp::SystemDefaultsQoS(),
            [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
                std::string status_str;

                switch (msg->status_list.back().status)
                {
                case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
                    status_str = "active";
                    break;

                case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
                    status_str = "reached";
                    break;

                case action_msgs::msg::GoalStatus::STATUS_CANCELED:
                    status_str = "canceled";
                    break;

                case action_msgs::msg::GoalStatus::STATUS_ABORTED:
                    status_str = "aborted";
                    break;

                case action_msgs::msg::GoalStatus::STATUS_UNKNOWN:
                    status_str = "unknown";
                    break;

                default:
                    status_str = "inactive";
                    break;
                }
                RCLCPP_INFO(node_->get_logger(), "[GOAL STATUS] %s",
                            std::string("Feedback: " + status_str).c_str());
                // navigation_goal_status_indicator_->setText(
                //   getGoalStatusLabel(msg->status_list.back().status));
                // if (msg->status_list.back().status !=
                // action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
                //   navigation_feedback_indicator_->setText(getNavToPoseFeedbackLabel());
                // }
            });
}

NavigationTester::~NavigationTester()
{
}

void NavigationTester::recordPoses(bool _status)
{
    record_poses_ = _status;
}

void NavigationTester::start()
{
    running_ = true;

    auto is_action_server_ready =
        waypoint_follower_action_client_->wait_for_action_server(
            std::chrono::seconds(5));
    if (!is_action_server_ready)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "follow_waypoints action server is not available."
                     " Is the initial pose set?");
        return;
    }

    // Send the goal poses
    waypoint_follower_goal_.poses = acummulated_poses_;

    RCLCPP_INFO(node_->get_logger(), "Sending a path of %zu waypoints:",
                waypoint_follower_goal_.poses.size());
    for (auto waypoint : waypoint_follower_goal_.poses)
    {
        RCLCPP_INFO(node_->get_logger(), "\t(%lf, %lf)",
                    waypoint.pose.position.x, waypoint.pose.position.y);
    }

    // Enable result awareness by providing an empty lambda function
    auto send_goal_options = rclcpp_action::Client<
        nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
    send_goal_options.result_callback = [this](auto) {
        waypoint_follower_goal_handle_.reset();
    };

    auto future_goal_handle = waypoint_follower_action_client_->async_send_goal(
        waypoint_follower_goal_, send_goal_options);
    if (rclcpp::spin_until_future_complete(node_, future_goal_handle,
                                           server_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Send goal call failed");
        return;
    }

    // Get the goal handle and save so that we can check on completion in the
    // timer callback
    waypoint_follower_goal_handle_ = future_goal_handle.get();
    if (!waypoint_follower_goal_handle_)
    {
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
        return;
    }

    // timer_.start(200, this);
    // if (!waypoint_follower_goal_handle_)
    // {
    //     RCLCPP_DEBUG(client_node_->get_logger(), "Waiting for Goal");
    //     state_machine_.postEvent(new
    //     ROSActionQEvent(QActionState::INACTIVE)); return;
    // }
    //
    // rclcpp::spin_some(client_node_);
    // auto status = waypoint_follower_goal_handle_->get_status();
    //
    // // Check if the goal is still executing
    // if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
    //     status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
    // {
    //     state_machine_.postEvent(new ROSActionQEvent(QActionState::ACTIVE));
    // }
    // else
    // {
    //     state_machine_.postEvent(new
    //     ROSActionQEvent(QActionState::INACTIVE)); timer_.stop();
    // }
}

void NavigationTester::stop()
{
    if (waypoint_follower_goal_handle_)
    {
        auto future_cancel =
            waypoint_follower_action_client_->async_cancel_goal(
                waypoint_follower_goal_handle_);

        if (rclcpp::spin_until_future_complete(node_, future_cancel,
                                               server_timeout_) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "Failed to cancel waypoint follower");
        }
        else
        {
            waypoint_follower_goal_handle_.reset();
            running_ = false;
        }
    }
}

bool NavigationTester::is_recording() const
{
    return record_poses_;
}

bool NavigationTester::is_running() const
{
    return running_;
}

std::string NavigationTester::status() const
{
    if (record_poses_)
    {
        return std::string("Gravando");
    }
    else if (running_)
    {
        return std::string("Ativo");
    }
    return std::string("Desativado");
}
