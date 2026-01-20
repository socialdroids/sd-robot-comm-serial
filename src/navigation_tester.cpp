#include "navigation_tester.hpp"
#include <rclcpp/logging.hpp>
#include <string>

NavigationTester::NavigationTester(rclcpp::Node::SharedPtr _parent,
                                   std::string _share_dir)
    : server_timeout_(100)
{
    record_poses_ = false;
    running_ = false;
    node_ = _parent;
    share_dir_ = _share_dir;
    acummulated_poses_.clear();

    rem_poses_ = 0;
    eta_ = 0;
    rem_distance_ = 0;
    total_time_ = 0;
    recoveries_ = 0;
    nav_status_ = "";

    waypoint_follower_action_client_ =
        rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
            node_, "follow_waypoints");
    waypoint_follower_goal_ = nav2_msgs::action::FollowWaypoints::Goal();

    // create action feedback subscribers
    navigation_feedback_sub_ = node_->create_subscription<
        nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>(
        "navigate_to_pose/_action/feedback", rclcpp::SystemDefaultsQoS(),
        std::bind(&NavigationTester::navigate_feedback_callback, this,
                  std::placeholders::_1));

    // create action goal status subscribers
    navigation_goal_status_sub_ =
        node_->create_subscription<action_msgs::msg::GoalStatusArray>(
            "navigate_to_pose/_action/status", rclcpp::SystemDefaultsQoS(),
            std::bind(&NavigationTester::navigate_status_callback, this,
                      std::placeholders::_1));

    waypoints_sub_ =
        node_->create_subscription<visualization_msgs::msg::MarkerArray>(
            "waypoints", rclcpp::QoS(1).transient_local(),
            std::bind(&NavigationTester::waypoints_callback, this,
                      std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "Navigation Tester Init!");
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
    if (!read_waypoints_file() && acummulated_poses_.empty())
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "Nenhuma pose definida para o trajeto!");
        return;
    }
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

void NavigationTester::add_goal(geometry_msgs::msg::PoseStamped& _goal)
{
}

geometry_msgs::msg::PoseStamped NavigationTester::last_goal()
{
}

int NavigationTester::remaining_poses()
{
    return rem_poses_;
}

float NavigationTester::eta()
{
    return eta_;
}

float NavigationTester::remaining_distance()
{
    return rem_distance_;
}

int NavigationTester::total_time()
{
    return total_time_;
}

int NavigationTester::recoveries()
{
    return recoveries_;
}

std::string NavigationTester::navigation_status()
{
    return nav_status_;
}

std::string NavigationTester::get_filename()
{
    return share_dir_ + "/config/nav_waypoints.json";
}

void NavigationTester::navigate_feedback_callback(
    const nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage::SharedPtr
        msg)
{
    eta_ = rclcpp::Duration(msg->feedback.estimated_time_remaining).seconds();
    rem_distance_ = msg->feedback.distance_remaining;
    total_time_ = rclcpp::Duration(msg->feedback.navigation_time).seconds();
    recoveries_ = msg->feedback.number_of_recoveries;
    // navigation_feedback_indicator_->setText(getNavToPoseFeedbackLabel(msg->feedback));
    std::string aux = std::string("ETA: " + std::to_string(eta_) +
                                  " s"
                                  "Distance remaining: " +
                                  std::to_string(rem_distance_) +
                                  " m"
                                  "Time taken: " +
                                  std::to_string(total_time_) +
                                  " s"
                                  "Recoveries: " +
                                  std::to_string(recoveries_) + "");
    RCLCPP_DEBUG(node_->get_logger(), "[GOAL FEEDBACK] %s", aux.c_str());
}

void NavigationTester::navigate_status_callback(
    const action_msgs::msg::GoalStatusArray::SharedPtr msg)
{
    switch (msg->status_list.back().status)
    {
    case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
        nav_status_ = "Ativo";
        break;

    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
        nav_status_ = "Chegou";
        break;

    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
        nav_status_ = "Cancelado";
        break;

    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
        nav_status_ = "Abortado";
        break;

    case action_msgs::msg::GoalStatus::STATUS_UNKNOWN:
        nav_status_ = "Desconhecido";
        break;

    default:
        nav_status_ = "Inativo";
        break;
    }
    RCLCPP_DEBUG(node_->get_logger(), "[GOAL STATUS] %s",
                 std::string("Feedback: " + nav_status_).c_str());
    // navigation_goal_status_indicator_->setText(
    //   getGoalStatusLabel(msg->status_list.back().status));
    // if (msg->status_list.back().status !=
    // action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
    //   navigation_feedback_indicator_->setText(getNavToPoseFeedbackLabel());
    // }
}

void NavigationTester::waypoints_callback(
    const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    if (!is_recording())
        return;

    auto pose = geometry_msgs::msg::PoseStamped();
    tf2::Quaternion q;
    acummulated_poses_.clear();

    json waypoints, json_point;
    waypoints["waypoints"] = json::array();

    for (const auto& marker : msg->markers)
    {
        if (marker.type == visualization_msgs::msg::Marker::SPHERE)
        {
            pose.header = marker.header;
            pose.pose = marker.pose;
            acummulated_poses_.push_back(pose);

            tf2::fromMsg(pose.pose.orientation, q);
            double r{}, p{}, yaw{};
            tf2::Matrix3x3 m(q);
            m.getRPY(r, p, yaw);

            json_point["x"] = pose.pose.position.x;
            json_point["y"] = pose.pose.position.y;
            json_point["yaw"] = yaw;
            json_point["q_x"] = pose.pose.orientation.x;
            json_point["q_y"] = pose.pose.orientation.y;
            json_point["q_z"] = pose.pose.orientation.z;
            json_point["q_w"] = pose.pose.orientation.w;

            waypoints["waypoints"].push_back(json_point);

            RCLCPP_INFO(node_->get_logger(),
                        "Added Pose %ld (%.2f; %.2f; %.2f)",
                        acummulated_poses_.size(), pose.pose.position.x,
                        pose.pose.position.y, yaw);
        }
    }
    std::ofstream json_file(get_filename());
    json_file << waypoints << std::endl;
}

bool NavigationTester::read_waypoints_file()
{
    std::ifstream json_file(get_filename());

    if (json_file.good()) // File exists
    {
        json waypoints;
        json_file >> waypoints;
        RCLCPP_INFO(node_->get_logger(), "JSON WAYPOINTS: %s", waypoints.dump().c_str());

        if (waypoints.is_array())
        {
            auto pose = geometry_msgs::msg::PoseStamped();
            tf2::Quaternion q;
            acummulated_poses_.clear();

            for (json::iterator it = waypoints.begin(); it != waypoints.end();
                 ++it)
            {
                pose.pose.position.x = (*it)["x"];
                pose.pose.position.y = (*it)["y"];
                pose.pose.orientation.x = (*it)["q_x"];
                pose.pose.orientation.y = (*it)["q_y"];
                pose.pose.orientation.z = (*it)["q_z"];
                pose.pose.orientation.w = (*it)["q_w"];
            }
            return true;
        }
    }
    return false;
}
