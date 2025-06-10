#ifndef INCLUDE_INCLUDE_ROBOT_COMM_SERIAL_HPP_
#define INCLUDE_INCLUDE_ROBOT_COMM_SERIAL_HPP_

#include <chrono>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <string.h>

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include "command.pb.h"
#include "feedback.pb.h"

using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::microseconds;
using std::chrono::milliseconds;
using std::chrono::nanoseconds;

class RobotSerial : public rclcpp::Node
{
public:
    RobotSerial();
    ~RobotSerial();

private:
    static constexpr char PACKET_END[] = {"\0"};
    static constexpr size_t MAX_PACKET_SIZE{2048};
    // Somente para exemplo
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr bumper_vel_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr bumper_sub_;

    std::shared_ptr<serial::Serial> serial_port_;
    rclcpp::TimerBase::SharedPtr packet_timer_;
    rclcpp::TimerBase::SharedPtr reconnect_timer_;

    std::string buffer_;
    float packet_frequency_;
    std::chrono::time_point<high_resolution_clock> last_packet_time_;

    unsigned long baud_;
    std::string port_;
    bool connected_;
    void bumper_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    void packet_callback();
    void reconnect_callback();

    void connect();
};

#endif // INCLUDE_INCLUDE_ROBOT_COMM_SERIAL_HPP_
