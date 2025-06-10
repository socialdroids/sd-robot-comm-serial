#include "robot_comm_serial.hpp"
#include <rclcpp/logging.hpp>

RobotSerial::RobotSerial()
    : Node("RobotSerial"), baud_(1000000), port_("/dev/ttyACM0"),
      connected_(false)
{
    bumper_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel_bumpers", 30);
    bumper_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/robot_base/bumpers", 10,
        std::bind(&RobotSerial::bumper_callback, this, std::placeholders::_1));

    connect();

    buffer_.reserve(MAX_PACKET_SIZE);

    packet_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(1),
                                std::bind(&RobotSerial::packet_callback, this));
    reconnect_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&RobotSerial::reconnect_callback, this));
    last_packet_time_ = high_resolution_clock::now();
    packet_frequency_ = 0;
}

RobotSerial::~RobotSerial()
{
}

void RobotSerial::bumper_callback(
    const std_msgs::msg::Int32MultiArray::SharedPtr)
{
}

void RobotSerial::packet_callback()
{
    if (!connected_)
    {
        RCLCPP_WARN(this->get_logger(), "Serial port disconnected!");
        return;
    }
    if (serial_port_->available() > 0)
    {
        RCLCPP_INFO(this->get_logger(), "Data available! %ld",
                    serial_port_->available());
        size_t packet_size = 0;
        buffer_.clear();

        try
        {
            // process packet
            FeedbackMessage message;
            packet_size =
                serial_port_->read(buffer_, serial_port_->available());

            packet_frequency_ =
                1e9 / (duration_cast<nanoseconds>(high_resolution_clock::now() -
                                                  last_packet_time_))
                          .count();

            last_packet_time_ = high_resolution_clock::now();
            std::cout << "Data = ";
            for (size_t n = 0; n < packet_size; ++n)
            {
                unsigned int num = buffer_.c_str()[n];
                std::cout << std::hex << num << std::dec << ", ";
            }
            std::cout << "\n";
            RCLCPP_INFO(this->get_logger(), "%f) Data", packet_frequency_);

            bool ok = message.ParseFromArray(buffer_.c_str(), packet_size);
            if (ok)
            {
                RCLCPP_INFO(this->get_logger(), "Received protobuf message!");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(),
                             "Failed to parse Protobuf message");
            }
        }
        catch (serial::PortNotOpenedException& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to read data! Reason: %s.",
                         e.what());
        }
        catch (serial::SerialException& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to read data! Reason: %s.",
                         e.what());
        }
    }
}

void RobotSerial::reconnect_callback()
{
    if (!connected_)
    {
        RCLCPP_INFO(this->get_logger(), "Trying to reconnect...");
        connect();
    }
}

void RobotSerial::connect()
{
    try
    {
        serial_port_ = std::make_shared<serial::Serial>(
            port_, baud_, serial::Timeout::simpleTimeout(10));
    }
    catch (serial::IOException& e)
    {
        serial_port_ = std::make_shared<serial::Serial>();
        RCLCPP_INFO(this->get_logger(),
                    "Failed to open serial port! Reason: %s.", e.what());
    }
    catch (serial::PortNotOpenedException& e)
    {
        serial_port_ = std::make_shared<serial::Serial>();
        RCLCPP_INFO(this->get_logger(),
                    "Failed to open serial port! Reason: %s.", e.what());
    }

    if (serial_port_)
    {
        serial_port_->setTimeout(10, 1, 0, 1, 0);
        connected_ = serial_port_->isOpen();
        RCLCPP_INFO(this->get_logger(),
                    "Serial port info\nPort: %s\nBaud Rate: %ld", port_.c_str(),
                    baud_);
        RCLCPP_INFO(this->get_logger(), "Serial port connected? %d",
                    connected_);
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto robot_serial_node = std::make_shared<RobotSerial>();
    rclcpp::spin(robot_serial_node);

    // FeedbackMessage message;
    rclcpp::shutdown();

    return 0;
}
