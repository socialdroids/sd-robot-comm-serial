#include "robot_comm_serial.hpp"
#include "cobs.h"
#include <cstring>
#include <rclcpp/logging.hpp>
#include <sstream>

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

    packet_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(2),
                                std::bind(&RobotSerial::packet_callback, this));
    reconnect_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&RobotSerial::reconnect_callback, this));

    last_packet_time_ = high_resolution_clock::now();
    packet_frequency_.resize(MAX_FREQUENCY_SAMPLES + 1);
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
    auto t_begin = high_resolution_clock::now();
    if (!connected_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Serial port disconnected!");
        return;
    }

    try
    {
        size_t available_data = serial_port_->available();

        if (available_data >= MAX_PACKET_SIZE)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Too much data in serial buffer (%ld > %ld)!",
                        available_data, MAX_PACKET_SIZE);
        }
        if (available_data > 0)
        {
            RCLCPP_DEBUG(this->get_logger(), "Data available! %ld",
                         available_data);
            packet_size_ = 0;
            memset(buffer_, 0xFF, MAX_PACKET_SIZE);
        }

        packet_size_ = serial_port_->read(
            buffer_, std::min(serial_port_->available(), MAX_PACKET_SIZE));
    }
    catch (serial::PortNotOpenedException& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to read data! Reason: %s.",
                     e.what());
        connected_ = false;
        return;
    }
    catch (serial::SerialException& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to read data! Reason: %s.",
                     e.what());
        connected_ = false;
        return;
    }
    catch (serial::IOException& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to read data! Reason: %s.",
                     e.what());
        connected_ = false;
        return;
    }

    decode_buffer();
    publish_data();
    auto t_end = high_resolution_clock::now();

    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Robot Feedback Data Rate: %.2f Hz | Processing Time = %.3f us",
        packet_frequency(),
        duration_cast<nanoseconds>(t_end - t_begin).count() / 1e3);
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
                    "Serial port info: {Port: %s, Baud Rate: %ld}",
                    port_.c_str(), baud_);
        RCLCPP_INFO(this->get_logger(), "Serial port connected? %d",
                    connected_);
    }
}

bool RobotSerial::decode_buffer()
{
    int end_byte_count = 0, packet_count = 0;
    size_t packet_start = 0, packet_end = 0;
    std::vector<size_t> end_pos;

    for (size_t n = 0; n < packet_size_; n++)
    {
        if (buffer_[n] == 0x00)
        {
            end_byte_count++;
            end_pos.push_back(n);
        }
    }
    if (end_byte_count)
        update_packet_frequency();

    while (end_byte_count > 0)
    {
        memset(decoded_packet_, 0x00, MAX_PACKET_SIZE);
        packet_end = end_pos.at(packet_count) - packet_start;

        cobs_decode_result decode_result =
            cobs_decode(decoded_packet_, MAX_PACKET_SIZE,
                        &buffer_[packet_start], packet_end);

        if (decode_result.status != COBS_DECODE_OK)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to decode COBS packet: %d. %s",
                         decode_result.status,
                         packet_to_str(&buffer_[packet_start], packet_end));
        }
        else if (decode_result.out_len == 0)
        {
            RCLCPP_WARN(this->get_logger(), "Null Packet: %s",
                        packet_to_str(decoded_packet_, decode_result.out_len));
        }
        else
        {
            CRC_t crc_check = crcFast(decoded_packet_, decode_result.out_len);
            if (crc_check != CRC_OK)
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid CRC Value");
            }
            else
            {
                FeedbackMessage message;

                // Exclui o valor do CRC
                bool ok = message.ParseFromArray(
                    decoded_packet_, decode_result.out_len - sizeof(CRC_t));
                if (ok)
                {
                    RCLCPP_INFO(this->get_logger(),
                                "Received protobuf message:\n %s",
                                message.DebugString().c_str());
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Failed to parse Protobuf message");
                    return false;
                }
            }
        }

        packet_start = end_pos.at(packet_count) + 1;
        packet_count++;
        end_byte_count--;
    }

    return true;
}

CRC_t RobotSerial::crcFast(uint8_t const* _message, int _nBytes)
{
    uint8_t data;
    CRC_t remainder = 0;

    // Divide the message by the polynomial, a byte at a time.
    for (int byte = 0; byte < _nBytes; ++byte)
    {
        data = _message[byte] ^ (remainder >> (WIDTH - 8));
        remainder = crcTable[data] ^ (remainder << 8);
    }

    // The final remainder is the CRC.
    return remainder;
}

void RobotSerial::update_packet_frequency()
{
    packet_frequency_.push_front(
        1e9 / (duration_cast<nanoseconds>(high_resolution_clock::now() -
                                          last_packet_time_))
                  .count());
    if (packet_frequency_.size() >= MAX_FREQUENCY_SAMPLES)
    {
        packet_frequency_.pop_back();
    }
    last_packet_time_ = high_resolution_clock::now();
}

float RobotSerial::packet_frequency()
{
    return std::accumulate(packet_frequency_.begin(), packet_frequency_.end(),
                           0.0) /
           packet_frequency_.size();
}

const char* RobotSerial::packet_to_str(uint8_t const* _buffer, size_t _buffLen)
{
    std::ostringstream aux;
    aux << "Data = [";
    for (size_t n = 0; n < _buffLen; ++n)
    {
        unsigned int num = _buffer[n];
        aux << std::hex << num << std::dec << ", ";
    }
    aux << "]\n";
    return aux.str().c_str();
}

void RobotSerial::publish_data()
{
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
