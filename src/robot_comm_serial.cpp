#include "robot_comm_serial.hpp"
#include "cobs.h"
#include <cstring>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <sstream>

RobotSerial::RobotSerial()
    : Node("RobotSerial"), baud_(1000000), port_("/dev/ttyACM0"),
      connected_(false)
{

    std::string share_dir =
        ament_index_cpp::get_package_share_directory("robot_comm_serial");
    std::string config_file = share_dir + "/config/robot_serial.yaml";
    RCLCPP_INFO(this->get_logger(), "Loading Config File: %s",
                config_file.c_str());

    YAML::Node config = YAML::LoadFile(config_file);
    port_ = yaml_get_value<std::string>(config, "serial_port");
    baud_ = yaml_get_value<unsigned long>(config, "baud_rate");
    int reception_freq = yaml_get_value<int>(config, "reception_frequency");
    int reconnection_freq =
        yaml_get_value<int>(config, "reconnection_frequency");
    int command_freq = yaml_get_value<int>(config, "command_frequency");

    RCLCPP_INFO(this->get_logger(), "Config File OK!");
    RCLCPP_INFO(this->get_logger(), "Serial Port: %s", port_.c_str());
    RCLCPP_INFO(this->get_logger(), "Baud Rate: %ld", baud_);
    RCLCPP_INFO(this->get_logger(), "Packet Reception Frequency: %d Hz",
                reception_freq);
    RCLCPP_INFO(this->get_logger(), "Reconnection Frequency: %d Hz",
                reconnection_freq);
    RCLCPP_INFO(this->get_logger(), "Command Update Frequency: %d Hz",
                command_freq);
    // Create a QoS profile for best effort reliability
    rclcpp::QoS best_effort_qos(10); // History depth of 10
    best_effort_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    robot_flags_pub_ = this->create_publisher<sd_msgs::msg::RobotFlags>(
        "robot_base/flags", best_effort_qos);
    imu_pub_ =
        this->create_publisher<sensor_msgs::msg::Imu>("robot_base/imu", best_effort_qos);
    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "robot_base/odometry", best_effort_qos);
    encoder_pub_ = this->create_publisher<sd_msgs::msg::RobotEncoders>(
        "robot_base/encoders", best_effort_qos);
    bumpers_pub_ = this->create_publisher<sd_msgs::msg::RobotBumpers>(
        "robot_base/bumpers", best_effort_qos);
    debug_pub_ = this->create_publisher<sd_msgs::msg::RobotDebug>(
        "robot_base/debug", best_effort_qos);
    base_params_pub_ = this->create_publisher<sd_msgs::msg::BaseParams>(
        "robot_base/params", best_effort_qos);

    power_status_pub_ = this->create_publisher<sd_msgs::msg::PowerStatus>(
        "robot_base/power", best_effort_qos);
    battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
        "robot_base/battery", best_effort_qos);

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", best_effort_qos, // QoS History Depth
        std::bind(&RobotSerial::cmd_vel_callback, this, std::placeholders::_1));

    jump_to_boot_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "robot_action/jump_to_boot", best_effort_qos, // QoS History Depth
        std::bind(&RobotSerial::jump_to_boot_callback, this,
                  std::placeholders::_1));

    connect();

    packet_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1e3 / reception_freq)),
        std::bind(&RobotSerial::packet_callback, this));
    reconnect_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1e3 / reconnection_freq)),
        std::bind(&RobotSerial::reconnect_callback, this));
    command_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1e3 / command_freq)),
        std::bind(&RobotSerial::command_callback, this));

    last_message_ = FeedbackMessage();
    VelocityCommand* velocity = last_command_.mutable_velocities();
    velocity->set_angular(0);
    velocity->set_linear(0);

    last_message_ok_ = false;

    current_buffer_pos_ = 0;
    last_packet_time_ = high_resolution_clock::now();
    packet_frequency_.resize(MAX_FREQUENCY_SAMPLES + 1);
}

RobotSerial::~RobotSerial()
{
}

void RobotSerial::cmd_vel_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg)
{
    VelocityCommand* velocity = last_command_.mutable_velocities();
    velocity->set_linear(msg->linear.x * 1000);
    velocity->set_angular(msg->angular.z * 1000);
}

void RobotSerial::jump_to_boot_callback(
    const std_msgs::msg::Bool::SharedPtr msg)
{
    RobotActions* action = last_command_.mutable_actions();
    RCLCPP_INFO(this->get_logger(), "[Robot Action] Jump to boot? %d",
                msg->data);
    action->set_jump_to_boot(msg->data);
}

void RobotSerial::packet_callback()
{
    auto t_begin = high_resolution_clock::now(), t_read = high_resolution_clock::now(),
         t_decode = high_resolution_clock::now(), t_publish = high_resolution_clock::now();
    if (!connected_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Serial port disconnected!");
        return;
    }

    t_read = high_resolution_clock::now();
    this->try_serial_operation([&]() {
        size_t available_data = serial_port_->available();

        if (available_data >= MAX_PACKET_SIZE)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Too much data in serial buffer (%ld > %ld)!",
                        available_data, MAX_PACKET_SIZE);
        }
        if (available_data > 0 && current_buffer_pos_ == 0)
        {
            RCLCPP_DEBUG(this->get_logger(), "Data available! %ld",
                         available_data);
            packet_size_ = 0;
            memset(buffer_, 0xFF, MAX_PACKET_SIZE);
        }

        size_t aux =
            serial_port_->read(&buffer_[current_buffer_pos_],
                               std::min(serial_port_->available(),
                                        MAX_PACKET_SIZE - current_buffer_pos_));

        if (current_buffer_pos_ == 0)
        {
            packet_size_ = aux;
        }
        else
        {
            packet_size_ += aux;
        }
        current_buffer_pos_ = aux;
    });
    auto dt_read = high_resolution_clock::now() - t_read ;

    t_decode = high_resolution_clock::now();
    decode_buffer();
    auto dt_decode = high_resolution_clock::now() - t_decode;

    t_publish = high_resolution_clock::now();
    publish_data();
    auto dt_publish = high_resolution_clock::now() - t_publish;

    auto t_end = high_resolution_clock::now();

    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Feedback Rate: %.2f Hz | Processing Time = %.3f (%.2f, %.2f, %.2f) us",
        packet_frequency(),
        duration_cast<nanoseconds>(t_end - t_begin).count() / 1e3,
        duration_cast<nanoseconds>(dt_read).count() / 1e3,
        duration_cast<nanoseconds>(dt_decode).count() / 1e3,
        duration_cast<nanoseconds>(dt_publish).count() / 1e3);
}

void RobotSerial::reconnect_callback()
{
    if (!connected_)
    {
        RCLCPP_INFO(this->get_logger(), "Trying to reconnect...");
        packet_frequency_.clear();
        last_packet_time_ = high_resolution_clock::now();
        connect();
    }
}

void RobotSerial::command_callback()
{
    if (!connected_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Serial port disconnected!");
        return;
    }

    bool ok = last_command_.SerializeToArray(encoded_packet_, MAX_PACKET_SIZE);
    size_t message_sz = last_command_.ByteSizeLong();

    if (ok)
    {
        CRC_t crc_value =
            crcFast(encoded_packet_, last_command_.ByteSizeLong());
        encoded_packet_[message_sz++] = crc_value;

        cobs_encode_result encode_result =
            cobs_encode(output_buffer_, sizeof(output_buffer_), encoded_packet_,
                        message_sz);

        output_buffer_[encode_result.out_len++] = 0x00;

        if (encode_result.status == COBS_ENCODE_OK)
        {
            this->try_serial_operation([&]() {
                size_t bytes_written =
                    serial_port_->write(output_buffer_, encode_result.out_len);

                if (bytes_written == encode_result.out_len)
                {
                    RCLCPP_DEBUG(this->get_logger(), "Packet Sent!");
                    clear_command();
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Failed to send packet!");
                }
            });
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed encode packet (COBS)");
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed encode packet (Protobuf)");
    }
}

void RobotSerial::clear_command()
{
    if (last_command_.has_actions())
    {
        last_command_.clear_actions();
    }
}

void RobotSerial::connect()
{
    try_serial_operation([&]() {
        serial_port_ = std::make_shared<serial::Serial>(
            port_, baud_, serial::Timeout::simpleTimeout(10));
    });

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

void RobotSerial::decode_buffer()
{
    int end_byte_count = 0, packet_count = 0;
    size_t packet_start = 0, packet_sz = 0;
    std::vector<size_t> end_pos;

    for (size_t n = 0; n < packet_size_; n++)
    {
        if (buffer_[n] == 0x00)
        {
            end_byte_count++;
            end_pos.push_back(n);
        }
    }
    RCLCPP_DEBUG(this->get_logger(), "Packets in buffer: %d", end_byte_count);

    if (end_byte_count > 0)
    {
        update_packet_frequency();
        current_buffer_pos_ = 0;
    }
    if (end_byte_count == 0 && packet_size_ > 0)
    {
        RCLCPP_DEBUG(this->get_logger(), "Wait for more bytes...");
    }

    while (end_byte_count > 0)
    {
        memset(decoded_packet_, 0x00, MAX_PACKET_SIZE);
        packet_sz = end_pos.at(packet_count) - packet_start;

        cobs_decode_result decode_result =
            cobs_decode(decoded_packet_, MAX_PACKET_SIZE,
                        &buffer_[packet_start], packet_sz);

        if (decode_result.status != COBS_DECODE_OK)
        {
            RCLCPP_ERROR(
                this->get_logger(), "Failed to decode COBS packet: %d. %s",
                decode_result.status,
                packet_to_str(&buffer_[packet_start], packet_sz).c_str());
        }
        else if (decode_result.out_len == 0)
        {
            RCLCPP_WARN(
                this->get_logger(), "Null Packet: %s",
                packet_to_str(decoded_packet_, decode_result.out_len).c_str());
        }
        else
        {
            RCLCPP_DEBUG(
                this->get_logger(), "Decoded COBS packet: %d. %s",
                decode_result.status,
                packet_to_str(&buffer_[packet_start], packet_sz).c_str());

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
                    RCLCPP_DEBUG(this->get_logger(),
                                 "Received protobuf message:\n %s",
                                 message.DebugString().c_str());

                    last_message_ = message;
                    last_message_ok_ = true;
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Failed to parse Protobuf message");
                }
            }
        }

        packet_start = end_pos.at(packet_count) + 1;
        packet_count++;
        end_byte_count--;
    }
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
    if (packet_frequency_.empty())
    {
        return 0;
    }
    return std::accumulate(packet_frequency_.begin(), packet_frequency_.end(),
                           0.0) /
           packet_frequency_.size();
}

std::string RobotSerial::packet_to_str(uint8_t const* _buffer, size_t _buffLen)
{
    std::ostringstream aux;
    aux << "Data = [";
    for (size_t n = 0; n < _buffLen; ++n)
    {
        unsigned int num = _buffer[n];
        aux << std::hex << num << std::dec << ", ";
    }
    aux << "]\n";
    return aux.str();
}

void RobotSerial::publish_data()
{
    if (!last_message_ok_)
    {
        return;
    }

    publish_flags();
    publish_imu();
    publish_odometry();
    publish_encoders();
    publish_bumpers();
    publish_debug();
    publish_base_params();
    publish_power_status();
    publish_battery();

    if (last_message_.has_last_command_ok())
    {
        RCLCPP_INFO(this->get_logger(), "Last Command ok: %d",
                    last_message_.last_command_ok());
    }

    last_message_ok_ = false;
}

void RobotSerial::publish_flags()
{
    sd_msgs::msg::RobotFlags msg;
    msg.emergency_button_status = last_message_.emergency_button_pressed();
    msg.colision_detected = last_message_.colision_detected();
    msg.motion_detection = last_message_.motion_detection();
    robot_flags_pub_->publish(msg);
}

void RobotSerial::publish_imu()
{
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "imu";

    msg.linear_acceleration.x = last_message_.imu().acc().x() / 1000.f;
    msg.linear_acceleration.y = last_message_.imu().acc().y() / 1000.f;
    msg.linear_acceleration.z = last_message_.imu().acc().z() / 1000.f;
    msg.angular_velocity.x = last_message_.imu().gyro().x() / 1000.f;
    msg.angular_velocity.y = last_message_.imu().gyro().y() / 1000.f;
    msg.angular_velocity.z = last_message_.imu().gyro().z() / 1000.f;

    // RCLCPP_INFO_THROTTLE(
    //     this->get_logger(), *this->get_clock(), 100,
    //     "Robot Feedback IMU| X: %d | Y: %d | Theta: %d",
    //     last_message_.imu().acc().x(),
    //     last_message_.imu().acc().y(),
    //     last_message_.imu().gyro().z());

    imu_pub_->publish(msg);
}

void RobotSerial::publish_odometry()
{
    nav_msgs::msg::Odometry msg;

    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_footprint";

    msg.pose.pose.position.x = last_message_.pose().x_mm() / 1000.0;
    msg.pose.pose.position.y = last_message_.pose().y_mm() / 1000.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, last_message_.pose().yaw_trad() / 1000.0);
    msg.pose.pose.orientation = tf2::toMsg(q);
    // msg.pose.covariance = last_message_.pose().covariance();

    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    //     "Received covariance sizes: POSE=%d, TWIST=%d",
    //     last_message_.pose().covariance().size(),
    //     last_message_.velocities().covariance().size());

    const auto& pose_cov_proto = last_message_.pose().covariance();
    if (pose_cov_proto.size() == 36)
    {
        std::copy(pose_cov_proto.begin(), pose_cov_proto.end(),
                  msg.pose.covariance.begin());
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100,
        // "Size of pose if correct!");
    }

    msg.twist.twist.linear.x =
        last_message_.velocities().linear_mm_s() / 1000.0;
    msg.twist.twist.angular.z =
        last_message_.velocities().angular_trad_s() / 1000.0;
    // msg.twist.covariance = last_message_.velocities().covariance();
    const auto& twist_cov_proto = last_message_.velocities().covariance();
    if (twist_cov_proto.size() == 36)
    {
        std::copy(twist_cov_proto.begin(), twist_cov_proto.end(),
                  msg.twist.covariance.begin());
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100,
        // "Size of velocity if correct!");
    }
    odometry_pub_->publish(msg);

    // RCLCPP_INFO_THROTTLE(
    //     this->get_logger(), *this->get_clock(), 100,
    //     "Robot Feedback Pose | X: %ld | Y: %ld | Theta: %d | V_x: %d | V_w:
    //     %d" , last_message_.pose().x_mm(), last_message_.pose().y_mm(),
    //     last_message_.pose().yaw_trad(),
    //     last_message_.velocities().linear_mm_s(),
    //     last_message_.velocities().angular_trad_s());
}

void RobotSerial::publish_encoders()
{
    sd_msgs::msg::Encoder data;
    sd_msgs::msg::RobotEncoders msg;

    // RCLCPP_INFO_THROTTLE(
    //     this->get_logger(), *this->get_clock(), 100,
    //     "Robot Feedback Pose | X: %ld | Y: %ld | Theta: %d | V_x: %d | V_w:
    //     %d" , last_message_.encoder().pose_enc().x_mm(),
    //     last_message_.encoder().pose_enc().y_mm(),
    //     last_message_.encoder().pose_enc().yaw_trad(),
    //     last_message_.encoder().twist_enc().linear_mm_s(),
    //     last_message_.encoder().twist_enc().angular_trad_s());

    msg.pose.pose.position.x =
        last_message_.encoder().pose_enc().x_mm() / 1000.0;
    msg.pose.pose.position.y =
        last_message_.encoder().pose_enc().y_mm() / 1000.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, last_message_.encoder().pose_enc().yaw_trad() / 1000.0);
    msg.pose.pose.orientation = tf2::toMsg(q);

    msg.yaw_radians = last_message_.encoder().pose_enc().yaw_trad() / 1000.0;

    msg.twist.twist.linear.x =
        last_message_.encoder().twist_enc().linear_mm_s() / 1000.0;
    msg.twist.twist.angular.z =
        last_message_.encoder().twist_enc().angular_trad_s() / 1000.0;

    // Encoder - left
    data.velocity = last_message_.velocities().left_wheel_mm_s() / 1000.0;
    data.count = last_message_.encoder().left();
    if (last_message_.encoder().has_left_status())
    {
        last_message_.encoder().left_status().magnitude();
        last_message_.encoder().left_status().gain();
        last_message_.encoder().left_status().connected();
        last_message_.encoder().left_status().magnet_detected();
        last_message_.encoder().left_status().magnet_weak();
        last_message_.encoder().left_status().magnet_strong();
        data.ppr = last_message_.encoder().left_status().ppr();
    }
    msg.left = data;

    // Encoder - right
    data.velocity = last_message_.velocities().right_wheel_mm_s() / 1000.0;
    data.count = last_message_.encoder().right();
    if (last_message_.encoder().has_right_status())
    {
        last_message_.encoder().right_status().magnitude();
        last_message_.encoder().right_status().gain();
        last_message_.encoder().right_status().connected();
        last_message_.encoder().right_status().magnet_detected();
        last_message_.encoder().right_status().magnet_weak();
        last_message_.encoder().right_status().magnet_strong();
        data.ppr = last_message_.encoder().right_status().ppr();
    }
    msg.right = data;
    encoder_pub_->publish(msg);
}

void RobotSerial::publish_bumpers()
{
    sd_msgs::msg::Bumper data;
    sd_msgs::msg::RobotBumpers msg;

    for (int n = 0; n < last_message_.bumpers_size(); n++)
    {
        data.status = last_message_.bumpers(n).status();
        data.bumper_id = last_message_.bumpers(n).id();
        msg.bumper_data.push_back(data);
    }
    bumpers_pub_->publish(msg);
}

void RobotSerial::publish_debug()
{
    sd_msgs::msg::RobotDebug msg;
    if (last_message_.has_info() && last_message_.info().has_debug_data())
    {
        msg.ecu_debug_info = last_message_.info().debug_data();
        debug_pub_->publish(msg);
    }
}

void RobotSerial::publish_base_params()
{
    sd_msgs::msg::BaseParams msg;
    sd_msgs::msg::BuildInfo data;

    if (last_message_.has_info())
    {
        if (last_message_.info().has_build_data())
        {
            data.commit_hash = last_message_.info().build_data().commit_hash();
            data.branch_name = last_message_.info().build_data().branch_name();
            data.tag = last_message_.info().build_data().tag();
            data.build_date = last_message_.info().build_data().build_date();

            msg.build_data = data;
        }

        // msg.robot_name = last_message_.info().hardware_info().ecu_version();
        // msg.robot_name =
        // last_message_.info().hardware_info().driver_version();
        // msg.robot_name =
        // last_message_.info().hardware_info().motor_version();
        msg.robot_name = last_message_.info().hardware_info().robot_name();
        msg.wheel_distance = last_message_.info().wheel_distance_mm();
        msg.wheel_diameter = last_message_.info().wheel_diameter_mm();
    }

    base_params_pub_->publish(msg);
}

void RobotSerial::publish_battery()
{
    sensor_msgs::msg::BatteryState msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "battery";

    msg.voltage = last_message_.power_status().voltage_mv() / 1000.f;
    msg.current = 0.0;
    msg.percentage = last_message_.power_status().battery_percent();
    msg.capacity = last_message_.power_status().battery_capacity();
    msg.design_capacity = last_message_.power_status().battery_capacity();
    msg.power_supply_technology =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;

    msg.power_supply_status =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    if (last_message_.power_status().charging())
    {
        msg.power_supply_status =
            sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
        msg.current =
            last_message_.power_status().charging_current_ma() / 1000.f;
    }

    msg.power_supply_health =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    msg.present = true;
    battery_pub_->publish(msg);
}

void RobotSerial::publish_power_status()
{
    sd_msgs::msg::PowerStatus msg;
    msg.charging_current =
        last_message_.power_status().charging_current_ma() / 1000.f;
    msg.driver_current =
        last_message_.power_status().driver_current_ma() / 1000.f;
    msg.charger_detected = last_message_.power_status().charger_detected();

    msg.ecu_temp.temperature = last_message_.power_status().temperature();
    msg.ecu_temp.variance = 0;
    msg.ecu_temp.header.stamp = this->get_clock()->now();
    msg.ecu_temp.header.frame_id = "ecu";

    msg.internal_temp.temperature =
        last_message_.power_status().internal_temperature();
    msg.internal_temp.variance = 0;
    msg.internal_temp.header.stamp = this->get_clock()->now();
    msg.internal_temp.header.frame_id = "mcu";

    power_status_pub_->publish(msg);
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
