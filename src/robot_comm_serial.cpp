#include "robot_comm_serial.hpp"
#include "cobs.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <string>

#include <SDL2/SDL.h>
#include <SDL2/SDL_mixer.h>

volatile bool finished = true;
Mix_Chunk* sound = nullptr;

void finishedCallback(int _ch)
{
    if (sound != nullptr)
    {
        Mix_FreeChunk(sound);
    }
    finished = true;
    std::cerr << "Music finished!\n";
    // Mix_CloseAudio();
}

int setupAudio()
{
    // Initialize SDL
    if (SDL_Init(SDL_INIT_AUDIO) < 0)
    {
        std::cerr << "SDL could not initialize! SDL Error: " << SDL_GetError()
                  << std::endl;
        return 1;
    }

    // Initialize SDL_mixer
    if (Mix_OpenAudio(44100, MIX_DEFAULT_FORMAT, 2, 2048) < 0)
    {
        std::cerr << "SDL_mixer could not initialize! SDL_mixer Error: "
                  << Mix_GetError() << std::endl;
        // SDL_Quit();
        return 1;
    }
}

int playSound(std::string _path)
{
    std::cout << "Play " << _path << std::endl;
    // Load WAV file
    sound = Mix_LoadWAV(_path.c_str());
    if (sound == nullptr)
    {
        std::cerr << "Failed to load sound effect! SDL_mixer Error: "
                  << Mix_GetError() << std::endl;
    }
    else
    {
        // Play the sound (channel -1 finds the first available channel, 0 loops
        // once, etc.)
        Mix_ChannelFinished(finishedCallback);
        Mix_PlayChannel(-1, sound, 0);

        // Wait while the sound is playing (simple wait, a real application
        // would use a loop/event system) SDL_Delay(2000);

        // Free the sound effect
        // Mix_FreeChunk(sound);
    }
    return 0;
}

RobotSerial::RobotSerial()
    : Node("RobotSerial"), baud_(1000000), port_("/dev/ttyACM0"),
      connected_(false)
{

    std::string share_dir =
        ament_index_cpp::get_package_share_directory("robot_comm_serial");
    std::string config_file = share_dir + "/config/robot_serial.yaml";
    RCLCPP_INFO(this->get_logger(), "Loading Config File: %s",
                config_file.c_str());

    // === CONFIGURAÇÕES
    YAML::Node config = YAML::LoadFile(config_file);
    port_ = yaml_get_value<std::string>(config, "serial_port");
    baud_ = yaml_get_value<unsigned long>(config, "baud_rate");
    int reception_freq = yaml_get_value<int>(config, "reception_frequency");
    int reconnection_freq =
        yaml_get_value<int>(config, "reconnection_frequency");
    int command_freq = yaml_get_value<int>(config, "command_frequency");
    fake_charging_ = yaml_get_value<bool>(config, "fake_charging");
    fake_charging_radius_ =
        yaml_get_value<double>(config, "fake_charging_radius");
    fake_charging_fail_count_ = 0;

    MAX_RTOS_TASKS = yaml_get_value<unsigned long>(config, "max_rtos_tasks");

    RCLCPP_INFO(this->get_logger(), "Config File OK!");
    RCLCPP_INFO(this->get_logger(), "Serial Port: %s", port_.c_str());
    RCLCPP_INFO(this->get_logger(), "Baud Rate: %ld", baud_);
    RCLCPP_INFO(this->get_logger(), "Packet Reception Frequency: %d Hz",
                reception_freq);
    RCLCPP_INFO(this->get_logger(), "Reconnection Frequency: %d Hz",
                reconnection_freq);
    RCLCPP_INFO(this->get_logger(), "Command Update Frequency: %d Hz",
                command_freq);
    RCLCPP_INFO(this->get_logger(), "Use Fake Charging: %d", fake_charging_);
    RCLCPP_INFO(this->get_logger(), "Max RTOS Tasks: %ld", MAX_RTOS_TASKS);

    // === TÓPICOS
    // Create a QoS profile for best effort reliability
    rclcpp::QoS best_effort_qos(10); // History depth of 10
    best_effort_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    robot_flags_pub_ = this->create_publisher<sd_msgs::msg::RobotFlags>(
        "robot_base/flags", best_effort_qos);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("robot_base/imu",
                                                             best_effort_qos);
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

    setupAudio();
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

    const int gui_frequency = 30;
    gui_update_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(1e3 / gui_frequency)),
        std::bind(&RobotSerial::publish_full_status, this));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    last_message_ = FeedbackMessage();
    VelocityCommand* velocity = last_command_.mutable_velocities();
    velocity->set_angular(0);
    velocity->set_linear(0);

    last_message_ok_ = false;

    current_buffer_pos_ = 0;
    last_packet_time_ = high_resolution_clock::now();
    packet_frequency_.resize(MAX_FREQUENCY_SAMPLES + 1);

    // === WEBSOCKET
    std::string docroot = "";

    // Se o caminho não foi fornecido, tenta encontrar automaticamente
    if (docroot.empty())
    {
        try
        {
            docroot = ament_index_cpp::get_package_share_directory(
                          "robot_comm_serial") +
                      "/web";
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Falha ao encontrar o docroot. Especifique "
                         "'docroot_path'. Erro: %s",
                         e.what());
            rclcpp::shutdown();
            return;
        }
    }
    RCLCPP_INFO(this->get_logger(), "Servindo arquivos da web de: %s",
                docroot.c_str());
    ws_interface_ = std::make_unique<WebsocketInterface>(9002, docroot);

    ws_interface_->register_command_callback(
        std::bind(&RobotSerial::handle_gui_command, this, _1, _2));
    ws_interface_->run();
}

RobotSerial::~RobotSerial()
{
    ws_interface_->stop();
}

void RobotSerial::cmd_vel_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg)
{
    VelocityCommand* velocity = last_command_.mutable_velocities();
    velocity->set_linear(msg->linear.x * 1000);
    velocity->set_angular(msg->angular.z * 1000);
    last_cmd_vel_time_ = high_resolution_clock::now();
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
    auto t_begin = high_resolution_clock::now(),
         t_read = high_resolution_clock::now(),
         t_decode = high_resolution_clock::now(),
         t_publish = high_resolution_clock::now();
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
    auto dt_read = high_resolution_clock::now() - t_read;

    t_decode = high_resolution_clock::now();
    decode_buffer();
    auto dt_decode = high_resolution_clock::now() - t_decode;

    t_publish = high_resolution_clock::now();
    publish_data();
    auto dt_publish = high_resolution_clock::now() - t_publish;

    auto t_end = high_resolution_clock::now();

    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
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
    if (!nav_tester_)
    {
        nav_tester_ = std::make_unique<NavigationTester>(
            shared_from_this(),
            ament_index_cpp::get_package_share_directory("robot_comm_serial"));
    }

    if (!connected_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Serial port disconnected!");
        return;
    }

    // Não está recebendo comando de nenhum lugar, garante que o robô está
    // recebendo 0
    if (timeSince<milliseconds>(last_virtual_cmd_time_) > 500 &&
        timeSince<milliseconds>(last_cmd_vel_time_) > 2e3)
    {
        VelocityCommand* velocity = last_command_.mutable_velocities();
        velocity->set_linear(0);
        velocity->set_angular(0);
    }

    bool ok = false;
    size_t message_sz = 0;

    try
    {
        ok = last_command_.SerializeToArray(encoded_packet_, MAX_PACKET_SIZE);
        message_sz = last_command_.ByteSizeLong();
    }
    catch (const google::protobuf::FatalException& e)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Falha ao encontrar codificar pacote: %s", e.what());
    }

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

    if (last_command_.has_config())
    {
        last_command_.clear_config();
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
                bool ok = false;
                try
                {
                    ok = message.ParseFromArray(
                        decoded_packet_, decode_result.out_len - sizeof(CRC_t));
                }
                catch (const google::protobuf::FatalException& e)
                {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Falha ao encontrar decodificar pacote: %s",
                                 e.what());
                }

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
    publish_rtos_info();
    publish_base_params();
    publish_power_status();
    publish_battery();
    publish_robot_config();

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
    std::fill(msg.linear_acceleration_covariance.begin(),
              msg.linear_acceleration_covariance.end(), 0.0);
    // 0 1 2
    // 3 4 5
    // 6 7 8
    msg.linear_acceleration_covariance[0] = 1e-2;
    msg.linear_acceleration_covariance[4] = 1e-2;
    msg.linear_acceleration_covariance[8] = 1e-2;

    msg.angular_velocity.x = last_message_.imu().gyro().x() / 1000.f;
    msg.angular_velocity.y = last_message_.imu().gyro().y() / 1000.f;
    msg.angular_velocity.z = last_message_.imu().gyro().z() / 1000.f;
    std::fill(msg.angular_velocity_covariance.begin(),
              msg.angular_velocity_covariance.end(), 0.0);
    msg.angular_velocity_covariance[0] = 1e-3;
    msg.angular_velocity_covariance[4] = 1e-3;
    msg.angular_velocity_covariance[8] = 1e-3;

    tf2::Quaternion q;
    q.setRPY(last_message_.imu().angle().roll(),
             last_message_.imu().angle().pitch(),
             last_message_.imu().angle().yaw());
    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
    msg.orientation.w = q.w();
    std::fill(msg.orientation_covariance.begin(),
              msg.orientation_covariance.end(), 0.0);
    msg.orientation_covariance[0] = 1e-3;
    msg.orientation_covariance[4] = 1e-3;
    msg.orientation_covariance[8] = 1e-3;

    // RCLCPP_INFO_THROTTLE(
    //     this->get_logger(), *this->get_clock(), 100,
    //     "Robot Feedback IMU| aX: %d | aY: %d | vW: %d | T: %.2f | t: %ld",
    //     last_message_.imu().acc().x(),
    //     last_message_.imu().acc().y(),
    //     last_message_.imu().gyro().z(),
    //     last_message_.imu().temperature(),
    //     last_message_.imu().timestamp());

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

    std::fill(msg.pose.covariance.begin(), msg.pose.covariance.end(), 0.0);
    //  0  1  2  3  4  5
    //  6  7  8  9 10 11
    // 12 13 14 15 16 17
    // 18 19 20 21 22 23
    // 24 25 26 27 28 29
    // 30 31 32 33 34 35
    msg.pose.covariance[0] = 1e-3;
    msg.pose.covariance[7] = 1e-3;
    msg.pose.covariance[35] = 1e-3;

    // const auto& pose_cov_proto = last_message_.pose().covariance();
    // if (pose_cov_proto.size() == 36)
    // {
    //     std::copy(pose_cov_proto.begin(), pose_cov_proto.end(),
    //               msg.pose.covariance.begin());
    // }

    msg.twist.twist.linear.x =
        last_message_.velocities().linear_mm_s() / 1000.0;
    msg.twist.twist.angular.z =
        last_message_.velocities().angular_trad_s() / 1000.0;

    std::fill(msg.twist.covariance.begin(), msg.twist.covariance.end(), 0.0);
    msg.twist.covariance[0] = 1e-3;
    msg.twist.covariance[7] = 1e-3;
    msg.twist.covariance[35] = 1e-3;
    // const auto& twist_cov_proto = last_message_.velocities().covariance();
    // if (twist_cov_proto.size() == 36)
    // {
    //     std::copy(twist_cov_proto.begin(), twist_cov_proto.end(),
    //               msg.twist.covariance.begin());
    // }
    odometry_pub_->publish(msg);
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
        // json j;
        // j["type"] = "sensor_update";
        // j["sensor"] = "bumper";
        // j["id"] = data.bumper_id;
        // j["value"] = data.status;
        //
        // ws_interface_->send_robot_status(j);
    }
    bumpers_pub_->publish(msg);
}

void RobotSerial::publish_debug()
{
    sd_msgs::msg::RobotDebug msg;
    if (last_message_.has_info() && last_message_.info().has_debug_data())
    {
        msg.ecu_debug_info = last_message_.info().debug_data();
        ws_interface_->send_log(msg.ecu_debug_info);
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
        msg.wheel_distance = last_message_.info().wheel_distance_mm() / 1000.f;
        msg.wheel_diameter = last_message_.info().wheel_diameter_mm() / 1000.f;

        update_ecu_info(msg.robot_name,
                        last_message_.info().hardware_info().driver_version(),
                        last_message_.info().hardware_info().ecu_version(),
                        last_message_.info().hardware_info().motor_version(),
                        msg.build_data.commit_hash, msg.build_data.branch_name,
                        msg.build_data.tag, msg.build_data.build_date,
                        msg.wheel_distance, msg.wheel_diameter);

        if (nav_tester_)
        {
            update_nav_info();
        }
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
    if (last_message_.power_status().charging() || fake_charging_status())
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

void RobotSerial::publish_robot_config()
{
    if (last_message_.has_parameters())
    {
        last_params_ = last_message_.parameters();
        RCLCPP_DEBUG(
            this->get_logger(), "LEFT (P=%.2f, I=%.2f, D=%.2f, EN=%d)",
            last_params_.left_pid().k_p(), last_params_.left_pid().k_i(),
            last_params_.left_pid().k_d(), last_params_.left_pid().enabled());
        RCLCPP_DEBUG(
            this->get_logger(), "RIGHT (P=%.2f, I=%.2f, D=%.2f, EN=%d)",
            last_params_.right_pid().k_p(), last_params_.right_pid().k_i(),
            last_params_.right_pid().k_d(), last_params_.right_pid().enabled());
        RCLCPP_DEBUG(
            this->get_logger(), "LINEAR (P=%.2f, I=%.2f, D=%.2f, EN=%d)",
            last_params_.linear_pid().k_p(), last_params_.linear_pid().k_i(),
            last_params_.linear_pid().k_d(),
            last_params_.linear_pid().enabled());
        RCLCPP_DEBUG(
            this->get_logger(), "ANGULAR (P=%.2f, I=%.2f, D=%.2f, EN=%d)\n",
            last_params_.angular_pid().k_p(), last_params_.angular_pid().k_i(),
            last_params_.angular_pid().k_d(),
            last_params_.angular_pid().enabled());
        update_config_info();
    }
}

void RobotSerial::publish_rtos_info()
{
    if (last_message_.has_info() && last_message_.info().has_rtos_tasks())
    {
        std::string data;
        char aux[100];

        char state[6][10] = {
            "RUNNING",   // A task is querying the state of itself, so must be
                         // running.
            "READY",     // The task being queried is in a read or pending ready
                         // list.
            "BLOCKED",   // The task being queried is in the Blocked state.
            "SUSPENDED", // The task being queried is in the Suspended state, or
                         // is in the Blocked state with an infinite time out.
            "DELETED",   // The task being queried has been deleted, but its TCB
                         // has not yet been freed.
            "INVALID"    // Used as an 'invalid state' value.
        };

        snprintf(aux, sizeof(aux), "\n%s) %-18s:%-10s %s %s\n", "ID", "Name",
                 "State", "Usage(%)", "StackFree");
        data = aux;

        struct Task_t
        {
            std::string name;
            float cpu;
            int stack;
            std::string state;
        };
        Task_t task;
        std::vector<Task_t> tasks;

        for (size_t n = 0;
             n < last_message_.info().rtos_tasks().task_info_size(); n++)
        {
            task.name = last_message_.info().rtos_tasks().task_info(n).name();
            task.cpu = last_message_.info().rtos_tasks().task_info(n).usage();
            task.stack =
                last_message_.info().rtos_tasks().task_info(n).stack_free();
            task.state = std::string(
                state[last_message_.info().rtos_tasks().task_info(n).state()]);

            snprintf(aux, sizeof(aux), "%2d) %-18s:%-10s %3.2f%%  %4i\n",
                     last_message_.info().rtos_tasks().task_info(n).id(),
                     task.name.c_str(), task.state.c_str(), task.cpu,
                     task.stack); // The minimum amount of stack space that
                                  // has remained for the task since the task
                                  // was created.  The closer this value is to
                                  // zero the closer the task has come to
                                  // overflowing its stack.

            tasks.push_back(task);

            data += aux;
        }
        std::sort(
            tasks.begin(), tasks.end(),
            [](const Task_t& a, const Task_t& b) { return a.cpu > b.cpu; });

        json rtos_tasks, json_task;
        rtos_tasks["type"] = "rtos_tasks";
        rtos_tasks["info"] = json::array();
        for (size_t n = 0; n < MAX_RTOS_TASKS; n++)
        {
            json_task["name"] = tasks.at(n).name.c_str();
            json_task["cpu"] = tasks.at(n).cpu;
            json_task["stack_free"] = tasks.at(n).stack;
            json_task["state"] = tasks.at(n).state.c_str();

            rtos_tasks["info"].push_back(json_task);
        }

        RCLCPP_DEBUG(this->get_logger(), "%s", data.c_str());
        last_message_.mutable_info()->clear_rtos_tasks();
        ws_interface_->send_robot_status(rtos_tasks);
    }
}

bool RobotSerial::fake_charging_status()
{
    if (!fake_charging_)
    {
        return false;
    }
    if (fake_charging_fail_count_ > 200)
    {
        RCLCPP_ERROR_ONCE(this->get_logger(),
                          "Stopping fake charging behavior due to unpublished "
                          "base_link to map transform");
        return false;
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 120000,
                         "Using fake charging!");

    geometry_msgs::msg::TransformStamped transform_stamped;
    try
    {
        // tf2::TimePointZero for latest available transform
        transform_stamped =
            tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    }
    catch (tf2::TransformException& ex)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                             "%d) Could not transform 'base_link' to 'map'! %s",
                             fake_charging_fail_count_++, ex.what());

        return false;
    }

    // Extract position and orientation from the transform
    double x = transform_stamped.transform.translation.x;
    double y = transform_stamped.transform.translation.y;
    double z = transform_stamped.transform.translation.z;

    // double qx = transform_stamped.transform.rotation.x;
    // double qy = transform_stamped.transform.rotation.y;
    // double qz = transform_stamped.transform.rotation.z;
    // double qw = transform_stamped.transform.rotation.w;

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Robot position in map frame: x=%.2f, y=%.2f, z=%.2f",
                          x, y, z);

    if (sqrt(x * x + y * y) < fake_charging_radius_)
    {
        return true;
    }

    return false;
}

void RobotSerial::handle_gui_command(const std::string& type, const json& data)
{
    RCLCPP_DEBUG(this->get_logger(), "Comando recebido da GUI: %s",
                 type.c_str());

    // --- Responde a solicitações GET ---
    if (type == "get_ecu_info")
    {
        update_ecu_info("", "", "", "", "", "", "", "", 0, 0);
    }
    else if (type == "get_all_configs")
    {
        update_config_info();
    }

    // --- Lida com comandos SET ---
    else if (type == "set_pid")
    {
        std::string target = data.at("target");

        double lastP, lastI, lastD, lastEN;

        RobotConfig* config = last_command_.mutable_config();
        RobotParameters* parameters = config->mutable_parameters();
        PIDConfig* cfg = nullptr;

        if (target == "linear")
        {
            cfg = parameters->mutable_linear_pid();
            lastP = last_params_.linear_pid().k_p();
            lastI = last_params_.linear_pid().k_i();
            lastD = last_params_.linear_pid().k_d();
            lastEN = last_params_.linear_pid().enabled();
        }
        else if (target == "angular")
        {
            cfg = parameters->mutable_angular_pid();
            lastP = last_params_.angular_pid().k_p();
            lastI = last_params_.angular_pid().k_i();
            lastD = last_params_.angular_pid().k_d();
            lastEN = last_params_.angular_pid().enabled();
        }
        else if (target == "left")
        {
            cfg = parameters->mutable_left_pid();
            lastP = last_params_.left_pid().k_p();
            lastI = last_params_.left_pid().k_i();
            lastD = last_params_.left_pid().k_d();
            lastEN = last_params_.left_pid().enabled();
        }
        else if (target == "right")
        {
            cfg = parameters->mutable_right_pid();
            lastP = last_params_.right_pid().k_p();
            lastI = last_params_.right_pid().k_i();
            lastD = last_params_.right_pid().k_d();
            lastEN = last_params_.right_pid().enabled();
        }

        if (cfg != nullptr)
        {
            double p = data.at("p").is_null()
                           ? lastP
                           : static_cast<double>(data.at("p"));
            double i = data.at("i").is_null()
                           ? lastI
                           : static_cast<double>(data.at("i"));
            double d = data.at("d").is_null()
                           ? lastD
                           : static_cast<double>(data.at("d"));
            bool enabled = data.at("enabled").is_null()
                               ? lastEN
                               : static_cast<bool>(data.at("enabled"));

            RCLCPP_INFO(this->get_logger(),
                        "Configurando PID %s: P=%.2f I=%.2f D=%.2f EN=%d",
                        target.c_str(), p, i, d, enabled);

            cfg->set_k_p(p);
            cfg->set_k_i(i);
            cfg->set_k_d(d);
            cfg->set_enabled(enabled);
        }
    }
    else if (type == "set_limits")
    {
        double linear_vel = data.at("linear_vel");
        RCLCPP_INFO(this->get_logger(), "Configurando limite linear_vel: %.2f",
                    linear_vel);
        // TODO: Aplicar os limites
    }
    else if (type == "set_kalman_cov")
    {
        // As matrizes estão em data["model"] (5x5) e data["measurement"] (4x4)
        // Você pode acessá-las como: json model_matrix = data.at("model");
        // double val_0_0 = model_matrix[0][0];
        RCLCPP_INFO(this->get_logger(), "Configurando matrizes Kalman.");
        // TODO: Aplicar as matrizes
    }
    else if (type == "set_open_loop")
    {
        bool enabled = data.at("enabled");
        RCLCPP_INFO(this->get_logger(), "Modo Malha Aberta: %s",
                    enabled ? "ON" : "OFF");
        // TODO: Aplicar o modo
    }
    else if (type == "set_velocity_command")
    {
        float linear = data.at("linear"), angular = data.at("angular");
        RCLCPP_DEBUG(this->get_logger(),
                     "Virtual Joystick: (Linear, Angular) = (%.3f, %.3f)",
                     linear, angular);
        if (timeSince<milliseconds>(last_cmd_vel_time_) > 2e3)
        {
            VelocityCommand* velocity = last_command_.mutable_velocities();
            if (linear < 0)
                angular *= -1;
            velocity->set_linear(linear * 1000);
            velocity->set_angular(angular * 1000);
            last_virtual_cmd_time_ = high_resolution_clock::now();
        }
    }
    else if (type == "record_poses")
    {
        bool enabled = data.at("enabled");

        nav_tester_->recordPoses(enabled);
        RCLCPP_INFO(this->get_logger(), "Record Poses: %d", enabled);
    }
    else if (type == "test_nav")
    {
        std::string button = data.at("button");
        if (button == "start")
        {
            nav_tester_->start();
        }
        else if (button == "stop")
        {
            nav_tester_->stop();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Valor desconhecido para test_nav!");
        }
        RCLCPP_INFO(this->get_logger(), "Test Navigation: %s", button.c_str());
    }
}

void RobotSerial::update_ecu_info(
    const std::string& _name, const std::string& _driver_version,
    const std::string& _ecu_version, const std::string& _motor_version,
    const std::string& _git_hash, const std::string& _git_branch,
    const std::string& _git_tag, const std::string& _build_date,
    float _wheel_distance, float _wheel_diameter)
{
    json info;
    info["type"] = "ecu_info";
    info["info"] = {{"robot_name", _name},
                    {"ecu_version", _ecu_version},
                    {"driver_version", _driver_version},
                    {"motor_version", _motor_version},
                    {"wheel_distance", _wheel_distance},
                    {"wheel_diameter", _wheel_diameter},
                    {"git_hash", _git_hash},
                    {"git_branch", _git_branch},
                    {"git_tag", _git_tag},
                    {"build_date", _build_date}};
    ws_interface_->send_robot_status(info); // Reutiliza o método de envio
}

void RobotSerial::update_nav_info()
{
    json info;
    json pose;

    pose["x"] = 0.0;
    pose["y"] = 0.0;
    pose["yaw"] = 0.0;

    info["type"] = "nav_info";
    info["info"] = {{"status", nav_tester_->status()},
                    {"last_pose", pose},
                    {"navigation_status", "ativo"},
                    {"localization_status", "ativo"},
                    {"nav_feedback", nav_tester_->navigation_status()},
                    {"rem_poses", nav_tester_->remaining_poses()},
                    {"eta", nav_tester_->eta()},
                    {"rem_distance", nav_tester_->remaining_distance()},
                    {"total_time", nav_tester_->total_time()},
                    {"recoveries", nav_tester_->recoveries()}};
    ws_interface_->send_robot_status(info); // Reutiliza o método de envio
}

void RobotSerial::update_config_info()
{
    json configs;
    configs["type"] = "current_configs";
    // Preenche com os valores atuais do robô (ex: lendo parâmetros ROS)
    struct
    {
        float p;
        float i;
        float d;
        bool en;
    } linear, angular, left, right;

    left.p = last_params_.left_pid().k_p();
    left.i = last_params_.left_pid().k_i();
    left.d = last_params_.left_pid().k_d();
    left.en = last_params_.left_pid().enabled();

    right.p = last_params_.right_pid().k_p();
    right.i = last_params_.right_pid().k_i();
    right.d = last_params_.right_pid().k_d();
    right.en = last_params_.right_pid().enabled();

    linear.p = last_params_.linear_pid().k_p();
    linear.i = last_params_.linear_pid().k_i();
    linear.d = last_params_.linear_pid().k_d();
    linear.en = last_params_.linear_pid().enabled();

    angular.p = last_params_.angular_pid().k_p();
    angular.i = last_params_.angular_pid().k_i();
    angular.d = last_params_.angular_pid().k_d();
    angular.en = last_params_.angular_pid().enabled();

    configs["configs"] = {
        {"pid",
         {{"linear",
           {{"p", linear.p},
            {"i", linear.i},
            {"d", linear.d},
            {"enabled", linear.en}}},
          {"angular",
           {{"p", angular.p},
            {"i", angular.i},
            {"d", angular.d},
            {"enabled", angular.en}}},
          {"left",
           {{"p", left.p}, {"i", left.i}, {"d", left.d}, {"enabled", left.en}}},
          {"right",
           {{"p", right.p},
            {"i", right.i},
            {"d", right.d},
            {"enabled", right.en}}}}},
        {"limits",
         {{"linear_vel", 1.5},
          {"linear_acc", 0.5},
          {"angular_vel", 1.0},
          {"angular_acc", 0.8}}},
        {"open_loop", false},
        {"kalman",
         {// Matriz 5x5 identidade de exemplo
          {"model",
           {{1, 0, 0, 0, 0},
            {0, 1, 0, 0, 0},
            {0, 0, 1, 0, 0},
            {0, 0, 0, 1, 0},
            {0, 0, 0, 0, 1}}},
          // Matriz 4x4 identidade de exemplo
          {"measurement",
           {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}}}}};
    ws_interface_->send_robot_status(configs);
}

void RobotSerial::publish_full_status()
{
    if (!connected_)
    {
        return;
    }

    {
        const std::string share_dir =
            ament_index_cpp::get_package_share_directory("robot_comm_serial");
        const std::array<std::string, 10> music_files = {
            share_dir + "/config/r2-d2.mp3",
            share_dir + "/config/1-screaming.mp3",
            share_dir + "/config/5.mp3",
            share_dir + "/config/10.mp3",
            share_dir + "/config/12.mp3",
            share_dir + "/config/19.mp3",
            share_dir + "/config/acknowledged-2.mp3",
            share_dir + "/config/acknowledged.mp3",
            share_dir + "/config/hee-hee.mp3",
            share_dir + "/config/worried.mp3"};
        static double dt = this->get_clock()->now().seconds();

        if (finished && this->get_clock()->now().seconds() - dt > 10)
        {
            dt = this->get_clock()->now().seconds();

            size_t index = std::rand() % (music_files.size());
            int ok = playSound(music_files.at(index).c_str());
        }
    }

    json status;
    status["type"] = "full_status";

    double timestamp = this->get_clock()->now().seconds();
    status["timestamp"] = timestamp;

    status["velocity"] = {
        {"fusion",
         {{"linear", last_message_.velocities().linear_mm_s() / 1000.0},
          {"angular", last_message_.velocities().angular_trad_s() / 1000.0}}},
        {"encoder",
         {{"linear",
           last_message_.encoder().twist_enc().linear_mm_s() / 1000.0},
          {"angular",
           last_message_.encoder().twist_enc().angular_trad_s() / 1000.0},
          {"left", last_message_.velocities().left_wheel_mm_s() / 1000.0},
          {"right", last_message_.velocities().right_wheel_mm_s() / 1000.0}}},
        {"imu", {{"angular", last_message_.imu().gyro().z() / 1000.f}}}};

    status["setpoints"] = {
        {"linear", last_command_.velocities().linear() / 1000.f},
        {"angular", last_command_.velocities().angular() / 1000.f}};

    status["acceleration"] = {{"x", last_message_.imu().acc().x() / 1000.f},
                              {"y", last_message_.imu().acc().y() / 1000.f},
                              {"z", last_message_.imu().acc().z() / 1000.f}};

    status["pose"] = {{"x", last_message_.pose().x_mm() / 1000.0},
                      {"y", last_message_.pose().y_mm() / 1000.0},
                      {"theta", last_message_.pose().yaw_trad() / 1000.0}};

    status["encoders"] = {{"left_pulses", last_message_.encoder().left()},
                          {"right_pulses", last_message_.encoder().right()}};
    status["gauges"] = {
        {"battery_level", last_message_.power_status().battery_percent()},
        {"motor_current",
         last_message_.power_status().driver_current_ma() / 1000.f},
        {"charging_status",
         last_message_.power_status().charging() ? "idle" : "discharging"},
        {"charging_current",
         last_message_.power_status().charging_current_ma() / 1000.f},
        {"temp_imu", last_message_.imu().temperature()},
        {"temp_ecu", last_message_.power_status().temperature()},
        {"temp_mcu", last_message_.power_status().internal_temperature()}};

    bool bpFL = false, bpFR = false, bpBL = false, bpBR = false;
    if (last_message_.bumpers_size() > 3)
    {
        bpFL = last_message_.bumpers(0).status();
        bpFR = last_message_.bumpers(1).status();
        bpBL = last_message_.bumpers(2).status();
        bpBR = last_message_.bumpers(3).status();
    }
    status["status_flags"] = {
        {"estop", last_message_.emergency_button_pressed()},
        {"bumpers", {{"fl", bpFL}, {"fr", bpFR}, {"bl", bpBL}, {"br", bpBR}}}};

    ws_interface_->send_robot_status(status);
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
