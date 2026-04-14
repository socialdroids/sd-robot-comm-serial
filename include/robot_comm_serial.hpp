#ifndef INCLUDE_INCLUDE_ROBOT_COMM_SERIAL_HPP_
#define INCLUDE_INCLUDE_ROBOT_COMM_SERIAL_HPP_

#include <chrono>
#include <deque>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <serial/serial.h>
#include <string.h>

#include "builtin_interfaces/msg/time.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "navigation_tester.hpp"
#include "websocket_interface.hpp"

#include "cobs.h"
#include "command.pb.h"
#include "feedback.pb.h"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "control_logger.hpp"

// Para ler a pose do robô
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// ROS messages
#include "sd_msgs/msg/base_params.hpp"
#include "sd_msgs/msg/build_info.hpp"
#include "sd_msgs/msg/bumper.hpp"
#include "sd_msgs/msg/encoder.hpp"
#include "sd_msgs/msg/encoder_status.hpp"
#include "sd_msgs/msg/pid_config.hpp"
#include "sd_msgs/msg/power_status.hpp"
#include "sd_msgs/msg/robot_bumpers.hpp"
#include "sd_msgs/msg/robot_debug.hpp"
#include "sd_msgs/msg/robot_encoders.hpp"
#include "sd_msgs/msg/robot_flags.hpp"
#include "sd_msgs/msg/robot_parameters.hpp"
#include "sd_msgs/msg/power_on_time.hpp"

using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::microseconds;
using std::chrono::milliseconds;
using std::chrono::nanoseconds;

typedef uint8_t CRC_t;
#define CRC_OK 0x00

// Tabela calculada com o polinomio 0x07
const CRC_t crcTable[256] = {
    0,   7,   14,  9,   28,  27,  18,  21,  56,  63,  54,  49,  36,  35,  42,
    45,  112, 119, 126, 121, 108, 107, 98,  101, 72,  79,  70,  65,  84,  83,
    90,  93,  224, 231, 238, 233, 252, 251, 242, 245, 216, 223, 214, 209, 196,
    195, 202, 205, 144, 151, 158, 153, 140, 139, 130, 133, 168, 175, 166, 161,
    180, 179, 186, 189, 199, 192, 201, 206, 219, 220, 213, 210, 255, 248, 241,
    246, 227, 228, 237, 234, 183, 176, 185, 190, 171, 172, 165, 162, 143, 136,
    129, 134, 147, 148, 157, 154, 39,  32,  41,  46,  59,  60,  53,  50,  31,
    24,  17,  22,  3,   4,   13,  10,  87,  80,  89,  94,  75,  76,  69,  66,
    111, 104, 97,  102, 115, 116, 125, 122, 137, 142, 135, 128, 149, 146, 155,
    156, 177, 182, 191, 184, 173, 170, 163, 164, 249, 254, 247, 240, 229, 226,
    235, 236, 193, 198, 207, 200, 221, 218, 211, 212, 105, 110, 103, 96,  117,
    114, 123, 124, 81,  86,  95,  88,  77,  74,  67,  68,  25,  30,  23,  16,
    5,   2,   11,  12,  33,  38,  47,  40,  61,  58,  51,  52,  78,  73,  64,
    71,  82,  85,  92,  91,  118, 113, 120, 127, 106, 109, 100, 99,  62,  57,
    48,  55,  34,  37,  44,  43,  6,   1,   8,   15,  26,  29,  20,  19,  174,
    169, 160, 167, 178, 181, 188, 187, 150, 145, 152, 159, 138, 141, 132, 131,
    222, 217, 208, 215, 194, 197, 204, 203, 230, 225, 232, 239, 250, 253, 244,
    243};

class RobotSerial : public rclcpp::Node
{
public:
    RobotSerial();
    ~RobotSerial();

private:
    static constexpr char PACKET_END[] = {"\0"};
    static constexpr size_t MAX_PACKET_SIZE = {
        4220}; /** Tamanho máximo de um pacote enviado ou recebido. Valor
definido com base no
COBS_ENCODE_DST_BUF_LEN_MAX(FEEDBACK_PB_H_MAX_SIZE+CRC)+DELIMITER};
*/
    static constexpr int8_t WIDTH = (8 * sizeof(CRC_t));
    static constexpr int16_t TOPBIT = (1 << (WIDTH - 1));
    static constexpr int8_t POLYNOMIAL = 0x07;

    rclcpp::Publisher<sd_msgs::msg::RobotFlags>::SharedPtr robot_flags_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr
        imu_temperature_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    rclcpp::Publisher<sd_msgs::msg::RobotEncoders>::SharedPtr encoder_pub_;
    rclcpp::Publisher<sd_msgs::msg::RobotBumpers>::SharedPtr bumpers_pub_;
    rclcpp::Publisher<sd_msgs::msg::RobotDebug>::SharedPtr debug_pub_;
    rclcpp::Publisher<sd_msgs::msg::BaseParams>::SharedPtr base_params_pub_;

    rclcpp::Publisher<sd_msgs::msg::PowerStatus>::SharedPtr power_status_pub_;
    rclcpp::Publisher<sd_msgs::msg::PowerOnTime>::SharedPtr power_on_time_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr jump_to_boot_sub_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enter_standby_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr emg_stop_srv_;

    std::shared_ptr<serial::Serial> serial_port_;
    rclcpp::TimerBase::SharedPtr packet_timer_;
    rclcpp::TimerBase::SharedPtr reconnect_timer_;
    rclcpp::TimerBase::SharedPtr command_timer_;
    rclcpp::TimerBase::SharedPtr gui_update_timer_;

    std::unique_ptr<WebsocketInterface> ws_interface_;
    std::unique_ptr<NavigationTester> nav_tester_;

    size_t packet_size_;
    uint8_t
        buffer_[MAX_PACKET_SIZE]; /**< COBS encoded packet buffer [input]. */
    size_t current_buffer_pos_;

    uint8_t output_buffer_[MAX_PACKET_SIZE];  /**< COBS encoded packet buffer
                                                 [output]. */
    uint8_t decoded_packet_[MAX_PACKET_SIZE]; /**< COBS decoded packet. */
    uint8_t encoded_packet_[MAX_PACKET_SIZE]; /**< Protobuf encoded packet. */
    static constexpr size_t MAX_FREQUENCY_SAMPLES = 100;
    std::deque<float> packet_frequency_;
    std::chrono::time_point<high_resolution_clock> last_packet_time_;
    std::chrono::time_point<high_resolution_clock> last_cmd_vel_time_;
    std::chrono::time_point<high_resolution_clock> last_virtual_cmd_time_;

    FeedbackMessage last_message_;
    rclcpp::Time last_message_timestamp_;
    RobotParameters last_params_;
    CommandMessage last_command_;
    bool last_message_ok_;

    bool fake_charging_;
    float fake_charging_radius_;
    uint8_t fake_charging_fail_count_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    unsigned long baud_;
    std::string port_;
    bool connected_;

    size_t MAX_RTOS_TASKS;

    struct RobotActionStatus {
        bool active;
        uint8_t sent;
        std::chrono::time_point<high_resolution_clock> sentTimestamp;
        bool confirmed;
        bool data;
    } jumpToBootStatus, enterStandByStatus, emergencyStopStatus;

    ControlLogger cLogger;

    void cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void jump_to_boot_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void enter_standby_srv_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void emergency_stop_srv_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void packet_callback();
    void reconnect_callback();
    void command_callback();

    void clear_command();

    void connect();

    void decode_buffer();
    CRC_t crcFast(uint8_t const* _message, int _nBytes);
    void update_packet_frequency();
    float packet_frequency();
    std::string packet_to_str(uint8_t const* _buffer, size_t _buffLen);
    void manage_robot_actions();

    void publish_data();
    void publish_power_status();
    void publish_stand_by_status();

    void publish_flags();
    void publish_imu();
    void publish_odometry();
    void publish_encoders();
    void publish_bumpers();
    void publish_debug();
    void publish_base_params();
    void publish_battery();
    void publish_robot_config();
    void publish_rtos_info();

    /**
     * @brief Determina o estado de carregamento fake baseado na distância para
     * a origem do mapa (base)
     *
     * @return True se estiver perto da base, False se não estiver ou se o fake
     * charging estiver desabilitado
     */
    bool fake_charging_status();

    /**
     * @brief Lida com os comandos da Web gUI
     *
     * @param type Tipo de comando
     * @param data Dados
     */
    void handle_gui_command(const std::string& type, const json& data);
    void update_ecu_info(
        const std::string& _name, const std::string& _driver_version,
        const std::string& _ecu_version, const std::string& _motor_version,
        const std::string& _git_hash, const std::string& _git_branch,
        const std::string& _git_tag, const std::string& _build_date,
        float _wheel_distance, float _wheel_diameter);
    void update_nav_info();
    void update_config_info();
    void publish_full_status();

    template <typename TimeResolution>
    std::chrono::high_resolution_clock::duration::rep timeSince(
        std::chrono::time_point<high_resolution_clock>& _start)
    {
        return duration_cast<TimeResolution>(high_resolution_clock::now() -
                                             _start)
            .count();
    }

    template <typename Func> void try_serial_operation(Func&& func)
    {
        try
        {
            func();
        }
        catch (serial::PortNotOpenedException& e)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Serial Operation Failed! Reason: %s.", e.what());
            connected_ = false;
        }
        catch (serial::SerialException& e)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Serial Operation Failed! Reason: %s.", e.what());
            connected_ = false;
        }
        catch (serial::IOException& e)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Serial Operation Failed! Reason: %s.", e.what());
            connected_ = false;
        }
    }
};

#endif // INCLUDE_INCLUDE_ROBOT_COMM_SERIAL_HPP_
