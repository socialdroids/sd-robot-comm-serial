#include "robot_comm_serial.hpp"

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  unsigned long baud = 1000000;
  std::string port("/dev/ttyACM0");

  FeedbackMessage message;
  serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));
  // auto node = std::make_shared<Bumpers>();
  // node->run();
  rclcpp::shutdown();
  return 0;

  return 0;
}
