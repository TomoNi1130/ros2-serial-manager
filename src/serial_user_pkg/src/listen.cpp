#include "rclcpp/rclcpp.hpp"
#include "serial_manager_pkg/msg/serial_msg.hpp"

class SerialListener : public rclcpp::Node {
 public:
  SerialListener() : Node("serial_listener") {
    subscription_ = this->create_subscription<serial_manager_pkg::msg::SerialMsg>("micro_data", 10, std::bind(&SerialListener::topic_callback, this, std::placeholders::_1));
  }

 private:
  void topic_callback(const serial_manager_pkg::msg::SerialMsg& msg) {
    // RCLCPP_INFO(this->get_logger(), "from %d size: n,%zu f,%zu", msg.msg_id, msg.numbers.size(), msg.flags.size());
    std::string msg_str = "";
    for (size_t i = 0; i < msg.numbers.size(); i++) {
      msg_str += " " + std::to_string(msg.numbers[i]);
    }
    // RCLCPP_INFO(this->get_logger(), "nums : %s", msg_str.c_str());
    msg_str.clear();
    for (size_t i = 0; i < msg.flags.size(); i++) {
      msg_str += std::string(" ") + (msg.flags[i] ? "1" : "0");
    }
    // RCLCPP_INFO(this->get_logger(), "flags : %s", msg_str.c_str());
  }

  rclcpp::Subscription<serial_manager_pkg::msg::SerialMsg>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialListener>());
  rclcpp::shutdown();
  return 0;
}