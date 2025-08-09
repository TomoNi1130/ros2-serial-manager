#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "serial_manager_pkg/msg/serial_msg.hpp"

class SerialTalker : public rclcpp::Node {
 public:
  SerialTalker() : Node("serial_talker") {
    publisher_ = this->create_publisher<serial_manager_pkg::msg::SerialMsg>("send_to_micro", 10);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&SerialTalker::joy_callback, this, std::placeholders::_1));
  }

 private:
  void joy_callback(const sensor_msgs::msg::Joy& msg) {
    serial_manager_pkg::msg::SerialMsg send_msg;

    if (msg.buttons[2] == true) {
      send_msg.flags.push_back(true);
    } else {
      send_msg.flags.push_back(false);
    }

    send_msg.msg_id = 1;
    publisher_->publish(send_msg);
  }

  bool pre_up = false;
  bool pre_down = false;

  rclcpp::Publisher<serial_manager_pkg::msg::SerialMsg>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialTalker>());
  rclcpp::shutdown();
  return 0;
}