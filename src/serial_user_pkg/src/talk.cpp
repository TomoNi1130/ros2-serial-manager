#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "serial_manager_pkg/msg/serial_msg.hpp"

class SerialTalker : public rclcpp::Node {
 public:
  SerialTalker() : Node("serial_talker") {
    publisher_ = this->create_publisher<serial_manager_pkg::msg::SerialMsg>("send_to_micro", 10);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&SerialTalker::joy_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&SerialTalker::timer_callback, this));
  }

 private:
  void joy_callback(const sensor_msgs::msg::Joy& msg) {
    serial_manager_pkg::msg::SerialMsg send_msg;

    for (bool i : msg.buttons)
      send_msg.flags.push_back(i);

    send_msg.numbers.push_back(msg.axes[1]);
    send_msg.numbers.push_back(msg.axes[0]);
    send_msg.numbers.push_back(msg.axes[4]);
    send_msg.numbers.push_back(msg.axes[3]);

    send_msg.msg_id = 1;
    publisher_->publish(send_msg);
    send_msg.msg_id = 2;
    publisher_->publish(send_msg);
    send_msg.msg_id = 3;
    publisher_->publish(send_msg);
  }

  void timer_callback() {
    serial_manager_pkg::msg::SerialMsg send_msg;

    send_msg.numbers.push_back(0.4);
    send_msg.numbers.push_back(11.30);
    send_msg.numbers.push_back(20.08);
    send_msg.numbers.push_back(0.00);

    send_msg.flags.push_back(true);
    send_msg.flags.push_back(false);

    send_msg.msg_id = 0;  // 0はブロードキャスト
    publisher_->publish(send_msg);
    RCLCPP_DEBUG(this->get_logger(), "送ったぜ");
  }

  bool pre_up = false;
  bool pre_down = false;

  rclcpp::Publisher<serial_manager_pkg::msg::SerialMsg>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialTalker>());
  rclcpp::shutdown();
  return 0;
}