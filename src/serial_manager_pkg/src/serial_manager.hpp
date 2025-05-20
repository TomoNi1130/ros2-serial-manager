#ifndef SERIAL_MANAGER_HPP
#define SERIAL_MANAGER_HPP

#include <boost/asio.hpp>
#include <filesystem>
#include <thread>
#include <type_traits>
#include <vector>

#include "interface_pkg/msg/serial_msg.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace boost::asio;

using std::placeholders::_1, std::placeholders::_2;

namespace serial_manager {

class SerialPort {
 public:
  SerialPort(boost::asio::io_context& io, const std::string& port_name, rclcpp::Publisher<interface_pkg::msg::SerialMsg>::SharedPtr publisher_, const rclcpp::Logger& logger);
  ~SerialPort();
  void send_serial(const std::string& send_str);
  int get_id();

 private:
  void serial_callback(const boost::system::error_code& ec, std::size_t bytes_transferred);

  template <typename T>
  std::vector<T> split_strring(const std::string& target_string) {  //: で分ける
    std::vector<T> result;
    std::stringstream ss(target_string);
    std::string token;
    while (std::getline(ss, token, ':'))
      if (!token.empty()) {
        if constexpr (std::is_same_v<T, float>)
          result.push_back(std::stof(token));
        else if constexpr (std::is_same_v<T, double>)
          result.push_back(std::stod(token));
        else if constexpr (std::is_same_v<T, std::string>)
          result.push_back(token);
        else if constexpr (std::is_same_v<T, bool>) {
          if (token == "0")
            result.push_back(false);
          else if (token == "1")
            result.push_back(true);
        }
      }

    return result;
  }

  boost::asio::serial_port serial;
  std::string port_name;
  std::string receive_msg;
  std::string send_msg;
  std::array<char, 1> buffer;
  int id;  // 0はなし

  rclcpp::Publisher<interface_pkg::msg::SerialMsg>::SharedPtr publisher_;
  rclcpp::Logger logger;

  std::string red = "\033[31m";     // 赤色
  std::string green = "\033[32m";   // 緑
  std::string yellow = "\033[33m";  // 黄色
  std::string reset = "\033[0m";    // リセット
};

class SerialManager : public rclcpp::Node {
 public:
  explicit SerialManager(const rclcpp::NodeOptions& options);
  virtual ~SerialManager();

 private:
  std::vector<std::string> find_serial_port();
  void topic_callback(const interface_pkg::msg::SerialMsg& msg);
  void serial_send();

  boost::asio::io_context io;
  std::thread io_thread_;
  std::vector<std::unique_ptr<SerialPort>> serial_ports;
  std::vector<std::string> port_names;

  std::string red = "\033[31m";    // 赤色
  std::string green = "\033[32m";  // 緑
  std::string reset = "\033[0m";   // リセット

  std::vector<std::string> send_msgs;

  rclcpp::Subscription<interface_pkg::msg::SerialMsg>::SharedPtr subscription_;
  rclcpp::Publisher<interface_pkg::msg::SerialMsg>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr send_msg_timer_;
};

}  // namespace serial_manager

#endif