#ifndef COBS_SERIAL_MANAGER_HPP
#define COBS_SERIAL_MANAGER_HPP

#include <boost/asio.hpp>
#include <filesystem>
#include <string>
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
  void send_serial(const std::vector<uint8_t>& send_bytes);
  int get_id();

  std::vector<uint8_t> heartbeat_bytes = {0xaa, 0x01, 0x01, 0x00};  // ハートビート用のバイト列
  std::vector<uint8_t> self_intro_bytes = {0x24, 0x08, 0x60};       // 自己紹介用のバイト列

 private:
  void serial_callback(const boost::system::error_code& ec, std::size_t bytes_transferred);

  boost::asio::serial_port serial;
  std::string port_name;
  std::string receive_msg;
  std::string send_msg;
  std::array<char, 1> buffer;
  int id;  // 0はなし

  interface_pkg::msg::SerialMsg pub_msg_data_;

  std::vector<uint8_t> receive_bytes;
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
  void serial_heartbeat();
  template <typename T>
  std::vector<uint8_t> cobs_encode(const std::vector<T>& input) {
    std::vector<uint8_t> encoded;
    if constexpr (std::is_same_v<T, float>)
      encoded.push_back(0x01);
    else if constexpr (std::is_same_v<T, uint8_t>)
      encoded.push_back(0x02);
    else
      encoded.push_back(0xff);
    encoded.push_back(0x00);
    uint8_t count = 0;  // 次にsource_data[i]に0x00が出るまでの配列番号をカウント
    int mark = 1;       // 最後に0x00が出たsource_data[i]の配列番号をキープ
    for (size_t i = 0; i < input.size(); ++i) {
      const uint8_t* raw = reinterpret_cast<const uint8_t*>(&input[i]);
      for (size_t j = 0; j < sizeof(T); ++j) {
        if (raw[j] != 0x00) {
          encoded.push_back(raw[j]);
          count++;
          if (count == 0xFF) {
            encoded[mark] = count;
            mark = encoded.size();
            encoded.push_back(0x00);
            count = 0;
          }
        } else {
          encoded[mark] = count + 1;
          mark = encoded.size();
          encoded.push_back(0x00);
          count = 0;
        }
      }
    }
    count++;
    encoded[mark] = count;
    encoded.push_back(0x00);
    return encoded;
  }

  struct SendMsgData {
    uint8_t msg_id;
    std::vector<uint8_t> float_data;
    std::vector<uint8_t> bool_data;
  };

  boost::asio::io_context io;
  std::thread io_thread_;
  std::vector<std::unique_ptr<SerialPort>> serial_ports;
  std::vector<std::string> port_names;

  std::string red = "\033[31m";    // 赤色
  std::string green = "\033[32m";  // 緑
  std::string reset = "\033[0m";   // リセット

  std::vector<SendMsgData> send_msgs;
  std::vector<SendMsgData> pub_msgs;

  rclcpp::Subscription<interface_pkg::msg::SerialMsg>::SharedPtr subscription_;
  rclcpp::Publisher<interface_pkg::msg::SerialMsg>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr send_msg_timer_;
  rclcpp::TimerBase::SharedPtr heart_beat_timer_;
};

}  // namespace serial_manager

#endif