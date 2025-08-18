#ifndef COBS_SERIAL_MANAGER_HPP
#define COBS_SERIAL_MANAGER_HPP

#include <boost/asio.hpp>
#include <filesystem>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "serial_manager_pkg/msg/serial_msg.hpp"

namespace serial_manager {

const std::string red = "\033[31m";     // 赤色
const std::string green = "\033[32m";   // 緑
const std::string yellow = "\033[33m";  // 黄色
const std::string reset = "\033[0m";    // リセット

const uint8_t FLOAT_HEADER = 0x01;
const uint8_t BOOL_HAEDER = 0x02;
const uint8_t LOG_HEADER = 0x09;
const uint8_t HEART_BEAT_HEADER = 0x77;
const std::vector<uint8_t> START_COM_BYTES = {0xff};                 // マイコンとの通信開始信号
const std::vector<uint8_t> INTRODUCTION_BYTES = {0x12, 0x00, 0x56};  // 自己紹介用データセット
const std::vector<uint8_t> RECORL_BYTES = {0x56, 0x34, 0x12};        // 返信用データセット
const std::vector<uint8_t> HEARTBEAT_BYTES = {0xaa, 0xbb, 0xcc};     // ハートビート用のバイト列

struct SendMsgData {
  std::vector<float> float_data;
  std::vector<bool> bool_data;
  void clear() {
    float_data.clear();
    bool_data.clear();
  }
};

class SerialPort {
 public:
  SerialPort(boost::asio::io_context& io, const std::string& port_name, rclcpp::Publisher<serial_manager_pkg::msg::SerialMsg>::SharedPtr publisher_, const rclcpp::Logger& logger);
  ~SerialPort();
  void set_sendmsg(const std::vector<uint8_t>& send_bytes);
  void set_sendmsg(const SendMsgData& send_bytes);
  int get_id();

 private:
  void serial_callback(const boost::system::error_code& ec, std::size_t bytes_transferred);
  void send_serial();
  void heartbeat();
  template <typename T>
  std::vector<uint8_t> make_msg(const std::vector<T>& input);
  template <typename T>
  std::vector<uint8_t> cobs_encode(const std::vector<T>& input);

  enum State {
    SETUP,    // 初期化中、マイコンへ存在を通知する
    STANBY,   // 接続待機状態、マイコンIDを待つ
    CONNECT,  // 通信状態、通信可能
  };

  const double WaitTimePerByte_ = 0.1;  // ms

  State state_;

  boost::asio::serial_port serial;
  std::string port_name;
  std::array<uint8_t, 1> buffer;
  uint8_t id;  // 0はなし

  serial_manager_pkg::msg::SerialMsg pub_msg_data_;

  std::vector<uint8_t> decoded_data;
  std::vector<uint8_t> receive_bytes;
  SendMsgData send_msg;
  rclcpp::Publisher<serial_manager_pkg::msg::SerialMsg>::SharedPtr publisher_;
  rclcpp::Logger logger;

  rclcpp::Clock clock;
  rclcpp::Time last_heartbeat_time;
  rclcpp::Time now;

  std::thread send_msg_thread;
  std::thread heartbeat_thread;

  bool running_ = true;  // スレッドの実行状態を管理
};

class SerialManager : public rclcpp::Node {
 public:
  SerialManager(const rclcpp::NodeOptions& options);
  ~SerialManager();

 private:
  void topic_callback(const serial_manager_pkg::msg::SerialMsg& msg);
  std::vector<std::string> find_serial_port();

  boost::asio::io_context io;
  std::thread io_thread_;

  std::vector<std::string> port_names;
  std::unordered_map<std::string, std::unique_ptr<SerialPort>> serial_ports;

  rclcpp::Subscription<serial_manager_pkg::msg::SerialMsg>::SharedPtr subscription_;
  rclcpp::Publisher<serial_manager_pkg::msg::SerialMsg>::SharedPtr publisher_;
};

template <typename T>
std::vector<uint8_t> SerialPort::make_msg(const std::vector<T>& input) {
  std::vector<uint8_t> encoded;
  if constexpr (std::is_same_v<T, float>) {
    encoded.push_back(FLOAT_HEADER);
    std::vector<uint8_t> encoded_data = cobs_encode(input);
    encoded.insert(encoded.end(), encoded_data.begin(), encoded_data.end());
    return encoded;
  } else if constexpr (std::is_same_v<T, double>) {
    std::vector<float> float_input(input.begin(), input.end());
    encoded.push_back(FLOAT_HEADER);
    std::vector<uint8_t> encoded_data = cobs_encode(float_input);
    encoded.insert(encoded.end(), encoded_data.begin(), encoded_data.end());
    return encoded;
  } else if constexpr (std::is_same_v<T, uint8_t>) {
    encoded.push_back(BOOL_HAEDER);
    std::vector<uint8_t> encoded_data = cobs_encode(input);
    encoded.insert(encoded.end(), encoded_data.begin(), encoded_data.end());
    return encoded;
  } else if constexpr (std::is_same_v<T, bool>) {
    encoded.push_back(BOOL_HAEDER);
    std::vector<uint8_t> bool_bytes(input.begin(), input.end());
    std::vector<uint8_t> encoded_data = cobs_encode(bool_bytes);
    encoded.insert(encoded.end(), encoded_data.begin(), encoded_data.end());
    return encoded;
  } else {
    encoded.push_back(0xff);
    std::vector<uint8_t> encoded_data = cobs_encode(input);
    encoded.insert(encoded.end(), encoded_data.begin(), encoded_data.end());
    return encoded;
  }
}

template <typename T>
std::vector<uint8_t> SerialPort::cobs_encode(const std::vector<T>& input) {
  std::vector<uint8_t> encoded;
  encoded.push_back(0x00);  // プレースホルダ
  size_t mark = 0;
  uint8_t count = 1;  // code byteは1から始まる

  for (size_t i = 0; i < input.size(); ++i) {
    const uint8_t* raw = reinterpret_cast<const uint8_t*>(&input[i]);

    for (size_t j = 0; j < sizeof(T); ++j) {
      if (raw[j] == 0x00) {
        encoded[mark] = count;
        mark = encoded.size();
        encoded.push_back(0x00);  // 新しいブロックのプレースホルダ
        count = 1;
      } else {
        encoded.push_back(raw[j]);
        ++count;

        if (count == 0xFF) {
          encoded[mark] = count;
          mark = encoded.size();
          encoded.push_back(0x00);  // 新しいブロック
          count = 1;
        }
      }
    }
  }

  // 最後のブロック処理
  encoded[mark] = count;
  encoded.push_back(0x00);  // 終端用

  return encoded;
}

}  // namespace serial_manager

#endif  // COBS_SERIAL_MANAGER_HPP