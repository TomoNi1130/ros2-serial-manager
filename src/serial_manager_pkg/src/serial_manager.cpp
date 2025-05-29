#include "serial_manager.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace serial_manager {

SerialPort::SerialPort(boost::asio::io_context &io, const std::string &port_name, rclcpp::Publisher<interface_pkg::msg::SerialMsg>::SharedPtr publisher_, const rclcpp::Logger &logger) : serial(io), port_name(port_name), id(0), publisher_(publisher_), logger(logger) {
  serial = boost::asio::serial_port(io, port_name);
  serial.set_option(serial_port_base::baud_rate(115200));
  serial.async_read_some(boost::asio::buffer(buffer, 1), [this](const boost::system::error_code &error, size_t bytes_transferred) { this->serial_callback(error, bytes_transferred); });
}

int SerialPort::get_id() { return id; }

float bytes_to_float(uint8_t *bytes) {
  float value;
  memcpy(&value, bytes, sizeof(float));
  return value;
}

void SerialPort::serial_callback(const boost::system::error_code &ec, std::size_t bytes_transferred) {
  if (!ec) {
    uint8_t buf = buffer[0];
    receive_bytes.push_back(buf);
    if (buf == 0) {
      uint8_t type_keeper = receive_bytes[0];  // 型識別用のデータ(使わない)
      receive_bytes.erase(receive_bytes.begin());

      uint8_t OBH;  // ゼロが出るまでの数
      std::vector<uint8_t> decorded_data;
      OBH = receive_bytes[0];
      for (uint8_t i = 1; i < receive_bytes.size(); i++) {
        if (i == OBH) {
          decorded_data.push_back(0);
          OBH = receive_bytes[i] + OBH;
        } else {
          decorded_data.push_back(receive_bytes[i]);
        }
      }
      std::string str;
      for (uint8_t byte : receive_bytes)
        str += std::to_string(byte) + " ";
      str.clear();

      if (type_keeper == 0x01) {  // 小数
        std::vector<float> results;
        for (size_t i = 0; i < decorded_data.size() / sizeof(float); i++) {
          float result = bytes_to_float(&decorded_data[i * sizeof(float)]);
          results.push_back(result);
        }
        if (id != 0) {
          pub_msg_data_.msg_id = id;
          pub_msg_data_.numbers.clear();
          pub_msg_data_.numbers = results;
          publisher_->publish(pub_msg_data_);
        }
      } else if (type_keeper == 0x02) {  // bool
        std::vector<bool> results;
        for (size_t i = 0; i < decorded_data.size(); i++) {
          if (decorded_data[i] == 0)
            results.push_back(false);
          else if (decorded_data[i] == 1)
            results.push_back(true);
        }
        if (id != 0) {
          pub_msg_data_.msg_id = id;
          pub_msg_data_.flags.clear();
          pub_msg_data_.flags = results;
          publisher_->publish(pub_msg_data_);
        }
      } else if (type_keeper == 0xfe) {  // log
        std::string log_msg(decorded_data.begin(), decorded_data.end());
        if (id != 0)
          RCLCPP_INFO(logger, "[%d]log: %s", id, log_msg.c_str());
      } else if (type_keeper == 0xaa) {  // マイコンからの自己紹介
        for (uint8_t byte : decorded_data)
          str += std::to_string(byte) + " ";
        if (std::equal(self_intro_bytes.begin(), self_intro_bytes.end(), decorded_data.begin())) id = decorded_data[3];
        RCLCPP_INFO(logger, "%s %s %sが ID[%s %d %s]として登録されました", green.c_str(), port_name.c_str(), reset.c_str(), green.c_str(), id, reset.c_str());
      }
      receive_bytes.clear();
    }
  } else {
    RCLCPP_ERROR(logger, "Serial error: %s", ec.message().c_str());
    return;
  }
  serial.async_read_some(boost::asio::buffer(buffer, 1), [this](const boost::system::error_code &error, size_t bytes_transferred) { this->serial_callback(error, bytes_transferred); });
}

void SerialPort::send_serial(const std::vector<uint8_t> &send_bytes) {
  boost::asio::async_write(serial, boost::asio::buffer(send_bytes), [this](boost::system::error_code ec, std::size_t bytes_transferred) {});
}

SerialPort::~SerialPort() {}

SerialManager::SerialManager(const rclcpp::NodeOptions &options) : rclcpp::Node("Serial_manager", options), io() {
  subscription_ = this->create_subscription<interface_pkg::msg::SerialMsg>("send_to_micro", 10, std::bind(&SerialManager::topic_callback, this, _1));
  publisher_ = this->create_publisher<interface_pkg::msg::SerialMsg>("micro_data", 10);
  send_msg_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&SerialManager::serial_send, this));
  heart_beat_timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&SerialManager::serial_heartbeat, this));
  port_names = find_serial_port();
  if (!port_names.empty()) {
    RCLCPP_INFO(this->get_logger(), "%s %zu つのシリアルデバイスが接続されています%s", green.c_str(), port_names.size(), reset.c_str());
    send_msgs.resize(4);
    for (size_t i = 0; i < port_names.size(); i++) {
      RCLCPP_INFO(this->get_logger(), "%sポート%zu %s%s", green.c_str(), i, port_names[i].c_str(), reset.c_str());
      serial_ports.emplace_back(std::make_unique<SerialPort>(io, port_names[i], publisher_, this->get_logger()));
    }
    io_thread_ = std::thread([this]() { io.run(); });
    RCLCPP_INFO(this->get_logger(), "setup!!");
  } else {
    RCLCPP_ERROR(this->get_logger(), "%sポートが接続されてないやんけ!%s", red.c_str(), reset.c_str());
    rclcpp::shutdown();
    return;
  }
}

SerialManager::~SerialManager() {
  io.stop();
  if (io_thread_.joinable())
    io_thread_.join();
}

std::vector<std::string> SerialManager::find_serial_port() {
  std::vector<std::string> device_paths;
  for (const auto &entry : std::filesystem::directory_iterator("/dev"))
    if (entry.is_character_file() || entry.is_block_file() || entry.is_symlink()) {
      std::string filename = entry.path().filename().string();
      if (filename.rfind("ttyACM", 0) == 0)  // パスがttyACMから始まるか
        device_paths.push_back(entry.path().string());
    }
  return device_paths;
}

void SerialManager::topic_callback(const interface_pkg::msg::SerialMsg &msg) {
  SendMsgData send_msg_data;
  send_msg_data.msg_id = msg.msg_id;
  send_msg_data.float_data = cobs_encode(msg.numbers);
  std::vector<uint8_t> booldata(msg.flags.begin(), msg.flags.end());
  send_msg_data.bool_data = cobs_encode(booldata);
  send_msgs.push_back(send_msg_data);
}

void SerialManager::serial_send() {
  for (size_t i = 0; i < send_msgs.size(); i++)
    for (const auto &serial_port_ptr : serial_ports) {
      SerialPort &serial_port = *serial_port_ptr;
      if (serial_port.get_id() == send_msgs[i].msg_id) {
        serial_port.send_serial(send_msgs[i].float_data);
        serial_port.send_serial(send_msgs[i].bool_data);
      }
    }
  send_msgs.clear();
}

void SerialManager::serial_heartbeat() {
  for (const auto &serial_port_ptr : serial_ports) {
    SerialPort &serial_port = *serial_port_ptr;
    serial_port.send_serial(serial_port.heartbeat_bytes);
  }
}

}  // namespace serial_manager

RCLCPP_COMPONENTS_REGISTER_NODE(serial_manager::SerialManager)