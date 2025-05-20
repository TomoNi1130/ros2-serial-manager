#include "serial_manager.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace serial_manager {

SerialPort::SerialPort(boost::asio::io_context &io, const std::string &port_name, rclcpp::Publisher<interface_pkg::msg::SerialMsg>::SharedPtr publisher_, const rclcpp::Logger &logger) : serial(io), port_name(port_name), id(0), publisher_(publisher_), logger(logger) {
  serial = boost::asio::serial_port(io, port_name);
  serial.set_option(serial_port_base::baud_rate(115200));
  serial.async_read_some(boost::asio::buffer(buffer, 1), [this](const boost::system::error_code &error, size_t bytes_transferred) { this->serial_callback(error, bytes_transferred); });
}

int SerialPort::get_id() { return id; }

void SerialPort::serial_callback(const boost::system::error_code &ec, std::size_t bytes_transferred) {
  if (!ec) {
    receive_msg += buffer[0];
    if (receive_msg.find("\n") != std::string::npos) {
      receive_msg.erase(receive_msg.length() - 2, receive_msg.length());
      if (receive_msg.substr(0, 5) == "[new]") {
        receive_msg.erase(0, 5);
        id = std::stoi(receive_msg);
        RCLCPP_INFO(logger, "port:%s's id set [%d]", port_name.c_str(), id);
      } else if (id == 0) {
        RCLCPP_INFO(logger, "[%swarm%s IDの未指定] ポート%s %s %sのidが指定されていません", red.c_str(), reset.c_str(), yellow.c_str(), port_name.c_str(), reset.c_str());
      } else if (receive_msg.substr(0, 5) == "[inf]") {
        receive_msg.erase(0, 5);
        size_t num_start = receive_msg.find("n");
        size_t flag_start = receive_msg.find("b");
        size_t str_start = receive_msg.find("s");
        std::string num_str = receive_msg.substr(num_start + 1, flag_start - num_start - 1);
        std::string flag_str = receive_msg.substr(flag_start + 1, str_start - flag_start - 1);
        std::string str = receive_msg.substr(str_start + 1, receive_msg.size() - str_start);  // nbsのぶんずらす

        interface_pkg::msg::SerialMsg publish_msg;
        if (num_start != std::string::npos) {
          publish_msg.numbers = split_strring<float>(num_str);
          // RCLCPP_INFO(logger, "%s", num_str.c_str());
        }
        if (flag_start != std::string::npos) {
          publish_msg.flags = split_strring<bool>(flag_str);
          // RCLCPP_INFO(logger, "%s", flag_str.c_str());
        }
        if (str_start != std::string::npos) {
          publish_msg.msg_str = split_strring<std::string>(str);
          // RCLCPP_INFO(logger, "%s", str.c_str());
        }
        publish_msg.msg_id = uint8_t(id);
        publisher_->publish(publish_msg);
      } else if (receive_msg.substr(0, 5) == "[log]") {
        receive_msg.erase(0, 5);
        RCLCPP_INFO(logger, "[log id:%d]%s", id, receive_msg.c_str());
      }
      receive_msg.clear();
    }
  } else {
    RCLCPP_ERROR(logger, "Serial error: %s", ec.message().c_str());
    return;
  }
  serial.async_read_some(boost::asio::buffer(buffer, 1), [this](const boost::system::error_code &error, size_t bytes_transferred) { this->serial_callback(error, bytes_transferred); });
}

void SerialPort::send_serial(const std::string &send_str) {
  boost::asio::async_write(serial, boost::asio::buffer(send_str), [this](boost::system::error_code ec, std::size_t bytes_transferred) {});
}

SerialPort::~SerialPort() {}

SerialManager::SerialManager(const rclcpp::NodeOptions &options) : rclcpp::Node("Serial_manager", options), io() {
  subscription_ = this->create_subscription<interface_pkg::msg::SerialMsg>("send_to_micro", 10, std::bind(&SerialManager::topic_callback, this, _1));
  publisher_ = this->create_publisher<interface_pkg::msg::SerialMsg>("micro_data", 10);
  send_msg_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&SerialManager::serial_send, this));
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
  std::string send_msg;
  send_msg += "n";
  for (double number : msg.numbers)
    send_msg += ":" + std::to_string(number);
  send_msg += "b";
  for (bool flag : msg.flags)
    if (flag)
      send_msg += ":1";
    else
      send_msg += ":0";
  send_msg += "s";
  for (std::string str : msg.msg_str)
    send_msg += ":" + str;
  send_msg += "\n";
  send_msgs[msg.msg_id] = send_msg;
}

void SerialManager::serial_send() {
  for (size_t i = 0; i < send_msgs.size(); i++) {
    for (const auto &serial_port_ptr : serial_ports) {
      SerialPort &serial_port = *serial_port_ptr;
      if (serial_port.get_id() == i && send_msgs[i] != "") {
        serial_port.send_serial(send_msgs[i]);
        send_msgs[i] = "";
      }
    }
  }
}

}  // namespace serial_manager

RCLCPP_COMPONENTS_REGISTER_NODE(serial_manager::SerialManager)