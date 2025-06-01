#include "new_serial_manager.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace serial_manager {

float bytes_to_float(uint8_t *bytes) {
  float value;
  memcpy(&value, bytes, sizeof(float));
  return value;
}

SerialPort::SerialPort(boost::asio::io_context &io, const std::string &port_name, rclcpp::Publisher<interface_pkg::msg::SerialMsg>::SharedPtr publisher_, const rclcpp::Logger &logger) : serial(io), port_name(port_name), id(0), publisher_(publisher_), logger(logger) {
  send_msg_thread = std::thread([this]() { SerialPort::send_serial(); });
  RCLCPP_INFO(logger, "%s%s%s", green.c_str(), port_name.c_str(), reset.c_str());
  state_ = SETUP;
  serial = boost::asio::serial_port(io, port_name);
  serial.set_option(boost::asio::serial_port_base::baud_rate(115200));
  serial.async_read_some(boost::asio::buffer(buffer, 1), [this](const boost::system::error_code &error, size_t bytes_transferred) { this->serial_callback(error, bytes_transferred); });
}

SerialPort::~SerialPort() {
  running_ = false;
  if (send_msg_thread.joinable()) send_msg_thread.join();
  if (serial.is_open()) {
    serial.close();
  }
}

int SerialPort::get_id() { return id; }

void SerialPort::serial_callback(const boost::system::error_code &ec, std::size_t bytes_transferred) {
  if (!ec) {
    uint8_t buf = buffer[0];
    receive_bytes.push_back(buf);

    if (buf == 0x00) {
      uint8_t type_keeper = 0;
      if (state_ == CONNECT) {
        type_keeper = receive_bytes[0];  // 型識別用のデータ
        receive_bytes.erase(receive_bytes.begin());
      }
      uint8_t OBH;  // ゼロが出るまでの数
      OBH = receive_bytes[0];
      for (uint8_t i = 1; i < receive_bytes.size(); i++) {
        if (i == OBH) {
          OBH = receive_bytes[i] + OBH;
        } else {
          decorded_data.push_back(receive_bytes[i]);
        }
      }
      // デコード完了

      // 処理部
      switch (state_) {
        case CONNECT: {
          if (type_keeper == serial_manager::FLOAT_HEADER) {
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
          } else if (type_keeper == serial_manager::BOOL_HAEDER) {
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
          } else if (type_keeper == serial_manager::LOG_HEADER) {
            std::string log_msg(decorded_data.begin(), decorded_data.end());
            if (id != 0)
              RCLCPP_INFO(logger, "[ポート:%s%s%s ID:%s %d %s] log: %s", green.c_str(), port_name.c_str(), reset.c_str(), green.c_str(), id, reset.c_str(), log_msg.c_str());
          }
          break;
        }
        case STANBY: {
          if (decorded_data == START_COM_BYTES) {  // マイコンからの開始信号を受信
            state_ = CONNECT;
            RCLCPP_INFO(logger, "[ポート:%s%s%s] マイコンとの通信を開始しました。", green.c_str(), port_name.c_str(), reset.c_str());
          }
          break;
        }
        case SETUP: {
          if (decorded_data.size() == INTRODUCTION_BYTES.size() + 1) {
            if (std::equal(INTRODUCTION_BYTES.begin(), INTRODUCTION_BYTES.end(), decorded_data.begin())) {
              decorded_data.erase(decorded_data.begin(), decorded_data.begin() + INTRODUCTION_BYTES.size());  // 自己紹介用のデータを削除
              id = decorded_data[0];                                                                          // IDを取得
              RCLCPP_INFO(logger, "[ポート:%s%s%s] マイコンIDをセット -> %s%d%s", green.c_str(), port_name.c_str(), reset.c_str(), green.c_str(), id, reset.c_str());
              state_ = STANBY;
            } else {
              RCLCPP_WARN(logger, "[ポート:%s%s%s]マイコンを再起動してください。受信データが規定の値と一致しません", green.c_str(), port_name.c_str(), reset.c_str());
            }
          } else {
            RCLCPP_WARN(logger, "[ポート:%s%s%s]マイコンを再起動してください。受信データが規定の値と一致しません。", green.c_str(), port_name.c_str(), reset.c_str());
          }
          break;
        }
        default:
          break;
      }
      receive_bytes.clear();
      decorded_data.clear();
    }
  } else {
    RCLCPP_ERROR(logger, "Serial error: %s", ec.message().c_str());
    return;
  }
  serial.async_read_some(boost::asio::buffer(buffer, 1), [this](const boost::system::error_code &error, size_t bytes_transferred) { this->serial_callback(error, bytes_transferred); });
}

void SerialPort::set_sendmsg(const SendMsgData &send_data) {
  send_msg.clear();
  send_msg = send_data;
}

void SerialPort::send_serial() {
  while (running_) {
    if (serial.is_open()) {
      switch (state_) {
        case CONNECT: {
          std::vector<uint8_t> send_msg_bytes;
          if (!send_msg.float_data.empty()) {
            send_msg_bytes = make_msg(send_msg.float_data);
            boost::asio::write(serial, boost::asio::buffer(send_msg_bytes));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            send_msg.float_data.clear();
          }
          send_msg_bytes.clear();
          if (!send_msg.bool_data.empty()) {
            send_msg_bytes = make_msg(send_msg.bool_data);
            boost::asio::write(serial, boost::asio::buffer(send_msg_bytes));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            send_msg.bool_data.clear();
          }
          break;
        }
        case STANBY: {
          if (id != 0) {
            std::vector<uint8_t> recorl_msg = RECORL_BYTES;
            recorl_msg.push_back(id);
            boost::asio::write(serial, boost::asio::buffer(cobs_encode(recorl_msg)));
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(cobs_encode(recorl_msg).size() * WaitTimePerByte_)));
          }
          break;
        }
        case SETUP: {
          boost::asio::write(serial, boost::asio::buffer(cobs_encode(INTRODUCTION_BYTES)));
          std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(cobs_encode(INTRODUCTION_BYTES).size() * WaitTimePerByte_)));
          break;
        }
        default:
          break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 適切な待機時間を設定
    }
  }
}

SerialManager::SerialManager(const rclcpp::NodeOptions &options) : rclcpp::Node("Serial_manager", options), io() {
  subscription_ = this->create_subscription<interface_pkg::msg::SerialMsg>("send_to_micro", 10, std::bind(&SerialManager::topic_callback, this, std::placeholders::_1));
  publisher_ = this->create_publisher<interface_pkg::msg::SerialMsg>("micro_data", 10);
  port_names = find_serial_port();
  if (!port_names.empty()) {
    RCLCPP_INFO(this->get_logger(), "%s %zu つのシリアルデバイスが接続されています%s", green.c_str(), port_names.size(), reset.c_str());
    for (size_t i = 0; i < port_names.size(); i++) {
      try {
        auto port = std::make_unique<SerialPort>(io, port_names[i], publisher_, this->get_logger());
        serial_ports.emplace(port_names[i], std::move(port));
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize port %s: %s", port_names[i].c_str(), e.what());
      }
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
  serial_ports.clear();
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
  for (std::string &port_name : port_names) {
    auto &it = serial_ports[port_name];
    if (it && it->get_id() == msg.msg_id) {
      send_msg_data.float_data = msg.numbers;
      send_msg_data.bool_data = msg.flags;
      it->set_sendmsg(send_msg_data);
    }
  }
}

}  // namespace serial_manager

RCLCPP_COMPONENTS_REGISTER_NODE(serial_manager::SerialManager)