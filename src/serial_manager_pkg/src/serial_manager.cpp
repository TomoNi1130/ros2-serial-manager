#include "serial_manager.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace serial_manager {

SerialPort::SerialPort (boost::asio::io_context &io, const std::string &port_name, rclcpp::Publisher<serial_manager_pkg::msg::SerialMsg>::SharedPtr publisher_, const rclcpp::Logger &logger) : serial (io), port_name (port_name), id (0), publisher_ (publisher_), logger (logger), running_ (true) {
  state_ = SETUP;
  atomic_state_.store (SETUP);

  serial = boost::asio::serial_port (io, port_name);
  serial.set_option (boost::asio::serial_port_base::baud_rate (115200));
  serial.async_read_some (boost::asio::buffer (buffer, 1), [this] (const boost::system::error_code &error, size_t bytes_transferred) { this->serial_callback (error, bytes_transferred); });

  send_msg_thread = std::thread ([this] () { SerialPort::send_serial (); });
  heartbeat_thread = std::thread ([this] () { SerialPort::heartbeat (); });

  RCLCPP_INFO (logger, "[ポート:%s%s%s] セットアップ完了", green.c_str (), port_name.c_str (), reset.c_str ());
}

SerialPort::~SerialPort () {
  shutdown ();
}

void SerialPort::shutdown () {
  if (!running_.exchange (false)) {
    return;  // 既にシャットダウン済み
  }

  RCLCPP_INFO (logger, "[ポート:%s%s%s] シャットダウン開始", green.c_str (), port_name.c_str (), reset.c_str ());

  // スレッドを起こす
  cv_.notify_all ();

  // シリアルポートをキャンセル・クローズ
  if (serial.is_open ()) {
    boost::system::error_code ec;
    serial.cancel (ec);
    serial.close (ec);
  }

  // スレッドの終了を待つ
  if (send_msg_thread.joinable ()) {
    send_msg_thread.join ();
  }
  if (heartbeat_thread.joinable ()) {
    heartbeat_thread.join ();
  }

  RCLCPP_INFO (logger, "[ポート:%s%s%s] シャットダウン完了", green.c_str (), port_name.c_str (), reset.c_str ());
}

int SerialPort::get_id () {
  return id;
}

std::vector<uint8_t> cobs_decode (const std::vector<uint8_t> &input) {
  std::vector<uint8_t> decoded_data;
  uint8_t OBH;
  OBH = input[0];
  for (uint8_t i = 1; i < input.size (); i++) {
    if (i == OBH) {
      OBH = input[i] + OBH;
      decoded_data.push_back (0x00);
    } else {
      decoded_data.push_back (input[i]);
    }
  }
  decoded_data.pop_back ();
  return decoded_data;
}

void SerialPort::serial_callback (const boost::system::error_code &ec, std::size_t bytes_transferred) {
  (void)bytes_transferred;

  if (!running_) {
    return;  // シャットダウン中は処理しない
  }

  if (!ec) {
    uint8_t buf = buffer[0];
    receive_bytes.push_back (buf);

    if (buf == 0x00) {
      uint8_t type_keeper = 0;
      if (state_ == CONNECT) {
        type_keeper = receive_bytes[0];
        receive_bytes.erase (receive_bytes.begin ());
      }
      decoded_data = cobs_decode (receive_bytes);

      switch (state_) {
        case CONNECT: {
          if (type_keeper == serial_manager::FLOAT_HEADER) {
            std::vector<float> results;
            for (size_t i = 0; i < decoded_data.size () / sizeof (float); i++) {
              uint8_t raw[4] = {decoded_data[i * sizeof (float) + 0], decoded_data[i * sizeof (float) + 1], decoded_data[i * sizeof (float) + 2], decoded_data[i * sizeof (float) + 3]};
              float result;
              std::memcpy (&result, raw, sizeof (float));
              results.push_back (result);
            }
            if (!results.empty ()) {
              pub_msg_data_.msg_id = id;
              pub_msg_data_.numbers.clear ();
              pub_msg_data_.numbers = results;
              publisher_->publish (pub_msg_data_);
            }
          } else if (type_keeper == serial_manager::BOOL_HAEDER) {
            std::vector<bool> results;
            for (size_t i = 0; i < decoded_data.size (); i++) {
              if (decoded_data[i] == 0x01)
                results.push_back (true);
              else if (decoded_data[i] == 0x00)
                results.push_back (false);
            }
            if (!results.empty ()) {
              pub_msg_data_.msg_id = id;
              pub_msg_data_.flags.clear ();
              pub_msg_data_.flags = results;
              publisher_->publish (pub_msg_data_);
            }
          } else if (type_keeper == serial_manager::LOG_HEADER) {
            std::string log_msg (decoded_data.begin (), decoded_data.end ());
            if (!log_msg.empty ()) RCLCPP_INFO (logger, "[ポート:%s%s%s ID:%s %d %s] log: %s", green.c_str (), port_name.c_str (), reset.c_str (), green.c_str (), id, reset.c_str (), log_msg.c_str ());
          } else if (type_keeper == HEART_BEAT_HEADER) {
            if (decoded_data == HEARTBEAT_BYTES) {
              last_heartbeat_time = clock.now ();
            }
          }
          break;
        }
        case STANBY: {
          if (decoded_data == START_COM_BYTES) {
            state_ = CONNECT;
            atomic_state_.store (CONNECT);
            cv_.notify_all ();  // 状態変化を通知
            RCLCPP_INFO (logger, "[ポート:%s%s%s] マイコンとの通信を開始しました。", green.c_str (), port_name.c_str (), reset.c_str ());
          } else {
            RCLCPP_WARN (logger, "[ポート:%s%s%s] 接続中...", green.c_str (), port_name.c_str (), reset.c_str ());
          }
          break;
        }
        case SETUP: {
          if (decoded_data.size () == INTRODUCTION_BYTES.size () + 1) {
            if (std::equal (INTRODUCTION_BYTES.begin (), INTRODUCTION_BYTES.end (), decoded_data.begin ())) {
              decoded_data.erase (decoded_data.begin (), decoded_data.begin () + INTRODUCTION_BYTES.size ());
              int pre_id = id;
              id = decoded_data[0];
              RCLCPP_INFO (logger, "[ポート:%s%s%s] マイコンIDをセット -> %s%d%s", green.c_str (), port_name.c_str (), reset.c_str (), green.c_str (), id, reset.c_str ());
              state_ = STANBY;
              atomic_state_.store (STANBY);
              cv_.notify_all ();  // 状態変化を通知
            }
          }
          break;
        }
        default:
          break;
      }
      receive_bytes.clear ();
      decoded_data.clear ();
    }
  } else if (ec == boost::asio::error::operation_aborted) {
    // シャットダウン時のキャンセル
    return;
  } else {
    RCLCPP_ERROR (logger, "マイコンとの通信が途切れました: %s", ec.message ().c_str ());
    return;
  }

  if (running_ && serial.is_open ()) {
    serial.async_read_some (boost::asio::buffer (buffer, 1), [this] (const boost::system::error_code &error, size_t bytes_transferred) { this->serial_callback (error, bytes_transferred); });
  }
}

void SerialPort::set_sendmsg (const SendMsgData &send_data) {
  std::lock_guard<std::mutex> lock (send_msg_mutex_);
  send_msg.clear ();
  send_msg = send_data;
}

void SerialPort::send_serial () {
  while (running_) {
    if (!serial.is_open () || !running_) {
      std::this_thread::sleep_for (std::chrono::milliseconds (10));
      continue;
    }

    switch (state_) {
      case CONNECT: {
        std::vector<uint8_t> send_msg_bytes;
        {
          std::lock_guard<std::mutex> lock (send_msg_mutex_);
          if (!send_msg.float_data.empty ()) {
            send_msg_bytes = make_msg (send_msg.float_data);
            boost::asio::write (serial, boost::asio::buffer (send_msg_bytes));
            std::this_thread::sleep_for (std::chrono::milliseconds (10));
            send_msg.float_data.clear ();
          }
          send_msg_bytes.clear ();
          if (!send_msg.bool_data.empty ()) {
            send_msg_bytes = make_msg (send_msg.bool_data);
            boost::asio::write (serial, boost::asio::buffer (send_msg_bytes));
            std::this_thread::sleep_for (std::chrono::milliseconds (10));
            send_msg.bool_data.clear ();
          }
        }
        break;
      }
      case STANBY: {
        std::vector<uint8_t> recorl_msg = RECORL_BYTES;
        recorl_msg.push_back (uint8_t (id));
        boost::asio::write (serial, boost::asio::buffer (cobs_encode (recorl_msg)));
        RCLCPP_INFO (logger, "[ポート:%s%s%s] マイコンIDの再確認中...", green.c_str (), port_name.c_str (), reset.c_str ());

        // 応答を待つ (タイムアウト付き)
        std::unique_lock<std::mutex> lock (cv_mutex_);
        cv_.wait_for (lock, std::chrono::milliseconds (100), [this] () { return !running_ || state_ != STANBY; });
        break;
      }
      case SETUP: {
        boost::asio::write (serial, boost::asio::buffer (cobs_encode (INTRODUCTION_BYTES)));
        RCLCPP_INFO (logger, "[ポート:%s%s%s] マイコンIDを探索中...", green.c_str (), port_name.c_str (), reset.c_str ());

        // 応答を待つ (タイムアウト付き)
        std::unique_lock<std::mutex> lock (cv_mutex_);
        cv_.wait_for (lock, std::chrono::milliseconds (100), [this] () { return !running_ || state_ != SETUP; });
        break;
      }
      default:
        break;
    }

    std::this_thread::sleep_for (std::chrono::milliseconds (1));
  }
}

void SerialPort::heartbeat () {
  while (running_) {
    // 接続状態になるまで待機 (タイムアウト付き)
    {
      std::unique_lock<std::mutex> lock (cv_mutex_);
      cv_.wait_for (lock, std::chrono::milliseconds (100), [this] () { return !running_ || state_ == CONNECT; });
    }

    if (!running_) break;

    if (state_ == CONNECT) {
      last_heartbeat_time = clock.now ();
    }

    std::vector<uint8_t> heartbeat_msg;
    now = clock.now ();

    if (state_ == CONNECT && now - last_heartbeat_time > rclcpp::Duration::from_seconds (0.5)) {
      RCLCPP_INFO (logger, "[ポート:%s%s%s] 接続が途絶えました。再接続を試みます", green.c_str (), port_name.c_str (), reset.c_str ());
      state_ = SETUP;
      atomic_state_.store (SETUP);
      cv_.notify_all ();
    }

    if (running_ && serial.is_open () && state_ == CONNECT) {
      heartbeat_msg.push_back (HEART_BEAT_HEADER);
      std::vector<uint8_t> heartbeat_bytes = cobs_encode (HEARTBEAT_BYTES);
      heartbeat_msg.insert (heartbeat_msg.end (), heartbeat_bytes.begin (), heartbeat_bytes.end ());
      boost::asio::write (serial, boost::asio::buffer (heartbeat_msg));
    }

    std::this_thread::sleep_for (std::chrono::milliseconds (100));
  }
}

SerialManager::SerialManager (const rclcpp::NodeOptions &options) : rclcpp::Node ("Serial_manager", options), io () {
  subscription_ = this->create_subscription<serial_manager_pkg::msg::SerialMsg> ("send_to_micro", 10, std::bind (&SerialManager::topic_callback, this, std::placeholders::_1));
  publisher_ = this->create_publisher<serial_manager_pkg::msg::SerialMsg> ("micro_data", 10);
  port_names = find_serial_port ();

  if (!port_names.empty ()) {
    RCLCPP_INFO (this->get_logger (), "%s %zu つのシリアルデバイスが接続されています%s", green.c_str (), port_names.size (), reset.c_str ());
    for (size_t i = 0; i < port_names.size (); i++) {
      try {
        auto port = std::make_unique<SerialPort> (io, port_names[i], publisher_, this->get_logger ());
        serial_ports.emplace (port_names[i], std::move (port));
      } catch (const std::exception &e) {
        RCLCPP_ERROR (this->get_logger (), "Failed to initialize port %s: %s", port_names[i].c_str (), e.what ());
      }
    }
    io_thread_ = std::thread ([this] () { io.run (); });
  } else {
    RCLCPP_ERROR (this->get_logger (), "%sポートが接続されてないやんけ!%s", red.c_str (), reset.c_str ());
    rclcpp::shutdown ();
    return;
  }
}

SerialManager::~SerialManager () {
  RCLCPP_INFO (this->get_logger (), "SerialManager シャットダウン開始");

  // 各ポートをシャットダウン
  for (auto &port_pair : serial_ports) {
    if (port_pair.second) {
      port_pair.second->shutdown ();
    }
  }
  serial_ports.clear ();

  // io_contextを停止
  io.stop ();

  if (io_thread_.joinable ()) {
    io_thread_.join ();
  }

  RCLCPP_INFO (this->get_logger (), "SerialManager シャットダウン完了");
}

std::vector<std::string> SerialManager::find_serial_port () {
  std::vector<std::string> device_paths;
  for (const auto &entry : std::filesystem::directory_iterator ("/dev"))
    if (entry.is_character_file () || entry.is_block_file () || entry.is_symlink ()) {
      std::string filename = entry.path ().filename ().string ();
      if (filename.rfind ("ttyACM", 0) == 0) device_paths.push_back (entry.path ().string ());
    }
  return device_paths;
}

void SerialManager::topic_callback (const serial_manager_pkg::msg::SerialMsg &msg) {
  SendMsgData send_msg_data;
  for (std::string &port_name : port_names) {
    auto &it = serial_ports[port_name];
    if (it && it->get_id () == msg.msg_id || it && it->get_id () == 0) {
      send_msg_data.float_data = msg.numbers;
      send_msg_data.bool_data = msg.flags;
      it->set_sendmsg (send_msg_data);
    }
  }
}

}  // namespace serial_manager

RCLCPP_COMPONENTS_REGISTER_NODE (serial_manager::SerialManager)