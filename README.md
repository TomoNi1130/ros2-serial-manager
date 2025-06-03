# 複数マイコン同時管理システム

このパッケージは、複数のマイコンを同時に管理するためのものです。  
各マイコンはIDで管理されています。  
`serial_msnager_test`と同時に使用してください。

- `serial_manager_pkg::msg::SerialMsg` 型の `send_to_micro` トピックに送信されたデータは、すべてマイコンに送られます。
- 受信したデータは同型の `micro_data` トピックで共有されます。

## シリアル通信

シリアルの送受信には **boost/asio** を使用しています。

## 注意

他のシリアル監視系ツールが動作していると通信品質が悪化します。  
特に VSCode のシリアルモニターを開きっぱなしにしないよう注意してください。

## 導入

- boost/asio が必要です。未インストールの場合は各自で導入してください。

## 起動方法

```sh
colcon build
. [setup.bash](http://_vscodecontentref_/0)
ros2 launch serial_manager_pkg serial_manager.launch.py
# または
ros2 run serial_manager_pkg serial_manager_node
```

## 使用方法

1. マイコンと接続する
2. ノードを起動する
3. `send_to_micro` トピックに `serial_manager_pkg::msg::SerialMsg` を送信する
4. `micro_data` トピックを同型でサブスクライブする