# 仕様

このパッケージは、複数のマイコンを同時に管理するためのもの　　
各マイコンはIDで管理されている。  
`serial_msnager_test`https://github.com/TomoNi1130/serial_manager_test/blob/main/README.mdと同時に使用する。

- `serial_manager_pkg::msg::SerialMsg` 型の `send_to_micro` トピックに送信されたデータは、対応したIDを持つマイコンに送られる。
- 受信したデータは同型の `micro_data` トピックで共有される。

## シリアル通信

シリアルの送受信には **boost/asio** を使用し、cobs形式で行っている。

## 注意

他のシリアル監視系ツールが動作していると通信品質が悪化します。  
(例　VSCodeのシリアルモニター等)

## 導入

- boost/asio が必要。未インストールの場合は各自で導入してくれ。

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