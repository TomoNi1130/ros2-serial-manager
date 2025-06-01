複数のマイコンを同時に管理するためのもの.
マイコンはIDで管理している.
serial_msnager_testと同時に使おう  
interface_pkg::msg::SerialMsg型のsend_to_microトピックに投げられたものをすべて送る.
送られてきたデータは同型のmicro_dataトピックにて共有される.

シリアルの送受信にはboost/asioを用いている.

-注意-
他のシリアル監視系ツールがあると通信品質が悪くなるので避けましょう(特にvscodeのシリアルモニターつけっぱとか)

-導入-
boost/asioが必要です。
インストールが必要であればしましょう。調べてくれ

-初ビルド時
colcon build --packages-select interface_pkg
. install/setup.bash
colcon build

-起動時
colcon build
. install/setup.bash
ros2 launch serial_manager_pkg serial_manager.launch.py or ros2 run serial_manager_pkg serial_manager_node