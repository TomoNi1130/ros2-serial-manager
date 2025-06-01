複数のマイコンを同時に管理するためのもの.
マイコンはIDで管理している.
serial_msnager_testと同時に使おう  
interface_pkg::msg::SerialMsg型のsend_to_microトピックに投げられたものをすべて送る.
送られてきたデータは同型のmicro_dataトピックにて共有される.

シリアルの送受信にはboost/asioを用いている.

-導入-
colcon build
. install/setup.bash
ros2 launch serial_manager_pkg serial_manager.launch.py