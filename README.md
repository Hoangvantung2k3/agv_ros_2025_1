

hướng dẫn chạy thử :
- mở terminal
- cd nơi muốn lưu file 
- git clone link
- lưu ý đây là bao gồm toàn bộ workspace nên nếu cần thì tự tạo workspace khác xong cop sang 
- catkin_make_isolated
- source devel/setup.bash

- model mới : roslaunch carmodel_description spawn.launch
- rosrun carmodel_description o + tab + .py
- rosrun teleop_twist_keyboard teleop_twist_keyboard.py : lệnh này chạy cho nhanh, thay vì roslaunch file teleop (rosrun carmodel_teleop carmodel_teleop_key.launch)
- khởi chạy PID :   rosrun carmodel_description pid_control_test



mở rviz : đã khởi chạy luôn trong file launch
xem thông số tọa độ thông qua camera : rostopic echo /T265_camera/odom/sample 
chuyển đổi topic của T265 sang /odom: rosrun topic_tools relay /T265_camera/odom/sample /odom
lưu ý: nhiều file cần cài thêm thư viện khi chạy, trong trường hợp trong máy ko có, post lỗi hiển thị ( khi khởi động ) lên mạng, sẽ có lệnh cài : sudo ... ( thường thiếu ở navigation dùng để chạy trong rviz)

chạy gmapping để tạo map trong rviz:
roscore
roslaunch carmodel_description spawn.launch 
rosrun gmapping slam_gmapping scan:=/m2wr/laser/scan _base_frame:=base_link _odom_frame:=odom
lưu trữ map rviz sau khi quét: rosrun map_server map_saver -f ~/your_map_name




AGV_IVSR: Phần mềm RPI, chạy pid 
Chạy:
roslaunch carmodel_description spawn.launch

Tạo xe trong Gazebo (và RViz)
roslaunch offboard offboard.launch

Chạy chế độ offboard, xe sẽ di chuyển đến vị trí được định nghĩa trong tệp khởi động.
Lưu ý:
Nếu bạn đang chạy trong môi trường mô phỏng, hãy đảm bảo rằng sim_mode và arm_mode được đặt là true.
Trước khi chạy bước thứ hai trong phần Chạy, bạn có thể chạy:
rosrun offboard odom_to_csv.py
Lệnh này dùng để ghi lại dữ liệu odometry vào tệp CSV, hãy nhớ chỉnh sửa đường dẫn trong tệp odom_to_csv.py cho phù hợp với môi trường của bạn.
Sau đó, bạn có thể sử dụng rosrun plot_juggler plot_juggler để quan sát dữ liệu.
rosrun offboard odom_to_path.py để công bố đường đi của odom theo thời gian thực trong khi xe đang di chuyển.

list lỗi: warning ở khoảng tần số tf, có khả năng ở các file gazebo, tần số xuất bản như kiểu 'odomertyRate'



sửa model: lưu ý chỉnh sensor ở link_joint.xacro
xóa base_footprint ở example.urdf.xacro, chỉnh model velodyne 3d cũng nằm ở đây luôn
nhảy tưng tưng là do cái sensor 2d =)))