환경은 우분투 22.04 ROS2 Humble입니다

colcon build로 빌드하면 되고 각각 명령어는

ros2 run rgt_solution main_2

ros2 run rgt_solution main_3

입니다.

main2는 터미널을 따로 켜서 수동으로 ros2 topic pub /raw_temperature std_msgs/msg/Float64 "{data: 26.1}" 토픽을 송출하면 됩니다

감사합니다
