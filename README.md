환경은 우분투 22.04 ROS2 Humble입니다

colcon build로 빌드하면 되고 각각 명령어는

ros2 run rgt_solution main_2

ros2 run rgt_solution main_3

입니다.

main_2는 터미널을 따로 켜서 수동으로 ros2 topic pub /raw_temperature std_msgs/msg/Float64 "{data: 26.1}" 토픽을 송출하면 됩니다

*2026.01.11 토픽 자동발행 파이썬 노드 추가 파이썬 과제는 수행하지 않았으니 이거라도

main1_은 어차피 로그 만들기이고 ros2는 이미 로그 시스템이 있어서 굳이 올리지 않았습니다.

감사합니다
