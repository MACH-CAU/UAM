
ros2 run px4_ros_com offboard_mavros

ros2 run px4_ros_com key_event_pub



갠2

# VTOL 환경 변수
source ~/VTOL/ws_sensor_combined_src/install/setup.bash


colcon build --packages-select px4_ros_com 
colcon build --packages-select px4_ros_com && source ~/VTOL/ws_sensor_combined_src/install/setup.bash
# 


PX4_GZ_WORLD=sensor make px4_sitl gz_standard_vtol
