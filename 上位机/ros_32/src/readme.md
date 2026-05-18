 ros2 run example_topic_rclpy topic_publisher_02


 ros2 run example_topic_rclpy topic_subscribe_02 
 ros2 launch fishbot_bringup fishbot_bringup.launch.py 

ros2 launch rplidar_ros rplidar_a3_launch.py

ros2 launch slam_toolbox online_async_launch.py 

pkill -9 -f ros2
pkill -9 -f slam_toolbox
ros2 topic echo /odom | grep -E "x:|y:"
colcon build --symlink-install

ros2 launch fishbot_navigation2 navigation2.launch.py

source ros_32/install/setup.bash


ros2 run nav2_map_server map_saver_cli -t map -f fishbot_map
