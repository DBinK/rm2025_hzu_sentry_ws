# 启动一号雷达 #
ros2 launch ydlidar_ros2_driver x3_ydlidar_launch.py

# 启动二号雷达 #
ros2 launch ydlidar_ros2_driver x3_ydlidar_launch_2.py

# 启动双雷达数据处理 #
ros2 launch map_tools merged_scan.launch.py

# 静态坐标系变换 #
ros2 launch map_tools tf_static_launch.py

# slam建图 #
ros2 launch slam_gmapping test_gmapping.launch.py 

# 开启里程计 #
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py

# 打开amcl #
ros2 launch map_tools amcl_launch.py


# 上发整合版（无amcl）#
ros2 launch turtlebot3_navigation2 test_all.launch.py


# 地图保存 #
ros2 run nav2_map_server map_saver_cli -t map -f slam_map
<!-- ros2 run nav2_map_server map_saver_cli -t map -f slam_map --free_threshold 64 --occupied_threshold 165 --image_format pgm -->

# 载入地图 #
ros2 run nav2_map_server map_server --ros-args --param yaml_filename:=slam_map.yaml

# 地图配置 #
ros2 lifecycle set /map_server configure

# 地图激活 #
ros2 lifecycle set /map_server activate

# 可视化界面 #
rviz2

# 查看tf树 #
ros2 run rqt_tf_tree rqt_tf_tree --force-discover

# 编译 #
colcon build --packages-select map_tools
<!-- colcon build -->

# 导航 #
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=False slam:=False map:=/home/morefine/rm2025_hzu_sentry_ws/src/map_tools/maps/slam_map.yaml

# 导航数据打包成自定义消息 #
ros2 run map_tools cmd_vel2serial.py

# 开启串口 #
ros2 run rm_serial_python rm_serial_node 


