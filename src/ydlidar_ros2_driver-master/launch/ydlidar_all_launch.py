from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取 ydlidar_ros2_driver 包的共享目录路径
    ydlidar_package_dir = get_package_share_directory('ydlidar_ros2_driver')

    # 创建包含 x3_ydlidar0_launch.py 的 IncludeLaunchDescription
    ydlidar0_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ydlidar_package_dir, '/launch/ydlidar0_launch.py'])
    )

    # 创建包含 4ros_ydlidar1_launch.py 的 IncludeLaunchDescription
    ydlidar1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ydlidar_package_dir, '/launch/x3_ydlidar1_launch.py'])
    )

    # 返回整合后的 LaunchDescription
    return LaunchDescription([
        ydlidar1_launch,
        ydlidar0_launch
    ])
