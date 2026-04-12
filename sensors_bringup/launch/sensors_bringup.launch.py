import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir = get_package_share_directory('sensors_bringup')
    ekf_config = os.path.join(pkg_dir, 'config', 'ekf.yaml')

    rs_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch', 'rs_launch.py'
    )

    set_ld_library_path = SetEnvironmentVariable(
        name='LD_LIBRARY_PATH',
        value='/opt/ros/humble/lib/aarch64-linux-gnu:' + os.environ.get('LD_LIBRARY_PATH', '')
    )

    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch_path),
        launch_arguments={
            'enable_gyro':                    'true',
            'enable_accel':                   'true',
            'unite_imu_method':               '2',
            'enable_color':                   'false',
            'enable_depth':                   'true',
            'depth_module.depth_profile':     '424x240x30',
            'depth_module.infra_profile':     '424x240x30',
            'enable_infra1':                  'true',
            'enable_infra2':                  'true',
        }.items(),
    )

    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf',
        arguments=[
            '0.1',  '0.0', '0.05',
            '0.0',  '0.0', '0.0',
            'base_link', 'camera_link'
        ],
    )

    imu_override_node = Node(
        package='sensors_bringup',
        executable='imu_covariance_override',
        name='imu_covariance_override',
        output='screen',
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'print_diagnostics': True}],
        remappings=[
            ('odometry/filtered', '/odom_ekf'),
        ],
    )

    return LaunchDescription([
        set_ld_library_path,
        realsense_node,
        camera_tf,
        imu_override_node,
        ekf_node,
    ])