import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    start_rviz = LaunchConfiguration('start_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    cloud_topic = LaunchConfiguration('cloud_topic')
    scan_raw_topic = LaunchConfiguration('scan_raw_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    lidar_frame = LaunchConfiguration('lidar_frame')
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    map_frame = LaunchConfiguration('map_frame')
    slam_params_file = LaunchConfiguration('slam_params_file')

    slam_share = get_package_share_directory('slam_toolbox')

    arguments = [
        DeclareLaunchArgument(
            'start_rviz', default_value='false',
            description='Start RViz with the slam_toolbox configuration'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description=(
                'Use /clock. Keep false for the current HL2 Mid-360 plugin, '
                'which stamps clouds with the ROS wall clock')),
        DeclareLaunchArgument(
            'cloud_topic', default_value='/sim/raw/mid360_front/points',
            description='Input Mid-360 PointCloud2 topic'),
        DeclareLaunchArgument(
            'scan_raw_topic', default_value='/scan_raw',
            description='Unfiltered LaserScan topic from pointcloud_to_laserscan'),
        DeclareLaunchArgument(
            'scan_topic', default_value='/scan',
            description='Filtered LaserScan topic used by slam_toolbox'),
        DeclareLaunchArgument(
            'lidar_frame', default_value='livox_frame_front',
            description='Front Mid-360 frame'),
        DeclareLaunchArgument(
            'base_frame', default_value='base_link',
            description='Robot base frame used by slam_toolbox'),
        DeclareLaunchArgument(
            'odom_frame', default_value='odom',
            description=(
                'Odometry frame used by slam_toolbox. The current simulation '
                'publishes odom to base_link dynamically')),
        DeclareLaunchArgument(
            'map_frame', default_value='map',
            description='Map frame published by slam_toolbox'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(
                slam_share, 'config', 'mapper_params_online_async.yaml'),
            description='Base slam_toolbox parameter file'),
    ]

    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_livox_frame_front',
        output='screen',
        arguments=[
            '--x', '0.445', '--y', '-0.245', '--z', '0.2495',
            '--qx', '0.00061507597', '--qy', '0.00190281825',
            '--qz', '-0.02060824726', '--qw', '0.99978562758',
            '--frame-id', base_frame,
            '--child-frame-id', lidar_frame,
        ],
    )

    pointcloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='front_mid360_to_scan',
        output='screen',
        remappings=[
            ('cloud_in', cloud_topic),
            ('scan', scan_raw_topic),
        ],
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'target_frame': lidar_frame,
            'transform_tolerance': 0.05,
            'min_height': -0.05,
            'max_height': 0.15,
            'angle_min': -3.14159265359,
            'angle_max': 3.14159265359,
            'angle_increment': 0.01256637061,
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 20.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
    )

    scan_body_filter = Node(
        package='slam_toolbox',
        executable='scan_body_filter.py',
        name='scan_body_filter',
        output='screen',
        remappings=[
            ('scan_in', scan_raw_topic),
            ('scan_out', scan_topic),
        ],
        parameters=[{
            # Static base_link -> livox_frame_front transform used above.
            'base_to_lidar_x': 0.445,
            'base_to_lidar_y': -0.245,
            'base_to_lidar_yaw': -0.0413,
            # HL2 base footprint and the rear rack/body footprint, expressed
            # in base_link coordinates (xmin, xmax, ymin, ymax).
            'body_boxes': [
                -0.55, 0.55, -0.34, 0.34,
                -0.55, 0.55, -1.75, -1.00,
            ],
            'replace_with_inf': True,
        }],
    )

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'odom_frame': odom_frame,
                'map_frame': map_frame,
                'base_frame': base_frame,
                'scan_topic': scan_topic,
                'mode': 'mapping',
                # LaserScan stores range_min as float32, so its nominal 0.1 m
                # is slightly greater than the decimal value. A 0.1 mm margin
                # avoids slam_toolbox's capability-boundary warning.
                'min_laser_range': 0.1001,
                'max_laser_range': 20.0,
                'minimum_time_interval': 0.1,
                'minimum_travel_distance': 0.1,
                'minimum_travel_heading': 0.1,
                'transform_timeout': 0.5,
            },
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='hl2_mapping_rviz',
        output='screen',
        arguments=[
            '-d', os.path.join(
                slam_share, 'config', 'slam_toolbox_default.rviz')
        ],
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool)
        }],
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription([
        *arguments,
        LogInfo(msg=(
            'HL2 mapping publishes base_link -> livox_frame_front as a static '
            'TF and uses odom -> base_link from simulation; vehicle returns are '
            'filtered from /scan.')),
        lidar_static_tf,
        pointcloud_to_scan,
        scan_body_filter,
        slam,
        rviz,
    ])
