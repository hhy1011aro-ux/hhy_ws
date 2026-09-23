from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    package_share = get_package_share_directory("perception_storage_ros2")

    config = os.path.join(
        package_share,
        "config",
        "config.yaml",
    )
    gazebo_launcher = os.path.join(
        package_share,
        "scripts",
        "start_hl2_gazebo.sh",
    )

    # Launch arguments must be included in the returned LaunchDescription.
    launch_arguments = [
        DeclareLaunchArgument(
            "sim_root",
            default_value="/home/ggdz/hhy_ws/sim/hl2_rack_sim_1_4",
            description="Path to the workspace-managed hl2_rack_sim_1_4 project",
        ),
        DeclareLaunchArgument(
            "start_gazebo",
            default_value="true",
            description="Start the HL2 Gazebo world, robot and rack models",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz2 with the perception debug displays",
        ),
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="true",
            description="Start the Gazebo graphical client",
        ),
        DeclareLaunchArgument(
            "gazebo_paused",
            default_value="false",
            description="Start Gazebo with physics paused",
        ),
        DeclareLaunchArgument(
            "use_sim_lidar_topics",
            default_value="true",
            description="Map simulated Mid-360 topics to the original LiDAR parameters",
        ),
        DeclareLaunchArgument(
            "sensor_type",
            default_value="lidar",
            description="Detection sensor type: lidar or tof",
        ),
    ]

    # Launch configurations.
    sim_root = LaunchConfiguration("sim_root")
    start_gazebo = LaunchConfiguration("start_gazebo")
    start_rviz = LaunchConfiguration("start_rviz")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    gazebo_paused = LaunchConfiguration("gazebo_paused")
    use_sim_lidar_topics = LaunchConfiguration("use_sim_lidar_topics")
    sensor_type = LaunchConfiguration("sensor_type")

    # 启动普通 OS 进程 (bash xxx.sh、python xxx.py等)
    gazebo = ExecuteProcess(
        cmd=[
            gazebo_launcher,
            sim_root,
            gazebo_gui,
            gazebo_paused,
        ],
        name="hl2_rack_sim", # 进程 Action 使用的名称
        output="screen",
        condition=IfCondition(start_gazebo),
        sigterm_timeout="2",
        sigkill_timeout="2",
    )

    # 启动 ROS 2 Node (ROS package 中的 executable)
    perception = Node(
        package="perception_storage_ros2",
        executable="perception_storage_node",
        name="perception_storage_node",
        output="screen",
        parameters=[config, {"SettingPara.sensor_type": sensor_type}],
        remappings=[
            ("/livox/lidar_rear", "/sim/raw/mid360_rear/points"),
            ("/livox/lidar_front", "/sim/raw/mid360_front/points"),
            # Keep perception on the raw cloud: its depth image and point cloud
            # must remain pixel-consistent for the geometric filters.
            ("/itof/itof_left_front/tof_frame/pointcloud", "/itof_left_front/points"),
            ("/itof/itof_left_front/tof_frame/depthz", "/itof_left_front/depth/image_raw"),
            ("/itof/itof_left_front/caminfo", "/itof_left_front/depth/camera_info"),
            ("/itof/itof_right_front/tof_frame/pointcloud", "/itof_right_front/points"),
            ("/itof/itof_right_front/tof_frame/depthz", "/itof_right_front/depth/image_raw"),
            ("/itof/itof_right_front/caminfo", "/itof_right_front/depth/camera_info"),
        ],
        condition=IfCondition(use_sim_lidar_topics),
    )

    perception_without_sim_remapping = Node(
        package="perception_storage_ros2",
        executable="perception_storage_node",
        name="perception_storage_node",
        output="screen",
        parameters=[config, {"SettingPara.sensor_type": sensor_type}],
        condition=UnlessCondition(use_sim_lidar_topics),
    )

    tof_noise_left = Node(
        package="perception_storage_ros2",
        executable="tof_pointcloud_noise.py",
        name="tof_left_front_noise",
        output="screen",
        parameters=[{"range_stddev_m": 0.01, "range_bias_m": 0.0,
                     "dropout_probability": 0.0}],
        remappings=[("points_in", "/itof_left_front/points"),
                    ("points_out", "/itof_left_front/points_noisy")],
        condition=IfCondition(use_sim_lidar_topics),
    )
    tof_noise_right = Node(
        package="perception_storage_ros2",
        executable="tof_pointcloud_noise.py",
        name="tof_right_front_noise",
        output="screen",
        parameters=[{"range_stddev_m": 0.01, "range_bias_m": 0.0,
                     "dropout_probability": 0.0}],
        remappings=[("points_in", "/itof_right_front/points"),
                    ("points_out", "/itof_right_front/points_noisy")],
        condition=IfCondition(use_sim_lidar_topics),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="perception_storage_rviz",
        output="screen",
        arguments=["-d", os.path.join(
            package_share,
            "config",
            "perception_storage.rviz",
        )],
        condition=IfCondition(start_rviz),
    )

    # Gazebo's camera/ray plugins publish sensor frames but do not publish the
    # robot's static TF tree. Keep both the configured *_frame names and the
    # model link names available because Gazebo versions differ in PointCloud2
    # frame_id behavior.
    static_tf_specs = [
        ("0.445", "-0.245", "0.2495", "0.00061507597", "0.00190281825", "-0.02060824726", "0.99978562758", "livox_frame_front"),
        ("-0.445", "0.245", "0.2495", "0.0", "0.0", "0.99999976790", "-0.00068132674", "livox_frame_rear"),
        ("0.0", "0.295", "0.265", "0.00000001895", "0.70710677171", "0.70710679066", "0.00000001895", "itof_left_front_frame"),
        ("0.0", "-0.295", "0.265", "0.70710677171", "0.0", "0.0", "0.70710679066", "itof_right_front_frame"),
        ("0.0", "0.295", "0.265", "0.00000001895", "0.70710677171", "0.70710679066", "0.00000001895", "itof_left_front_link"),
        ("0.0", "-0.295", "0.265", "0.70710677171", "0.0", "0.0", "0.70710679066", "itof_right_front_link"),
    ]
    static_tfs = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_base_to_" + child.replace("-", "_"),
            arguments=[
                "--x", values[0], "--y", values[1], "--z", values[2],
                "--qx", values[3], "--qy", values[4], "--qz", values[5],
                "--qw", values[6], "--frame-id", "base_link",
                "--child-frame-id", child,
            ],
            output="screen",
        )
        for values in static_tf_specs
        for child in [values[7]]
    ]

    return LaunchDescription([
        *launch_arguments,
        RegisterEventHandler(
            OnProcessExit(
                target_action=gazebo,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(reason="HL2 Gazebo process exited")
                    )
                ],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=perception,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(reason="Perception storage node exited")
                    )
                ],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=perception_without_sim_remapping,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(reason="Perception storage node exited")
                    )
                ],
            )
        ),
        gazebo,
        perception,
        tof_noise_left,
        tof_noise_right,
        perception_without_sim_remapping,
        *static_tfs,
        rviz,
    ])
