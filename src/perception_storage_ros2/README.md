# perception_storage_ros2

ROS 2 Humble package containing the standalone storage detection pipeline.

The package supports left/right LiDAR and ToF inputs. It intentionally excludes
the ROS 1 service integration, event reporting, monitor, pose tracker and analyzer.

## Run

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select perception_storage_ros2
source install/setup.bash
ros2 launch perception_storage_ros2 run.launch.py
```

The launch file also starts the workspace-managed Gazebo 11 world from
`sim/hl2_rack_sim_1_4`. That world already includes the HL2 base,
drive, lift and rack models. The local ROS 2 Mid-360 plugin is loaded when it
is available; the old ROS 1 Kinetic launch is not required. Useful launch
arguments are:

```bash
# Headless Gazebo
ros2 launch perception_storage_ros2 run.launch.py gazebo_gui:=false start_rviz:=false

# Start only the detector and RViz2
ros2 launch perception_storage_ros2 run.launch.py start_gazebo:=false

# Run only the detector, without starting Gazebo
ros2 launch perception_storage_ros2 run.launch.py start_gazebo:=false use_sim_lidar_topics:=false

# Use a relocated simulation project
ros2 launch perception_storage_ros2 run.launch.py sim_root:=/path/to/hl2_rack_sim_1_4
```

With the default `use_sim_lidar_topics:=true`, the simulator's
`/sim/raw/mid360_front/points` and `/sim/raw/mid360_rear/points` are remapped to
the original front/rear LiDAR parameter topics. Set it to `false` when using
real sensor topics or a different bag layout.

RViz2 loads `config/perception_storage.rviz` and displays the simulated front
and rear point clouds, detection poses, and debug markers.

`SettingPara.sensor_type` selects `lidar` or `tof`.
`SettingPara.sensor_side` selects `left` or `right` when auto start is enabled.

Runtime control uses `std_msgs/msg/String`:

```bash
ros2 topic pub --once /perception_storage/control std_msgs/msg/String "{data: start_left}"
ros2 topic pub --once /perception_storage/control std_msgs/msg/String "{data: start_right}"
ros2 topic pub --once /perception_storage/control std_msgs/msg/String "{data: stop}"
```
