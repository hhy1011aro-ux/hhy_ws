# hhy_ws 工作空间说明

## 基本环境

- 工作空间：`/home/ggdz/hhy_ws`
- ROS 版本：ROS 2 Humble
- Gazebo：Gazebo Classic 11
- 系统：Ubuntu 22.04 / Jammy
- 默认 shell：bash
- 构建工具：`colcon`

## 主要目录

```text
hhy_ws/
├── src/perception_storage_ros2/       ROS2 检测算法功能包
├── src/cartographer/
├── src/cartographer_ros/
├── src/slam_toolbox-ros2/
├── bag/                               bag包储存目录
├── maps/                              地图保存、点云处理目录
├── docs/                              文档保存目录
├── build/                             colcon 构建目录
├── install/                           ROS2 安装目录
├── log/                               colcon 日志
└── bag/                               rosbag 数据
```

## ROS2 检测包

功能包：`perception_storage_ros2`

核心代码：

- `src/perception_storage_ros2/src/perception_storage.cpp`：检测主流程，保留原 `running()` 全链路逻辑；
- `src/perception_storage_ros2/src/lidar_detector.cpp`：雷达检测辅助算法；
- `src/perception_storage_ros2/src/tof_detector.cpp`：ToF 检测辅助算法；
- `src/perception_storage_ros2/config/config.yaml`：检测参数；
- `src/perception_storage_ros2/launch/run.launch.py`：检测节点、Gazebo、TF 和 RViz 启动文件；
- `src/perception_storage_ros2/scripts/start_hl2_gazebo.sh`：由 ROS2 launch 调用的 Gazebo 启动脚本。

构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select perception_storage_ros2
source install/setup.bash
```

## 已知限制和警告

- 检测算法遇到空点云时原代码可能触发 PCL KDTree 错误，除非用户明确要求，否则不要擅自修改算法保护逻辑。

## maps 点云脚本

- `maps/pcd_ground_align.py`：拟合地面并将点云旋转、平移到地面坐标系。

```bash
python3 maps/pcd_ground_align.py input.pcd output.pcd \
  --ground-candidate-percentile 20
```

- `maps/pcd_to_pgm.py`：将 binary PCD 投影为 ROS 使用的 `.pgm` 和 `.yaml` 地图；可用 `--z-min`、`--z-max` 限制投影高度。

```bash
python3 maps/pcd_to_pgm.py input.pcd output_base \
  --resolution 0.05 --z-min 0.1 --z-max 2.0
```

- `maps/pcd_io.py`：供其他脚本调用的 binary PCD 读写模块，不单独运行。

## 修改原则

- `src/perception_storage_ros2/` 等其他目录下有对应的 `README.md`和其他`.md`文件；处理这些目录中的文件时，应先参考对应 README，完成修改后同步更新 README；
- 保留原检测算法变量名和主流程；
- 不重新引入 `hecbot_srvs`、monitor、pose tracker 或外部业务服务；
- 修改仿真模型前，确认是否会影响检测点云、TF 或运动插件；
- 对用户未要求的算法行为不要主动修改；
- 运行长时间仿真或录包前，确认没有旧 Gazebo/ROS2 节点占用端口和仿真锁。

## slam_toolbox 建图（ROS 2 Humble）

### 安装和构建

- 源码：`src/slam_toolbox-ros2/`；当前使用 ROS 2 Humble 兼容版本。
- 已安装 `pointcloud_to_laserscan` Humble 包，节点为 `pointcloud_to_laserscan_node`。
- 构建命令：

```bash
cd /home/ggdz/hhy_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select slam_toolbox
source install/setup.bash
```
