# 货架感知与定位

本 ROS1 包使用 Livox 激光雷达或 iToF 数据检测、跟踪货架/储物柜位姿，并发布 `geometry_msgs/PoseStamped` 目标位姿以及调试点云、图像、候选、证据和跟踪状态。

## 入口

- `perception_storage_node`：提供检测服务，完成传感器数据接入、多帧预处理、分割、位姿精修、结果发布和位姿跟踪。
- `monitor`：iToF 货架与碰撞监测节点。
- `launch/run.launch`：加载 `config/config.yaml`，以 respawn 模式启动 `perception_storage_node`。

## 传感器

- Livox 前后点云：`/livox/lidar_front`、`/livox/lidar_rear`。
- `/itof/...` 下左右前方 iToF 点云、深度图和相机信息。
- `SettingPara/sensor_type` 选择当前检测器；现有配置使用 `lidar`。
- 当前 iToF 批大小为 1 帧，激光雷达批大小为 20 帧。

## 公共 ROS 接口

- 服务：`/hecbot_perception/storage_detect`，类型为 `hecbot_srvs/JString`。
  - 请求 JSON 存放在 `request` 字段中。
  - `detect_op` 为 `start` 或 `stop`。
  - 启动请求使用 `side_option`（`left` 或 `right`）和包含 `x`、`y`、`th` 的 `reference_pose`。
- 主要结果：`/hecbot_perception/storage_result`，类型为 `geometry_msgs/PoseStamped`。
- 主要激光调试输出：`/perception/capture/points`，类型为 `sensor_msgs/PointCloud2`。
- 精调原始点调试：`/perception/debug/refine/raw_points`，类型为 `sensor_msgs/PointCloud2`。
- 精调投影点调试：`/perception/debug/refine/projected_points`，类型为 `sensor_msgs/PointCloud2`。
  - intensity `0` 表示标签点，`1` 表示长边点，`2` 表示短边点。
- 参考短线聚类调试：`/perception/debug/reference/cloud_cluster`，类型为 `sensor_msgs/PointCloud2`；发布RANSAC拟合前且长度初筛合格的完整聚类点。
- 跟踪命令：`/hecbot_perception/storage_goal_pose`。
- 跟踪输出位于 `/perception/storage/tracker/*`。
- 其他可配置调试输出定义在 `config/config.yaml` 的 `PubInfo` 中。

除非用户明确要求，否则不要修改公共 ROS 服务名、话题名、消息类型、JSON 键或坐标系约定。如果必须修改接口，应同步更新全部生产者、消费者、launch/config 文件和依赖包。

## TF 约定

- 传感器坐标系来自 `FrameNames`：`livox_frame_front`、`livox_frame_rear`、`itof_left_front_frame`、`itof_right_front_frame`。
- 检测结果保留被处理传感器消息的坐标系。
- 位姿跟踪器要求输入传感器坐标系与 `odom` 之间存在有效变换。
- 保持目标位姿局部轴定义：`y_v` 沿检测到的长边，`z_v` 为竖直方向，`x_v = y_v × z_v` 为目标水平法向。

## 检测约定和已知注意事项

- 当前 YAML/API 中的 `LidarSensors/range_compenstion` 是既有拼写。它被读入 `SensorInfo::distance_compensation`；不要静默改名。
- 距离单位为米。当前激光补偿为 `0.05`，最终目标中心移动量为 `-distance_compensation * x_v`；它不会修改每个原始激光点的量程。
- `lidar_detector` 使用硬编码的 `free_distance = 0.495` 米进行短边校验和位置精修。
- 激光长边和标签线在 RANSAC 选出内点后使用 `optimizeModelCoefficients()` 重新估计系数；标签中心使用优化直线重新筛选候选点，并取主方向 Q2/Q98 端点中点。`naive_line()` 其他调用点默认不开启系数优化。
- 有效精修结果必须通过长边角度和距离检查并建立 `large_condition`。短边证据会增加置信度并可能精修位置，但长边结果有效后，短边不是结果发布的必要条件。
- 无结果可能发生在进入 `finetuning_filter()` 之前。诊断检测丢失时，至少区分：深度候选、映射后的非空 patch、几何过滤、支撑线、标签长度、长短边角度和距离、置信度以及最终方向检查。
- 使用 `SegmentationSummary`、`HypothesisMapping` 和 `InferenceSummary` 区分深度候选为空、像素无法映射到点云、几何过滤失败以及精调失败；放宽几何阈值前应先根据这些日志定位。
- 多处参数读取目前使用未检查的 `getParam()`。修改参数处理时保留现有键名，并优先使用已初始化默认值和返回值检查。

## 验证

- 优先在 ROS1 Kinetic 环境中对 `perception_storage` 做包级构建，不要默认重建整个工作区。
- 修改检测逻辑后，验证服务启停、左右传感器、输出坐标系、四元数归一化、结果可用率、位置/航向稳定性以及输入到结果的延迟。
- bag 和其他二进制录制数据只读。离线回放必须使用隔离的 ROS master/容器，避免影响正在运行的机器人或仿真程序。
