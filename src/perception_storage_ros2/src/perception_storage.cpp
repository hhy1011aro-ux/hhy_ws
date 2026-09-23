#include "perception_storage_ros2/perception_storage.hpp"

PerceptionStorage::PerceptionStorage()
  : rclcpp::Node("perception_storage_node"), cam_left_ready(false), cam_right_ready(false)
{
  init();
  if(config_info.m_settinginfo.sensor_type==SensorType::TOF)
  {
    detect_machine=std::make_shared<tof_detector>(config_info);
  }
  else if(config_info.m_settinginfo.sensor_type==SensorType::LIDAR)
  {
    detect_machine=std::make_shared<lidar_detector>(config_info);
  }
  if(auto_start)
  {
    std::vector<float> pose_{0.0f,0.0f,0.0f};
    start(sensor_side,pose_);
  }
  worker_running=true;
  threads_pool.push_back(std::make_shared<std::thread>(&PerceptionStorage::running, this));
}

PerceptionStorage::~PerceptionStorage()
{
  worker_running=false;
  for (size_t i=0; i != threads_pool.size(); i++)
  {
    if(threads_pool[i] && threads_pool[i]->joinable())
    {
      threads_pool[i]->join();
    }
  }
}

void PerceptionStorage::init()
{
  if(!read_params())
  {
    throw std::runtime_error("invalid perception_storage parameters");
  }
  active_=ActiveState::SLEEPY;
  set_processor();
  set_publishers();
  msg_packet.clear();
  process_queue.clear();
  job_num=0;
  valid_hys=0;
  valid_re=0;
  filter_machine=std::make_shared<points_filter>();
}

bool PerceptionStorage::read_params()
{
  std::string topic_lidar_left=this->declare_parameter<std::string>("LidarSensors.points_left","/livox/lidar_rear");
  std::string topic_lidar_right=this->declare_parameter<std::string>("LidarSensors.points_right","/livox/lidar_front");
  float width_lidar=static_cast<float>(this->declare_parameter<double>("LidarSensors.width",360.0));
  float height_lidar=static_cast<float>(this->declare_parameter<double>("LidarSensors.height",59.0));
  bool flip_lidar=this->declare_parameter<bool>("LidarSensors.flip",false);
  float compensation_lidar=static_cast<float>(this->declare_parameter<double>("LidarSensors.range_compenstion",0.05));
  SensorInfo lidar_info(SensorType::LIDAR,topic_lidar_left,topic_lidar_right,width_lidar,height_lidar,flip_lidar,compensation_lidar);

  std::string topic_tof_left=this->declare_parameter<std::string>("TofSensors.points_left","/itof/itof_left_front/tof_frame/pointcloud");
  std::string topic_tof_right=this->declare_parameter<std::string>("TofSensors.points_right","/itof/itof_right_front/tof_frame/pointcloud");
  std::string topic_image_left=this->declare_parameter<std::string>("TofSensors.image_left","/itof/itof_left_front/tof_frame/depthz");
  std::string topic_image_right=this->declare_parameter<std::string>("TofSensors.image_right","/itof/itof_right_front/tof_frame/depthz");
  std::string topic_camera_left=this->declare_parameter<std::string>("TofSensors.camera_left","/itof/itof_left_front/caminfo");
  std::string topic_camera_right=this->declare_parameter<std::string>("TofSensors.camera_right","/itof/itof_right_front/caminfo");
  float width_tof=static_cast<float>(this->declare_parameter<double>("TofSensors.width",224.0));
  float height_tof=static_cast<float>(this->declare_parameter<double>("TofSensors.height",109.0));
  bool flip_tof=this->declare_parameter<bool>("TofSensors.flip",true);
  float compensation_tof=static_cast<float>(this->declare_parameter<double>("TofSensors.range_compenstion",0.0));
  SensorInfo tof_info(SensorType::TOF,
                      topic_tof_left,topic_tof_right,
                      topic_image_left,topic_image_right,
                      topic_camera_left,topic_camera_right,
                      width_tof,height_tof,
                      flip_tof,compensation_tof);
  float upper_length=static_cast<float>(this->declare_parameter<double>("TargetInfo.upper_layer.length",0.996));
  float upper_width=static_cast<float>(this->declare_parameter<double>("TargetInfo.upper_layer.width",0.558));
  float upper_height=static_cast<float>(this->declare_parameter<double>("TargetInfo.upper_layer.height",1.0));
  float lower_length=static_cast<float>(this->declare_parameter<double>("TargetInfo.lower_layer.length",1.07));
  float lower_width=static_cast<float>(this->declare_parameter<double>("TargetInfo.lower_layer.width",0.594));
  float lower_height=static_cast<float>(this->declare_parameter<double>("TargetInfo.lower_layer.height",0.43));
  std::string reference_feature=this->declare_parameter<std::string>("TargetInfo.supplementary.feature","tag");
  auto feature_params_double=this->declare_parameter<std::vector<double>>("TargetInfo.supplementary.paras",{0.21,0.20});
  std::vector<float> feature_params(feature_params_double.begin(),feature_params_double.end());
  TargetInfo target_info(upper_length,upper_width,upper_height,lower_length,lower_width,lower_height,reference_feature,feature_params);

  bool trouble_shoting=this->declare_parameter<bool>("SettingPara.trouble_shoting",false);
  std::string sensor_type=this->declare_parameter<std::string>("SettingPara.sensor_type","lidar");
  sensor_type_str=sensor_type;
  auto batch_size_values=this->declare_parameter<std::vector<int64_t>>("SettingPara.batch_size",{1,20});
  std::vector<int> batch_size_list(batch_size_values.begin(),batch_size_values.end());
  bool reference_pose=this->declare_parameter<bool>("SettingPara.reference_pose",false);
  int stop_criteria=static_cast<int>(this->declare_parameter<int64_t>("SettingPara.stop_criteria",250));
  auto_start=this->declare_parameter<bool>("SettingPara.auto_start",true);
  sensor_side=this->declare_parameter<std::string>("SettingPara.sensor_side","right");
  SettingInfo setting_info(trouble_shoting,sensor_type,batch_size_list,reference_pose,stop_criteria);

  std::string service_name=this->declare_parameter<std::string>("PubInfo.service_name","/hecbot_perception/storage_detect");
  std::string result_name=this->declare_parameter<std::string>("PubInfo.result_name","/hecbot_perception/storage_result");
  std::string reference_name=this->declare_parameter<std::string>("PubInfo.reference_name","/hecbot_perception/storage/reference_pose");
  std::string refer_region_name=this->declare_parameter<std::string>("PubInfo.refer_region_name","/hecbot_perception/storage/reference_region");
  std::string naive_region_name=this->declare_parameter<std::string>("PubInfo.naive_region_name","/hecbot_perception/storage/naive_region");
  std::string raw_name=this->declare_parameter<std::string>("PubInfo.raw_name","/hecbot_perception/storage/base_points");
  std::string depth_name=this->declare_parameter<std::string>("PubInfo.depth_name","/hecbot_perception/storage/depth_detect");
  std::string hyp_points_name=this->declare_parameter<std::string>("PubInfo.hyp_points_name","/hecbot_perception/storage/proposal_points");
  std::string hyp_pose_name=this->declare_parameter<std::string>("PubInfo.hyp_pose_name","/hecbot_perception/storage/proposal_pose");
  std::string evidence_points_name=this->declare_parameter<std::string>("PubInfo.evidence_points_name","/hecbot_perception/storage/evidence_points");
  std::string evidence_pose_name=this->declare_parameter<std::string>("PubInfo.evidence_pose_name","/hecbot_perception/storage/evidence_pose");
  std::string remember_pose_name=this->declare_parameter<std::string>("PubInfo.remember_pose_name","/perception/debug/poses");
  std::string repub_points_name=this->declare_parameter<std::string>("PubInfo.repub_points_name","/perception/source/points");
  std::string repub_image_name=this->declare_parameter<std::string>("PubInfo.repub_image_name","/perception/source/image");
  std::string lidar_capture_name=this->declare_parameter<std::string>("PubInfo.lidar_capture_name","/perception/capture/points");
  control_name=this->declare_parameter<std::string>("PubInfo.control_name","/perception_storage/control");
  PubInfo pub_info(service_name,result_name,reference_name,refer_region_name,naive_region_name,
                   raw_name,depth_name,hyp_points_name,hyp_pose_name,
                   evidence_points_name,evidence_pose_name,
                   remember_pose_name,
                   repub_points_name,repub_image_name,lidar_capture_name);
  // 配置项
  config_info.m_lidarinfo=lidar_info;
  config_info.m_tofinfo=tof_info;
  config_info.m_targetinfo=target_info;
  config_info.m_settinginfo=setting_info;
  config_info.m_pubinfo=pub_info;
  this->declare_parameter<std::string>("FrameNames.front_frame","livox_frame_front");
  this->declare_parameter<std::string>("FrameNames.rear_frame","livox_frame_rear");
  this->declare_parameter<std::string>("FrameNames.left_frame","itof_left_front_frame");
  this->declare_parameter<std::string>("FrameNames.right_frame","itof_right_front_frame");

  if((sensor_type!="lidar" && sensor_type!="tof") || batch_size_list.size()<2 ||
     batch_size_list[0]<=0 || batch_size_list[1]<=0 ||
     (sensor_side!="left" && sensor_side!="right"))
  {
    RCLCPP_ERROR(this->get_logger(),"invalid sensor_type, sensor_side, or batch_size parameter");
    return false;
  }
  return true;
}

void PerceptionStorage::set_processor()
{
  std::string sensor_left="left";
  std::string sensor_right="right";
  // tof
  if(config_info.m_settinginfo.sensor_type==SensorType::TOF)
  {
    auto sensor_qos=rclcpp::SensorDataQoS();
    cam_info_left=this->create_subscription<sensor_msgs::msg::CameraInfo>(config_info.m_tofinfo.cam_left,sensor_qos,
      [this,sensor_left](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg){cam_callback(*msg,sensor_left);});
    cam_info_right=this->create_subscription<sensor_msgs::msg::CameraInfo>(config_info.m_tofinfo.cam_right,sensor_qos,
      [this,sensor_right](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg){cam_callback(*msg,sensor_right);});
    sub_tof_points_left.subscribe(this,config_info.m_tofinfo.topic_left,sensor_qos.get_rmw_qos_profile());
    sub_tof_points_right.subscribe(this,config_info.m_tofinfo.topic_right,sensor_qos.get_rmw_qos_profile());
    sub_tof_gray_left.subscribe(this,config_info.m_tofinfo.gray_left,sensor_qos.get_rmw_qos_profile());
    sub_tof_gray_right.subscribe(this,config_info.m_tofinfo.gray_right,sensor_qos.get_rmw_qos_profile());
    sync_tof_left.reset(new tof_sync(SyncPolicy_tof(5),sub_tof_points_left,sub_tof_gray_left));
    sync_tof_right.reset(new tof_sync(SyncPolicy_tof(5),sub_tof_points_right,sub_tof_gray_right));
    sync_tof_left->registerCallback(std::bind(&PerceptionStorage::process_tof,this,
                                              std::placeholders::_1,std::placeholders::_2,sensor_left));
    sync_tof_right->registerCallback(std::bind(&PerceptionStorage::process_tof,this,
                                               std::placeholders::_1,std::placeholders::_2,sensor_right));
  }
  // lidar
  else if(config_info.m_settinginfo.sensor_type==SensorType::LIDAR)
  {
    auto sensor_qos=rclcpp::SensorDataQoS();
    sub_livox_left=this->create_subscription<sensor_msgs::msg::PointCloud2>(config_info.m_lidarinfo.topic_left,sensor_qos,
      [this,sensor_left](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){process_lidar(msg,sensor_left);});
    sub_livox_right=this->create_subscription<sensor_msgs::msg::PointCloud2>(config_info.m_lidarinfo.topic_right,sensor_qos,
      [this,sensor_right](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){process_lidar(msg,sensor_right);});
  }
  sub_control=this->create_subscription<std_msgs::msg::String>(control_name,10,
    std::bind(&PerceptionStorage::control_callback,this,std::placeholders::_1));
}

void PerceptionStorage::set_publishers()
{
  // 结果
  pub_result=this->create_publisher<geometry_msgs::msg::PoseStamped>(config_info.m_pubinfo.result_name,1);
  pub_naive_region=this->create_publisher<visualization_msgs::msg::MarkerArray>(config_info.m_pubinfo.naive_region_name,1);
  // 处理点
  pub_base_points=this->create_publisher<sensor_msgs::msg::PointCloud2>(config_info.m_pubinfo.raw_name,1);
  // 深度图检测结果
  pub_depth_detect=this->create_publisher<sensor_msgs::msg::Image>(config_info.m_pubinfo.depth_name,1);
  // 候选点
  pub_hypothesis_points=this->create_publisher<sensor_msgs::msg::PointCloud2>(config_info.m_pubinfo.hyp_points_name,1);
  // 候选姿态
  pub_hypothesis_pose=this->create_publisher<geometry_msgs::msg::PoseArray>(config_info.m_pubinfo.hyp_pose_name,1);
  // 满足条件的目标点
  pub_evidence_points=this->create_publisher<sensor_msgs::msg::PointCloud2>(config_info.m_pubinfo.evidence_points_name,1);
  // 满足条件的姿态
  pub_evidence_pose=this->create_publisher<geometry_msgs::msg::PoseArray>(config_info.m_pubinfo.evidence_pose_name,1);
  // 一次任务的姿态记忆
  pub_remember_pose=this->create_publisher<geometry_msgs::msg::PoseArray>(config_info.m_pubinfo.remember_pose_name,1);
  // 转发原始数据
  pub_repub_points=this->create_publisher<sensor_msgs::msg::PointCloud2>(config_info.m_pubinfo.repub_points_name,1);
  pub_repub_image=this->create_publisher<sensor_msgs::msg::Image>(config_info.m_pubinfo.repub_image_name,1);
  // 雷达对货架底座的观测
  pub_lidar_capture=this->create_publisher<sensor_msgs::msg::PointCloud2>(config_info.m_pubinfo.lidar_capture_name,1);
}

void PerceptionStorage::insert_buffer(MsgPacket &msg)
{
  std::unique_lock<std::mutex> lock(_mtx_);
  msg_packet.emplace_back(msg);
}

void PerceptionStorage::parse_data(std::deque<MsgPacket> &msgs)
{
  std::unique_lock<std::mutex> lock(_mtx_);
  if(!msg_packet.empty())
  {
    msgs.insert(msgs.end(),msg_packet.begin(),msg_packet.end());
    msg_packet.clear();
  }
}

bool PerceptionStorage::parse_frame(std::deque<MsgPacket> &msgs)
{
  std::deque<MsgPacket> msgs_segments;
  parse_data(msgs_segments);
  if(msgs_segments.empty())
  {
    return false;
  }
  for(auto it=msgs_segments.begin();it!=msgs_segments.end();it++)
  {
    auto data=*it;
    if(data.enum_side==active_)
    { 
      process_queue.emplace_back(data); 
    }
  }
  if(process_queue.size()>=config_info.m_settinginfo.batch_size)
  {
    msgs.insert(msgs.end(),process_queue.begin(),process_queue.end());
    while(process_queue.size()>=(config_info.m_settinginfo.batch_size))
    {
      process_queue.pop_front();
    }
    return true;
  }
  else
  {
    return false;
  }
}

depth_mapping PerceptionStorage::couple_data(std::deque<MsgPacket> &multiframes)
{
  if(config_info.m_settinginfo.sensor_type==SensorType::TOF)
  {
    return couple_tof(multiframes);
  }
  else if(config_info.m_settinginfo.sensor_type==SensorType::LIDAR)
  {
    return couple_lidar(multiframes);
  }
  return depth_mapping();
}
 
depth_mapping PerceptionStorage::couple_lidar(std::deque<MsgPacket> &multiframes)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr concate_points(new pcl::PointCloud<pcl::PointXYZI>);
  auto pub_header_msg=multiframes.back().lidar_data.msg_header;
  
  // Step1: 0 帧滑动窗口融合,三维 ROI 裁剪
  pcl::PassThrough<pcl::PointXYZI> pass;
  while(!multiframes.empty())
  {
    auto raw_ptr=multiframes.front().lidar_data.point_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_points(new pcl::PointCloud<pcl::PointXYZI>);
    // hard cut
    pass.setInputCloud(raw_ptr);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-2.5*config_info.m_targetinfo.lower_width, -0.1);
    pass.filter(*filtered_points);
    pass.setInputCloud(filtered_points);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-1.2*config_info.m_targetinfo.lower_length,0.2*config_info.m_targetinfo.lower_length);
    pass.filter(*filtered_points);
    pass.setInputCloud(filtered_points);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-config_info.m_targetinfo.lower_height,1.2*config_info.m_targetinfo.lower_height);
    pass.filter(*filtered_points);
    *concate_points+=*filtered_points;
    multiframes.pop_front();
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger("perception_storage"), "[PerceptionStorage][Perception] CoupleLidarFrames");
  
  // Step2: 均匀 2.5mm 降采样，密度一致
  filter_machine->sampling_filter(concate_points,concate_points,"uniform",0.0025f);
  
  // Step3: 按 0.00175 rad 角分辨率分桶，同一方向桶只保留到雷达原点欧氏距离更近的点
  std::map<coord,pcl::PointXYZI> coord_map;
  for(auto & mmpoint:concate_points->points)
  {
    coord mq(Eigen::Vector3f(mmpoint.x,mmpoint.y,mmpoint.z),0.00175);
    if(coord_map.find(mq)!=coord_map.end())
    {
      if(Eigen::Vector3f(mmpoint.x,mmpoint.y,mmpoint.z).norm()<
         Eigen::Vector3f(coord_map[mq].x,coord_map[mq].y,coord_map[mq].z).norm())
      {
        coord_map[mq]=mmpoint;
      }
    }
    else
    {
      coord_map.insert(std::make_pair(mq,mmpoint));
    }
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr t_points(new pcl::PointCloud<pcl::PointXYZI>);
  for(auto& mmpoint:coord_map)
  {
    t_points->push_back(mmpoint.second);
  }
  pcl::copyPointCloud(*t_points,*concate_points);
  
  // Step4: 截取 z≈0 薄层，投影到xoy平面上，以 0.25° 角分辨率分桶；每桶保留最近点
  pcl::PointCloud<pcl::PointXYZI>::Ptr planar_lidar(new pcl::PointCloud<pcl::PointXYZI>);
  pass.setInputCloud(concate_points);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-0.025,0.025);
  pass.filter(*planar_lidar);
  std::map<int,pcl::PointCloud<pcl::PointXYZI>> theta_map;
  for(auto &planar_point:planar_lidar->points)
  {
    planar_point.z=0;
    int theta_bin=floor(atan2(planar_point.x,planar_point.y)*180.0/M_PI/0.25);
    if(theta_map.find(theta_bin)!=theta_map.end())
    {
      theta_map[theta_bin].points.emplace_back(planar_point);
    }
    else
    {
      pcl::PointCloud<pcl::PointXYZI> temple_pc;
      temple_pc.points.emplace_back(planar_point);
      theta_map.insert(std::make_pair(theta_bin,temple_pc));
    }
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr planar_lidar_filter(new pcl::PointCloud<pcl::PointXYZI>);
  for(auto &theta_points:theta_map)
  {
    std::vector<compare_point> top_points;
    for(int p_index=0;p_index<theta_points.second.points.size();p_index++)
    {
      top_points.emplace_back(compare_point(p_index,theta_points.second.points[p_index]));
    }
    std::sort(top_points.begin(),top_points.end(),compere_func);
    planar_lidar_filter->points.emplace_back(top_points[0].point);
    if(top_points.size()>1 && fabs(top_points[0].range-top_points[1].range)<0.01)
    {
      planar_lidar_filter->points.emplace_back(top_points[1].point);
    }
  }
  
  // Step5: 按 x 以 5 mm 分桶；每桶保留 |y| 最小的点(靠近机器人的点)
  std::map<int,pcl::PointCloud<pcl::PointXYZI>> x_map;
  for(auto &planar_point:planar_lidar_filter->points)
  {
    int x_bin=floor(planar_point.x/0.005);
    if(x_map.find(x_bin)!=x_map.end())
    {
      x_map[x_bin].points.emplace_back(planar_point);
    }
    else
    {
      pcl::PointCloud<pcl::PointXYZI> temple_pc;
      temple_pc.points.emplace_back(planar_point);
      x_map.insert(std::make_pair(x_bin,temple_pc));
    }
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr planar_lidar_filter2(new pcl::PointCloud<pcl::PointXYZI>);
  for(auto &x_points:x_map)
  {
    std::vector<compare_point> top_points;
    for(int p_index=0;p_index<x_points.second.points.size();p_index++)
    {
      top_points.emplace_back(compare_point(x_points.second.points[p_index]));
    }
    std::sort(top_points.begin(),top_points.end(),compere_func);
    planar_lidar_filter2->points.emplace_back(top_points[0].point);
  }
  
  // Step6: 按 `atan2(x, |y|)` 排序，形成沿扫描方向有序的二维轮廓，并设置强度值 62
  std::vector<compare_point> reorder_points;
  for(auto &r_point:planar_lidar_filter2->points)
  {
    reorder_points.emplace_back(compare_point(r_point,true));
  }
  std::sort(reorder_points.begin(),reorder_points.end(),compere_func);
  planar_lidar_filter->clear();
  for(auto &reorder_point:reorder_points)
  {
    auto mrp=reorder_point.point;
    mrp.intensity=62;
    planar_lidar_filter->push_back(mrp);
  }
  planar_lidar_filter2->clear();

  // Step7: 欧氏聚类：容差 2.5 cm，点数 5～450
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(planar_lidar_filter);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(0.025);
  ec.setMinClusterSize(5);
  ec.setMaxClusterSize(450);
  ec.setSearchMethod(tree);
  ec.setInputCloud(planar_lidar_filter);
  ec.extract(cluster_indices);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("perception_storage"), "[PerceptionStorage][Perception] cluster_indices "<<cluster_indices.size());

  // Step8: find 辅助板投影线
  int cluster_index=0;
  std::vector<Eigen::VectorXf> proposal_coef;
  std::vector<pcl::PointXYZ> proposal_mid;
  for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    cluster_index++;
    const std::vector<int> &index_vec = it->indices;
    auto direct_dist=pcl::euclideanDistance(planar_lidar_filter->points[index_vec.front()],planar_lidar_filter->points[index_vec.back()]);
    float  dd=0.5*config_info.m_targetinfo.params[0];
    // 聚类首尾距离必须接近辅助板宽度 0.21 m，允许误差为其 50%，即约 0.105～0.315 m
    if(fabs(direct_dist-config_info.m_targetinfo.params[0])<=dd)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
      for(std::vector<int>::const_iterator pit = index_vec.begin(); pit != index_vec.end(); ++pit)
      {
        int cp_index=*pit;
        auto &cp_p=planar_lidar_filter->points[cp_index];
        cloud_cluster->points.push_back(cp_p);
      }
      // 对聚类做 RANSAC 直线拟合：距离阈值 1 cm
      pcl::SampleConsensusModelLine<pcl::PointXYZI>::Ptr sim_model(new pcl::SampleConsensusModelLine<pcl::PointXYZI>(cloud_cluster));	
      pcl::RandomSampleConsensus<pcl::PointXYZI> line_fit(sim_model);	
      line_fit.setDistanceThreshold(0.01);
      line_fit.setMaxIterations(100);
      line_fit.computeModel();
      std::vector<int> line_inliers_;
      line_fit.getInliers(line_inliers_);
      // 要求至少 10 个内点且内点率（直线上点占聚类总点数的比例）至少 80%
      if(line_inliers_.size()<10 || line_inliers_.size()<0.8*index_vec.size())
      {
        continue;
      }
      // 在当前聚类点云的前后搜索货架腿结构
      Eigen::VectorXf line_coef_;
      line_fit.getModelCoefficients(line_coef_);
      Eigen::Vector3f line_dir_={line_coef_[3],line_coef_[4],line_coef_[5],};
      Eigen::Vector3f line_pt_={line_coef_[0],line_coef_[1],line_coef_[2]};
      std::vector<int> with_line,width_diagonal;
      // 聚类点云左侧剩余点
      if(index_vec.front()>1)
      {
        for(int pk=0;pk<index_vec.front()-1;pk++)
        {
          Eigen::Vector4f pt_4f(planar_lidar_filter->points[pk].x,planar_lidar_filter->points[pk].y,planar_lidar_filter->points[pk].z,0);
          float p2line =std::sqrt(pcl::sqrPointToLineDistance(pt_4f,
                                 Eigen::Vector4f(line_coef_(0),line_coef_(1),line_coef_(2),0),
                                 Eigen::Vector4f(line_coef_(3),line_coef_(4),line_coef_(5),0)));
          // 点到拟合线的投影距离小于 2 cm 的共线延伸点
          if(p2line<0.02)
          {
            float dist_left=pcl::euclideanDistance(planar_lidar_filter->points[index_vec.front()],planar_lidar_filter->points[pk]);
            // 点在货架腿和辅助板之间（到拟合线左端点的欧式距离）
            if(dist_left<(0.5*config_info.m_targetinfo.lower_length-0.5*config_info.m_targetinfo.params[0]-0.05))
            {
              with_line.push_back(pk);
            }
          }
          // 点到拟合线的投影距离大于 2 cm，且小于货架宽度一半（不超出货架投影矩形）
          else if(fabs(p2line-0.5*config_info.m_targetinfo.lower_width)<0.05)
          {
            float mt=line_dir_(0)*(planar_lidar_filter->points[pk].x-line_pt_(0))+line_dir_(1)*(planar_lidar_filter->points[pk].y-line_pt_(1))+line_dir_(2)*(planar_lidar_filter->points[pk].z-line_pt_(2));
            float px_=line_pt_(0)+line_dir_(0)*mt;
            float py_=line_pt_(1)+line_dir_(1)*mt;
            float pz_=line_pt_(2)+line_dir_(2)*mt;
            float p_p_dist=Eigen::Vector3f(planar_lidar_filter->points[index_vec.front()].x-px_,
                                          planar_lidar_filter->points[index_vec.front()].y-py_,
                                          planar_lidar_filter->points[index_vec.front()].z-pz_).norm();
            // 点为货架腿（到拟合线左端点的欧式距离）
            if(fabs(p_p_dist-(0.5*config_info.m_targetinfo.lower_length-0.5*config_info.m_targetinfo.params[0]))<0.05)
            {
              // 找到货架腿
              width_diagonal.push_back(pk);
            }
          }
        }
      }
      // 聚类点云右侧剩余点
      if(index_vec.back()<(planar_lidar_filter->size()-1))
      {
        for(int pk=index_vec.back()+1;pk<planar_lidar_filter->size();pk++)
        {
          Eigen::Vector4f pt_4f(planar_lidar_filter->points[pk].x,planar_lidar_filter->points[pk].y,planar_lidar_filter->points[pk].z,0);
          float p2line =std::sqrt(pcl::sqrPointToLineDistance(pt_4f,
                                 Eigen::Vector4f(line_coef_(0),line_coef_(1),line_coef_(2),0),
                                 Eigen::Vector4f(line_coef_(3),line_coef_(4),line_coef_(5),0)));
          if(p2line<0.02)
          {
            float dist_left=pcl::euclideanDistance(planar_lidar_filter->points[index_vec.back()],planar_lidar_filter->points[pk]);
            if(dist_left<(0.5*config_info.m_targetinfo.lower_length-0.5*config_info.m_targetinfo.params[0]-0.05))
            {
              with_line.push_back(pk);
            }
          }
          else if(fabs(p2line-0.5*config_info.m_targetinfo.lower_width)<0.05)
          {
            float mt=line_dir_(0)*(planar_lidar_filter->points[pk].x-line_pt_(0))+line_dir_(1)*(planar_lidar_filter->points[pk].y-line_pt_(1))+line_dir_(2)*(planar_lidar_filter->points[pk].z-line_pt_(2));
            float px_=line_pt_(0)+line_dir_(0)*mt;
            float py_=line_pt_(1)+line_dir_(1)*mt;
            float pz_=line_pt_(2)+line_dir_(2)*mt;
            float p_p_dist=Eigen::Vector3f(planar_lidar_filter->points[index_vec.back()].x-px_,
                                          planar_lidar_filter->points[index_vec.back()].y-py_,
                                          planar_lidar_filter->points[index_vec.back()].z-pz_).norm();
            if(fabs(p_p_dist-(0.5*config_info.m_targetinfo.lower_length-0.5*config_info.m_targetinfo.params[0]))<0.05)
            {
              width_diagonal.push_back(pk);
            }
          }
        }
      }
      // 可能的参考对象
      RCLCPP_INFO_STREAM(rclcpp::get_logger("perception_storage"), "[PerceptionStorage][Perception] width_diagonal "<<width_diagonal.size()<<" with_line "<<with_line.size());
      // 聚类满足辅助板长度（辅助板投影线），且找到货架腿
      if(width_diagonal.size()>0 && with_line.size()<5)
      {
        for(std::vector<int>::const_iterator pit = index_vec.begin(); pit != index_vec.end(); ++pit)
        {
          int cp_index=*pit;
          planar_lidar_filter->points[cp_index].intensity=255;
        }
        for(auto diagonal_index:width_diagonal)
        {
          planar_lidar_filter->points[diagonal_index].intensity=127;
        }
        float mid_x=(cloud_cluster->at(line_inliers_.front()).x+cloud_cluster->at(line_inliers_.back()).x)/2.0;
        float mid_y=(cloud_cluster->at(line_inliers_.front()).y+cloud_cluster->at(line_inliers_.back()).y)/2.0;
        // 辅助板投影线中点在指定范围内，认为找到候选辅助板
        if(fabs(mid_x+0.5*config_info.m_targetinfo.lower_length)<0.4*config_info.m_targetinfo.lower_length &&
           fabs(mid_y)>0.5*config_info.m_targetinfo.lower_width &&
           fabs(mid_y)<2.0*config_info.m_targetinfo.lower_width)
        {
          Eigen::Vector4f line_direction_={line_coef_[3],line_coef_[4],line_coef_[5],0};
          if(line_direction_[0]<0) line_direction_*=-1.0;
          Eigen::Vector4f x_direction_={1.0,0,0,0};
          auto angle=fabs(pcl::rad2deg(fabs(pcl::normAngle(pcl::getAngle3D(line_direction_,x_direction_)))));
          if(angle<30.0)
          {
            proposal_coef.emplace_back(line_coef_);
            proposal_mid.emplace_back(pcl::PointXYZ(mid_x,mid_y,0));
          }
        }
      }
    }
    else
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("perception_storage"), "[PerceptionStorage][Perception] distres "<<fabs(direct_dist-config_info.m_targetinfo.params[0])<<" dd "<<dd);
    }
  }
  
  // Step9: 恰好找到 1 个候选辅助板时，构建辅助板坐标系，计算辅助板坐标系和雷达坐标系的旋转矩阵；否则使用默认旋转矩阵
  /* 辅助板坐标系 y 轴取标签线方向，并强制其 x 分量为正；
   * 辅助板坐标系 z 轴固定为 `(0,0,1)`；
   * 辅助板坐标系 x 轴为 `y × z`；
   * 辅助板坐标系 o 原点为辅助板投影线中点；
   * 
   * 默认旋转矩阵：绕 z 轴旋转 +90°，平移 0.5 × lower_length
  */
  Eigen::Matrix4f T_lidar_matrix(Eigen::Matrix4f::Identity());
  bool reference_init=false;
  if(proposal_coef.size()==1 && proposal_mid.size()==1)
  {
    Eigen::Vector3f z_direction_(0,0,1);
    Eigen::Vector3f y_direction_={proposal_coef[0][3],proposal_coef[0][4],proposal_coef[0][5]};
    if(y_direction_[0]<0) y_direction_*=-1.0;
    Eigen::Vector3f x_direction_=y_direction_.cross(z_direction_);
    Eigen::Matrix3f r_matrix;
    r_matrix.col(0)=x_direction_;r_matrix.col(1)=y_direction_;r_matrix.col(2)=z_direction_;
    T_lidar_matrix.topLeftCorner<3, 3>() =r_matrix;
    T_lidar_matrix.block<3,1>(0,3)=Eigen::Vector3f(proposal_mid[0].x,proposal_mid[0].y,proposal_mid[0].z);
    reference_init=true;
    pcl::PointCloud<pcl::PointXYZI>::Ptr add_points(new pcl::PointCloud<pcl::PointXYZI>);
    for(int add_index=0;add_index<50;add_index++)
    {
      auto y_add=y_direction_*add_index*0.01;
      auto x_add=x_direction_*add_index*0.01;
      pcl::PointXYZI y_add_p,x_add_p;
      y_add_p.x=y_add(0)+proposal_mid[0].x;y_add_p.y=y_add(1)+proposal_mid[0].y;y_add_p.z=y_add(2)+proposal_mid[0].z;y_add_p.intensity=30;
      x_add_p.x=x_add(0)+proposal_mid[0].x;x_add_p.y=x_add(1)+proposal_mid[0].y;x_add_p.z=x_add(2)+proposal_mid[0].z;x_add_p.intensity=60;
      add_points->push_back(y_add_p);
      add_points->push_back(x_add_p);
    }
    *planar_lidar_filter+=*add_points;
    // Do not assign Eigen's inverse expression back to the same matrix
    // without materializing it first; Eigen detects this aliasing at runtime.
    T_lidar_matrix=T_lidar_matrix.inverse().eval();
  }
  else
  {
    T_lidar_matrix.topLeftCorner<3, 3>()=(Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ())).toRotationMatrix();
    T_lidar_matrix.block<3,1>(0,3)=Eigen::Vector3f(0.5*config_info.m_targetinfo.lower_length,0,0);
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger("perception_storage"), "[PerceptionStorage][Perception] DepthReference "<<reference_init<<" "<<proposal_coef.size()<<" "<<proposal_mid.size());

  // Step9: 3d点云转到辅助板坐标系，计算辅助板投影线在辅助板坐标系下的坐标
  pcl::PointCloud<pcl::PointXYZI>::Ptr trans_lidar(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*concate_points,*trans_lidar,T_lidar_matrix);
  Eigen::Vector3f refer_mid_point;
  if(proposal_mid.size()==1)
  {
    Eigen::Vector4f mid_point(proposal_mid[0].x,proposal_mid[0].y,proposal_mid[0].z,1);
    mid_point=T_lidar_matrix*mid_point;
    refer_mid_point=mid_point.head<3>();
  }

  // Step10: ROI 裁剪
  if(reference_init)
  {
    pass.setInputCloud(trans_lidar);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-0.5*config_info.m_targetinfo.lower_length-0.1,0.5*config_info.m_targetinfo.lower_length+0.1);
    pass.filter(*trans_lidar);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-0.5*config_info.m_targetinfo.lower_width-0.1,0.5*config_info.m_targetinfo.lower_width-0.1);
    pass.filter(*trans_lidar);
  }
  
  // Step11: 以 0.00175 rad 的球面角分桶，每桶保留近点
  coord_map.clear();
  for(auto & mmpoint:trans_lidar->points)
  {
    coord mq(Eigen::Vector3f(mmpoint.x,mmpoint.y,mmpoint.z),0.00175);
    if(coord_map.find(mq)!=coord_map.end())
    {
      if(Eigen::Vector3f(mmpoint.x,mmpoint.y,mmpoint.z).norm()<
        Eigen::Vector3f(coord_map[mq].x,coord_map[mq].y,coord_map[mq].z).norm())
      {
        coord_map[mq]=mmpoint;
      }
    }
    else
    {
      coord_map.insert(std::make_pair(mq,mmpoint));
    }
  }
  t_points->points.clear();
  for(auto& mmpoint:coord_map)
  {
    t_points->push_back(mmpoint.second);
  }
  pcl::copyPointCloud(*t_points,*trans_lidar);
  
  // Step12: 货架底部点过滤，只保留 z ∈ [-0.2, 0.2] 的点，按 y 以 1 cm 分桶
  std::map<int,pcl::PointCloud<pcl::PointXYZI>> zero_slice;
  float z_cut_min=-0.2;
  float z_cut_max=0.2;
  for(auto &raw_p:trans_lidar->points)
  {
    if(raw_p.z<z_cut_min || raw_p.z>z_cut_max) continue;
    int y_bin=floor(raw_p.y/0.01);
    if(zero_slice.find(y_bin)!=zero_slice.end())
    {
      zero_slice[y_bin].points.emplace_back(raw_p);
    }
    else
    { 
      pcl::PointCloud<pcl::PointXYZI> temple_pc;
      temple_pc.points.emplace_back(raw_p);
      zero_slice.insert(std::make_pair(y_bin,temple_pc));
    }
  }

  // Step13: 每个 y 桶计算 `|x|` 的最小值和最大值，根据跨度选择保留点
  pcl::PointCloud<pcl::PointXYZI>::Ptr ray_filter(new pcl::PointCloud<pcl::PointXYZI>);
  float max_gap=Eigen::Vector2f(config_info.m_targetinfo.lower_width,config_info.m_targetinfo.lower_length).norm();
  for(auto & theta_x:zero_slice)
  {
    std::vector<float> dist_arrray;
    float min_dist=std::numeric_limits<float>::max();
    float max_dist=std::numeric_limits<float>::min();
    for(auto & x_p:theta_x.second.points)
    {
      auto _dist_=fabs(x_p.x);
      dist_arrray.emplace_back(_dist_);
      min_dist=(_dist_<min_dist)?_dist_:min_dist;
      max_dist=(_dist_>max_dist)?_dist_:max_dist;
    }
    // 跨度超过货架宽度，全部保留；跨度小于货架宽度，保留货架宽度内的点
    float dist_range=((max_dist-min_dist)>config_info.m_targetinfo.lower_width)?(max_dist-0.1):(min_dist+0.6*config_info.m_targetinfo.lower_width);
    for(int i=0;i<dist_arrray.size();i++)
    {
      if(dist_arrray[i]<=dist_range)
      {
        ray_filter->points.emplace_back(theta_x.second.points[i]);
      }
    }
  }
  // 统计滤波去噪
  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud(ray_filter);
  sor.setMeanK(20);
  sor.setStddevMulThresh(1);
  sor.filter(*ray_filter);

  // Step14: 构造二值投影和像素—点云映射,栅格分辨率固定为 0.0075 m/pixel
  int mcols=0;
  int mrows=0;
  pcl::PointXYZI _base_min_,_base_max_;
  bool depth_state=true;
  if(ray_filter->size()>100)
  {
    pcl::getMinMax3D(*ray_filter,_base_min_,_base_max_);
    // 栅格填充
    mcols=ceil((_base_max_.y-_base_min_.y)/0.0075);
    mrows=ceil((z_cut_max-z_cut_min)/0.0075);
  }
  if(mcols<=1 || mrows<=1)
  {
    depth_state=false;
    mcols=std::max(mcols,10);
    mrows=std::max(mrows,10);
  }
  Eigen::Vector2i refer_mix_pixel(-1,-1);
  if(proposal_mid.size()==1)
  {
    refer_mix_pixel=Eigen::Vector2i(int(fabs(refer_mid_point[1]-_base_max_.y)/0.0075),int(fabs(refer_mid_point[2]-z_cut_max)/0.0075));
  }
  cv::Mat base_image(mrows,mcols,CV_8UC1,cv::Scalar::all(0));
  std::vector<int> mapping_table(mrows*mcols,-1);
  if(depth_state)
  {
    for(int mp_index=0;mp_index<ray_filter->size();mp_index++)
    {
      auto x_p=ray_filter->at(mp_index);
      int y_group=int(fabs(x_p.z-z_cut_max)/0.0075);
      int x_group=int(fabs(x_p.y-_base_max_.y)/0.0075);
      base_image.at<uchar>(y_group,x_group)=255;
      int mmindex=y_group*mcols+x_group;
      if(mapping_table[mmindex]==-1)
      {
        mapping_table[mmindex]=mp_index;
      }
      else
      {
        Eigen::Vector3f Pr=ray_filter->at(mapping_table[mmindex]).getVector3fMap();
        Eigen::Vector3f Pc=ray_filter->at(mp_index).getVector3fMap();
        if(Pc.norm()<Pr.norm())
        {
          mapping_table[mmindex]=mp_index;
        }
      }
    }
  }
  // 对空像素搜索约 ±1、±2 像素邻域，有至少两个非零邻居时补成非零
  cv::Mat smooth_range = base_image.clone();
  for (int x = 0; x < base_image.cols; x++)
  {
    for (int y = 0; y < base_image.rows; y++)
    {
      if (base_image.at<uchar>(y, x) == 0)
      {
        std::vector<int> temp_depth;
        for (int x_inc = 0; x_inc < 4; x_inc++)
        {
          for (int y_inc = 0; y_inc < 4; y_inc++)
          {
            int x_near = x + pow(-1, x_inc) * int((x_inc + 2) / 2);
            int y_near = y + pow(-1, y_inc) * int((y_inc + 2) / 2);
            if (x_near > 0 && y_near > 0 && base_image.at<uchar>(y_near, x_near) != 0)
            {
              temp_depth.push_back(base_image.at<uchar>(y_near, x_near));
            }
          }
          if(temp_depth.size() > 1)
          {
            int mean_depth = 0;
            for (size_t i = 0; i < temp_depth.size(); i++)
            {
              mean_depth += temp_depth[i];
            }
            smooth_range.at<uchar>(y, x) =int(mean_depth / temp_depth.size());
          }
        }
      }
    }
  }
  // 二值化
  base_image=smooth_range;
  cv::threshold(base_image,base_image,1,255,cv::THRESH_BINARY);
  // 点云变回雷达坐标
  pcl::PointCloud<pcl::PointXYZI>::Ptr back_origin_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*ray_filter,*back_origin_ptr,(T_lidar_matrix.inverse().cast<float>()));
  pcl::PointCloud<pcl::PointXYZI>::Ptr cues_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud(*back_origin_ptr,*cues_ptr);
  back_origin_ptr->width=back_origin_ptr->points.size();
  back_origin_ptr->height = 1;
  back_origin_ptr->is_dense = false;
  pcl::PointCloud<pcl::PointXYZ>::Ptr nointensity_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*back_origin_ptr,*nointensity_ptr);
  pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_zero_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud(*nointensity_ptr,*intensity_zero_ptr);
  {
    *planar_lidar_filter+=*intensity_zero_ptr;
    planar_lidar_filter->width=planar_lidar_filter->size();
    planar_lidar_filter->height = 1;
    planar_lidar_filter->is_dense = false;
    sensor_msgs::msg::PointCloud2 ray_filter_msg;
    pcl::toROSMsg(*planar_lidar_filter, ray_filter_msg);
    ray_filter_msg.height=planar_lidar_filter->height;
    ray_filter_msg.width=planar_lidar_filter->width;
    ray_filter_msg.header=pub_header_msg;
    pub_lidar_capture->publish(ray_filter_msg);
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger("perception_storage"), "[PerceptionStorage][Perception] LidarData_Finish");

  return depth_mapping(base_image,back_origin_ptr,mapping_table,cues_ptr,refer_mix_pixel);
}

depth_mapping PerceptionStorage::couple_tof(std::deque<MsgPacket> &multiframes)
{
  auto frame=multiframes.back();
  auto depth=frame.tof_data.gray_image;
  int img_width=depth.cols;
  int img_height=depth.rows;
  auto points=frame.tof_data.point_cloud;
  std::vector<int> points_mapping(depth.rows*depth.cols,-1);
  if(config_info.m_tofinfo.axis_flip)
  {
    cv::flip(depth,depth,-1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr reorder_points(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=(points->points.size()-1);i>=0;i--)
    {
      reorder_points->points.emplace_back(points->points[i]); 
    }
    points->swap(*reorder_points);
  }
  for(int i=0;i<points->points.size();i++)
  {
    int row=floor(i/img_width);
    int col=i%img_width;
    if(points->points[i].z==0)
    { 
      depth.at<uchar>(row, col) = 255; 
    }
    else
    {
      points_mapping[i]=i;
    }
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr copy_points(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud(*points,*copy_points);

  return depth_mapping(depth,copy_points,points_mapping);
}

void PerceptionStorage::cam_callback(const sensor_msgs::msg::CameraInfo &msg,
                                     const std::string &left_or_right)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("perception_storage"), "[PerceptionStorage][Perception] Heared "<<left_or_right<<" camera_info");
  cv::Mat cameraMatrix(3, 3, CV_64FC1);
  cv::Mat distorsionCoeff(4, 1, CV_64FC1);
  cv::Size size(msg.height, msg.width);
  for(int i=0; i<9; ++i)
  {
    cameraMatrix.at<double>(i%3, i-(i%3)*3) = msg.k[i];
  }
  if(msg.d.size() >= 4)
  {
    for(int i=0; i<4; ++i)
    {
      distorsionCoeff.at<double>(i, 0) = msg.d[i];
    }
  }
  else
  {
    for(int i=0; i<4; ++i)
    {
      distorsionCoeff.at<double>(i, 0) = 0;
    }
  }
  if(left_or_right=="left")
  {
    tof_cam_left=CameraInfo(cameraMatrix,distorsionCoeff,size);
    cam_left_ready=true;
    cam_info_left.reset();
  }
  else if(left_or_right=="right")
  {
    tof_cam_right=CameraInfo(cameraMatrix,distorsionCoeff,size);
    cam_right_ready=true;
    cam_info_right.reset();
  }
}

void PerceptionStorage::process_tof(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &points,
                                    const sensor_msgs::msg::Image::ConstSharedPtr &image,
                                    const std::string &which_side)
{
  if(which_side=="left")
  {
   if(active_==ActiveState::LEFT && cam_left_ready)
   {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("perception_storage"), "[PerceptionStorage][Perception] Heared_LeftTofMsg");
    int sensor_side_=1;
    MsgPacket msg_pack_(sensor_side_,tof_cam_left,image,points); 
    if(msg_pack_.tof_data.gray_image.empty())
    {
      RCLCPP_ERROR(this->get_logger(),"ToF left depth image encoding is unsupported: %s",image->encoding.c_str());
      return;
    }
    insert_buffer(msg_pack_);
    pub_repub_points->publish(*points);
    pub_repub_image->publish(*image);
   }
  }
  else
  {
   if(active_==ActiveState::RIGHT && cam_right_ready)
   {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("perception_storage"), "[PerceptionStorage][Perception] Heared_RightTofMsg");
    int sensor_side_=2;
    MsgPacket msg_pack_(sensor_side_,tof_cam_right,image,points);
    if(msg_pack_.tof_data.gray_image.empty())
    {
      RCLCPP_ERROR(this->get_logger(),"ToF right depth image encoding is unsupported: %s",image->encoding.c_str());
      return;
    }
    insert_buffer(msg_pack_);
    pub_repub_points->publish(*points);
    pub_repub_image->publish(*image);
   }
  }
}

void PerceptionStorage::process_lidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &points,
                                      const std::string &which_side)
{
  if(which_side=="left")
  {
   if(active_==ActiveState::LEFT)
   { 
    RCLCPP_INFO_STREAM(rclcpp::get_logger("perception_storage"), "[PerceptionStorage][Perception] Heared_LeftLidarMsg");
    int sensor_side_=1;
    MsgPacket msg_pack_(sensor_side_,points); 
    insert_buffer(msg_pack_);
    pub_repub_points->publish(*points);
   }
  }
  else
  {
   if(active_==ActiveState::RIGHT)
   {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("perception_storage"), "[PerceptionStorage][Perception] Heared_RightLidarMsg");
    int sensor_side_=2;
    MsgPacket msg_pack_(sensor_side_,points);
    insert_buffer(msg_pack_);
    pub_repub_points->publish(*points);
   }
  }
}

void PerceptionStorage::control_callback(const std_msgs::msg::String::ConstSharedPtr msg)
{
  std::vector<float> pose_{0.0f,0.0f,0.0f};
  if(msg->data=="start_left")
  {
    std::string side_="left";
    start(side_,pose_);
  }
  else if(msg->data=="start_right")
  {
    std::string side_="right";
    start(side_,pose_);
  }
  else if(msg->data=="stop")
  {
    stop();
  }
  else
  {
    RCLCPP_WARN(this->get_logger(),"unknown control command: %s",msg->data.c_str());
  }
}

void PerceptionStorage::start(std::string &side_,std::vector<float> &pose_)
{
  std::unique_lock<std::mutex> lock(_mtx_);
  job_num=0;
  valid_hys=0;
  valid_re=0;
  msg_packet.clear();
  process_queue.clear();
  task_remember_pose.poses.clear();
  if(side_=="left")
  {
    active_=ActiveState::LEFT;
  }
  else if(side_=="right")
  {
    active_=ActiveState::RIGHT;
  }
  RCLCPP_INFO_STREAM(rclcpp::get_logger("perception_storage"), "[PerceptionStorage][Perception] RequestStart "<<side_<<"_"<<pose_[0]<<"_"<<pose_[1]<<"_"<<pose_[2]);
}

void PerceptionStorage::stop()
{
  std::unique_lock<std::mutex> lock(_mtx_);
  active_=ActiveState::SLEEPY;
  msg_packet.clear();
  process_queue.clear();
  job_num=0;
  valid_hys=0;
  valid_re=0;
  task_remember_pose.poses.clear();
}

bool PerceptionStorage::stop_self()
{
  if(job_num>config_info.m_settinginfo.stop_criteria)
  {
    stop();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("perception_storage"), "[PerceptionStorage][Perception] StopByCriteria");
    return true;
  }
  else
  {
    return false;
  }
}

void PerceptionStorage::publish_msgs(std::pair<cv::Mat,std::vector<intermediate>> &interm_info,
                                    std::vector<mpose> &target_info,std::vector<mpose> &result_info,
                                    std_msgs::msg::Header &header_info)
{
  provide_evidence(interm_info,target_info,result_info,header_info);
  if(result_info.size()>0)
  { 
    auto best_pose=result_info[0];
    for(auto &pose:result_info)
    {
      if(pose.confident_index>best_pose.confident_index)
      {
        best_pose=pose;
      }
    }
    send_result(best_pose,header_info);
  }
}

void PerceptionStorage::provide_evidence(std::pair<cv::Mat,std::vector<intermediate>> &interm_info,
                                         std::vector<mpose> &target_info,std::vector<mpose> &result_info,
                                         std_msgs::msg::Header &header_info)
{
  if(pub_naive_region->get_subscription_count()>0)
  {
    pub_naive_region->publish(detect_machine->roi_region);
  }
  if(pub_depth_detect->get_subscription_count()>0)
  {
    cv_bridge::CvImage depth_msg;
    depth_msg.header=header_info;
    depth_msg.encoding = sensor_msgs::image_encodings::BGR8;
    depth_msg.image = interm_info.first;
    pub_depth_detect->publish(*depth_msg.toImageMsg());
  }
  if(pub_hypothesis_points->get_subscription_count()>0)
  {
    std::vector<intermediate> inter_points=interm_info.second;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr intermediate_points(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int i=0;i<inter_points.size();i++)
    {
      auto ipoints=inter_points[i];
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_points(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::copyPointCloud(*ipoints.source_points,*color_points);
      for(auto &cp:color_points->points)
      {
        cp.r=static_render[i][0];cp.g=static_render[i][1];cp.b=static_render[i][2];
      }
      *intermediate_points+=*color_points;
    }
    intermediate_points->width=intermediate_points->points.size();
    intermediate_points->height = 1;
    intermediate_points->is_dense = false;
    sensor_msgs::msg::PointCloud2 intermediate_msg;
    pcl::toROSMsg(*intermediate_points, intermediate_msg);
    intermediate_msg.height=intermediate_points->height;
    intermediate_msg.width=intermediate_points->width;
    intermediate_msg.header=header_info;
    pub_hypothesis_points->publish(intermediate_msg);
  }
  if(pub_hypothesis_pose->get_subscription_count()>0)
  {
    geometry_msgs::msg::PoseArray hypothesis_pose;
    hypothesis_pose.header=header_info;
    for(auto &pose_i:target_info)
    {
      geometry_msgs::msg::Pose target_pose=pose_i.pose.pose;
      hypothesis_pose.poses.emplace_back(target_pose);
    }
    pub_hypothesis_pose->publish(hypothesis_pose);
  }
  if(pub_evidence_points->get_subscription_count()>0)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr evidence_points(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int i=0;i<result_info.size();i++)
    {
      auto ipoints=result_info[i].target_points;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_points(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::copyPointCloud(*ipoints,*color_points);
      for(auto &cp:color_points->points)
      {
        cp.r=static_render[i][0];cp.g=static_render[i][1];cp.b=static_render[i][2];
      }
      *evidence_points+=*color_points;
    }
    evidence_points->width=evidence_points->points.size();
    evidence_points->height = 1;
    evidence_points->is_dense = false;
    sensor_msgs::msg::PointCloud2 evidence_msg;
    pcl::toROSMsg(*evidence_points, evidence_msg);
    evidence_msg.height=evidence_points->height;
    evidence_msg.width=evidence_points->width;
    evidence_msg.header=header_info;
    pub_evidence_points->publish(evidence_msg);
  }
  if(pub_evidence_pose->get_subscription_count()>0)
  {
    geometry_msgs::msg::PoseArray evidence_pose;
    evidence_pose.header=header_info;
    for(auto &pose_i:result_info)
    {
      geometry_msgs::msg::Pose target_pose=pose_i.pose.pose;
      evidence_pose.poses.emplace_back(target_pose);
    }
    pub_evidence_pose->publish(evidence_pose);
  }
}

void PerceptionStorage::send_result(mpose & target_pose,std_msgs::msg::Header &header_info)
{
  geometry_msgs::msg::PoseStamped result_pose=target_pose.pose;
  result_pose.header=header_info;
  pub_result->publish(result_pose);
  // 添加到本任务周期记录中
  task_remember_pose.header=header_info;
  geometry_msgs::msg::Pose add_pose=result_pose.pose;
  task_remember_pose.poses.emplace_back(add_pose);
  pub_remember_pose->publish(task_remember_pose);
}

void PerceptionStorage::post_process()
{
  auto detect_intermediate=detect_machine->get_intermediate();
  auto detect_targets=detect_machine->get_targets();
  auto detect_result=detect_machine->get_result();
  valid_hys+=std::min(1,int(detect_targets.size()));
  valid_re+=std::min(1,int(detect_result.size()));
  ++job_num;
  publish_msgs(detect_intermediate,detect_targets,detect_result,job_header);
}

void PerceptionStorage::running()
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("perception_storage"), "[PerceptionStorage][Perception] DetectionThread_Start "<<" SensorType_"<<sensor_type_str);
  while(rclcpp::ok() && worker_running)
  { 
    std::deque<MsgPacket> multiframes;
    if(parse_frame(multiframes))
    { 
      if(config_info.m_settinginfo.sensor_type==SensorType::TOF)
      {
        job_header=multiframes.back().tof_data.msg_header;
      }
      else if(config_info.m_settinginfo.sensor_type==SensorType::LIDAR)
      {
        job_header=multiframes.back().lidar_data.msg_header;
      }
      auto depth_points=couple_data(multiframes);
      detect_machine->inference(depth_points,job_header);
      post_process();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}
