#include "perception_storage/perception_storage.hpp"

PerceptionStorage::PerceptionStorage(ros::NodeHandle& pnh,ros::NodeHandle& nh):nh_(pnh),_nh_(nh),cam_left_ready(false),cam_right_ready(false)
{
  // 初始化
  init();
  // 注册服务
  perception_srv_=nh_.advertiseService(config_info.m_pubinfo.service_name, &PerceptionStorage::perception_service, this);
  event_client_=nh_.serviceClient<hecbot_srvs::JString>("/hecbot_event_tracer/report_event");
  sub_tracker_state=nh_.subscribe("/perception/storage/tracker/tracker_state", 1, &PerceptionStorage::tracker_cbk, this);
  pub_event=nh_.advertise<std_msgs::String>("/hecbot_perception/storage_detect_info",1);
  // 处理线程
  threads_pool.push_back(std::make_shared<boost::thread>(boost::bind(&PerceptionStorage::running, this)));
  // 检测器初始化
  if(config_info.m_settinginfo.sensor_type==SensorType::TOF)
  {
    detect_machine=std::make_shared<tof_detector>(config_info);
  }
  else if(config_info.m_settinginfo.sensor_type==SensorType::LIDAR)
  {
    detect_machine=std::make_shared<lidar_detector>(config_info);
  }
}

PerceptionStorage::~PerceptionStorage()
{
  for (int i=0; i != threads_pool.size(); i++)
  {
    threads_pool[i]->join();
  }
}

void PerceptionStorage::init()
{
  read_params();
  std::string t_frame="base_link";
  if(config_info.m_settinginfo.sensor_type==SensorType::TOF)
  {
    matrix_right=tool_box::read_tf(t_frame,frame_right,3.0);
    matrix_left=tool_box::read_tf(t_frame,frame_left,3.0);
  }
  else if(config_info.m_settinginfo.sensor_type==SensorType::LIDAR)
  {
    matrix_right=tool_box::read_tf(t_frame,frame_front,3.0);
    matrix_left=tool_box::read_tf(t_frame,frame_rear,3.0);
  }
  // 休眠状态
  active_=ActiveState::SLEEPY;
  set_processor();
  set_publishers();
  msg_packet.clear();
  process_queue.clear();
  job_num=0;
  valid_hys=0;
  valid_re=0;
  filter_machine=std::make_shared<points_filter>();
  int task_remeber=10;
  int job_remeber=100;
  analyzer_machine=std::make_shared<analyzer>(config_info,task_remeber,job_remeber);
}

bool PerceptionStorage::read_params()
{
  // lidar
  std::string topic_lidar_left;
  nh_.getParam("LidarSensors/points_left",topic_lidar_left);
  std::string topic_lidar_right;
  nh_.getParam("LidarSensors/points_right",topic_lidar_right);
  float width_lidar;
  nh_.getParam("LidarSensors/width",width_lidar);
  float height_lidar;
  nh_.getParam("LidarSensors/height",height_lidar);
  bool flip_lidar;
  nh_.getParam("LidarSensors/flip",flip_lidar);
  float compensation_lidar;
  nh_.getParam("LidarSensors/range_compenstion",compensation_lidar);
  SensorInfo lidar_info(SensorType::LIDAR,topic_lidar_left,topic_lidar_right,width_lidar,height_lidar,flip_lidar,compensation_lidar);
  // tof
  std::string topic_tof_left;
  nh_.getParam("TofSensors/points_left",topic_tof_left);
  std::string topic_tof_right;
  nh_.getParam("TofSensors/points_right",topic_tof_right);
  std::string topic_image_left;
  nh_.getParam("TofSensors/image_left",topic_image_left);
  std::string topic_image_right;
  nh_.getParam("TofSensors/image_right",topic_image_right);
  std::string topic_camera_left;
  nh_.getParam("TofSensors/camera_left",topic_camera_left);
  std::string topic_camera_right;
  nh_.getParam("TofSensors/camera_right",topic_camera_right);
  float width_tof;
  nh_.getParam("TofSensors/width",width_tof);
  float height_tof;
  nh_.getParam("TofSensors/height",height_tof);
  bool flip_tof;
  nh_.getParam("TofSensors/flip",flip_tof);
  float compensation_tof;
  nh_.getParam("TofSensors/range_compenstion",compensation_tof);
  SensorInfo tof_info(SensorType::TOF,
                      topic_tof_left,topic_tof_right,
                      topic_image_left,topic_image_right,
                      topic_camera_left,topic_camera_right,
                      width_tof,height_tof,
                      flip_tof,compensation_tof);
  // 货架
  float upper_length;
  nh_.getParam("TargetInfo/upper_layer/length",upper_length);
  float upper_width;
  nh_.getParam("TargetInfo/upper_layer/width",upper_width);
  float upper_height;
  nh_.getParam("TargetInfo/upper_layer/height",upper_height);
  float lower_length;
  nh_.getParam("TargetInfo/lower_layer/length",lower_length);
  float lower_width;
  nh_.getParam("TargetInfo/lower_layer/width",lower_width);
  float lower_height;
  nh_.getParam("TargetInfo/lower_layer/height",lower_height);
  std::string reference_feature;
  nh_.getParam("TargetInfo/supplementary/feature",reference_feature);
  std::vector<float> feature_params;
  nh_.getParam("TargetInfo/supplementary/paras",feature_params);
  TargetInfo target_info(upper_length,upper_width,upper_height,lower_length,lower_width,lower_height,reference_feature,feature_params);
  // 参数
  bool trouble_shoting;
  nh_.getParam("SettingPara/trouble_shoting",trouble_shoting);
  std::string sensor_type;
  nh_.getParam("SettingPara/sensor_type",sensor_type);
  sensor_type_str=sensor_type;
  std::vector<int> batch_size_list;
  nh_.getParam("SettingPara/batch_size",batch_size_list);
  bool reference_pose;
  nh_.getParam("SettingPara/reference_pose",reference_pose);
  int stop_criteria;
  nh_.getParam("SettingPara/stop_criteria",stop_criteria);
  SettingInfo setting_info(trouble_shoting,sensor_type,batch_size_list,reference_pose,stop_criteria);
  // 消息
  std::string service_name;
  nh_.getParam("PubInfo/service_name",service_name);
  std::string result_name;
  nh_.getParam("PubInfo/result_name",result_name);
  std::string reference_name;
  nh_.getParam("PubInfo/reference_name",reference_name);
  std::string refer_region_name;
  nh_.getParam("PubInfo/refer_region_name",refer_region_name);
  std::string naive_region_name;
  nh_.getParam("PubInfo/naive_region_name",naive_region_name);
  std::string raw_name;
  nh_.getParam("PubInfo/raw_name",raw_name);
  std::string depth_name;
  nh_.getParam("PubInfo/depth_name",depth_name);
  std::string hyp_points_name;
  nh_.getParam("PubInfo/hyp_points_name",hyp_points_name);
  std::string hyp_pose_name;
  nh_.getParam("PubInfo/hyp_pose_name",hyp_pose_name);
  std::string evidence_points_name;
  nh_.getParam("PubInfo/evidence_points_name",evidence_points_name);
  std::string evidence_pose_name;
  nh_.getParam("PubInfo/evidence_pose_name",evidence_pose_name);
  std::string remember_pose_name;
  nh_.getParam("PubInfo/remember_pose_name",remember_pose_name);
  std::string repub_points_name;
  nh_.getParam("PubInfo/repub_points_name",repub_points_name);
  std::string repub_image_name;
  nh_.getParam("PubInfo/repub_image_name",repub_image_name);
  std::string lidar_capture_name;
  nh_.getParam("PubInfo/lidar_capture_name",lidar_capture_name);
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
  // 坐标系
  nh_.getParam("FrameNames/front_frame",frame_front);
  nh_.getParam("FrameNames/rear_frame",frame_rear);
  nh_.getParam("FrameNames/right_frame",frame_right);
  nh_.getParam("FrameNames/left_frame",frame_left);

  return true;
}

void PerceptionStorage::set_processor()
{
  std::string sensor_left="left";
  std::string sensor_right="right";
  // tof
  if(config_info.m_settinginfo.sensor_type==SensorType::TOF)
  {
    // 相机内参
    boost::function<void (sensor_msgs::CameraInfo)> bleft=boost::bind(&PerceptionStorage::cam_callback,this,_1,sensor_left);
    cam_info_left=nh_.subscribe<sensor_msgs::CameraInfo>(config_info.m_tofinfo.cam_left,2,bleft);
    boost::function<void (sensor_msgs::CameraInfo)> bright=boost::bind(&PerceptionStorage::cam_callback,this,_1,sensor_right);
    cam_info_right=nh_.subscribe<sensor_msgs::CameraInfo>(config_info.m_tofinfo.cam_right,2,bright);
    // 图像与点云
    sub_tof_points_left.subscribe(nh_,config_info.m_tofinfo.topic_left,2);
    sub_tof_points_right.subscribe(nh_,config_info.m_tofinfo.topic_right,2);
    sub_tof_gray_left.subscribe(nh_,config_info.m_tofinfo.gray_left,2);
    sub_tof_gray_right.subscribe(nh_,config_info.m_tofinfo.gray_right,2);
    sync_tof_left.reset(new tof_sync(SyncPolicy_tof(5),sub_tof_points_left,sub_tof_gray_left));
    sync_tof_right.reset(new tof_sync(SyncPolicy_tof(5),sub_tof_points_right,sub_tof_gray_right));
    sync_tof_left->registerCallback(boost::bind(&PerceptionStorage::process_tof,this, _1, _2,sensor_left));
    sync_tof_right->registerCallback(boost::bind(&PerceptionStorage::process_tof,this, _1, _2,sensor_right));
  }
  // lidar
  else if(config_info.m_settinginfo.sensor_type==SensorType::LIDAR)
  {
    boost::function<void (sensor_msgs::PointCloud2ConstPtr)> bleft=boost::bind(&PerceptionStorage::process_lidar,this,_1,sensor_left);
    sub_livox_left=nh_.subscribe<sensor_msgs::PointCloud2ConstPtr>(config_info.m_lidarinfo.topic_left,2,bleft);
    boost::function<void (sensor_msgs::PointCloud2ConstPtr)> bright=boost::bind(&PerceptionStorage::process_lidar,this,_1,sensor_right);
    sub_livox_right=nh_.subscribe<sensor_msgs::PointCloud2ConstPtr>(config_info.m_lidarinfo.topic_right,2,bright);
  }
}

void PerceptionStorage::set_publishers()
{
  // 结果
  pub_result=nh_.advertise<geometry_msgs::PoseStamped>(config_info.m_pubinfo.result_name, 1);
  pub_naive_region=nh_.advertise<visualization_msgs::MarkerArray>(config_info.m_pubinfo.naive_region_name, 1);
  // 处理点
  pub_base_points=nh_.advertise<sensor_msgs::PointCloud2>(config_info.m_pubinfo.raw_name, 1);
  // 深度图检测结果
  pub_depth_detect=nh_.advertise<sensor_msgs::Image>(config_info.m_pubinfo.depth_name, 1);
  // 候选点
  pub_hypothesis_points=nh_.advertise<sensor_msgs::PointCloud2>(config_info.m_pubinfo.hyp_points_name, 1);
  // 候选姿态
  pub_hypothesis_pose=nh_.advertise<geometry_msgs::PoseArray>(config_info.m_pubinfo.hyp_pose_name, 1);
  // 满足条件的目标点
  pub_evidence_points=nh_.advertise<sensor_msgs::PointCloud2>(config_info.m_pubinfo.evidence_points_name, 1);
  // 满足条件的姿态
  pub_evidence_pose=nh_.advertise<geometry_msgs::PoseArray>(config_info.m_pubinfo.evidence_pose_name, 1);
  // 一次任务的姿态记忆
  pub_remember_pose=nh_.advertise<geometry_msgs::PoseArray>(config_info.m_pubinfo.remember_pose_name, 1);
  // 转发原始数据
  pub_repub_points=nh_.advertise<sensor_msgs::PointCloud2>(config_info.m_pubinfo.repub_points_name, 1);
  pub_repub_image=nh_.advertise<sensor_msgs::Image>(config_info.m_pubinfo.repub_image_name, 1);
  // 雷达对货架底座的观测
  pub_lidar_capture=nh_.advertise<sensor_msgs::PointCloud2>(config_info.m_pubinfo.lidar_capture_name, 1);
  // 调试输出：发布送入参考短线RANSAC之前的完整聚类点。
  pub_reference_cloud_cluster=nh_.advertise<sensor_msgs::PointCloud2>("/perception/debug/reference/cloud_cluster",1);
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
}
 
depth_mapping PerceptionStorage::couple_lidar(std::deque<MsgPacket> &multiframes)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr concate_points(new pcl::PointCloud<pcl::PointXYZI>);
  auto pub_header_msg=multiframes.back().lidar_data.msg_header;
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
  ROS_INFO_STREAM("[PerceptionStorage][Perception] CoupleLidarFrames");
  // 滤波
  filter_machine->sampling_filter(concate_points,concate_points,"uniform",0.0025f);
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
  // planar
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
  ROS_INFO_STREAM("[PerceptionStorage][Perception] cluster_indices "<<cluster_indices.size());
  int cluster_index=0;
  std::vector<Eigen::VectorXf> proposal_coef;
  std::vector<pcl::PointXYZ> proposal_mid;
  for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    cluster_index++;
    const std::vector<int> &index_vec = it->indices;
    auto direct_dist=pcl::euclideanDistance(planar_lidar_filter->points[index_vec.front()],planar_lidar_filter->points[index_vec.back()]);
    float  dd=0.5*config_info.m_targetinfo.params[0];
    if(fabs(direct_dist-config_info.m_targetinfo.params[0])<=dd)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
      for(std::vector<int>::const_iterator pit = index_vec.begin(); pit != index_vec.end(); ++pit)
      {
        int cp_index=*pit;
        auto &cp_p=planar_lidar_filter->points[cp_index];
        cloud_cluster->points.push_back(cp_p);
      }
      // hhy add. 2026.09.24
      // 在RANSAC筛选前发布完整cloud_cluster，保留点带厚度、密度和离群点等原始形态。
      if(pub_reference_cloud_cluster.getNumSubscribers()>0)
      {
        cloud_cluster->width=cloud_cluster->points.size();
        cloud_cluster->height=1;
        cloud_cluster->is_dense=false;
        sensor_msgs::PointCloud2 cloud_cluster_msg;
        pcl::toROSMsg(*cloud_cluster,cloud_cluster_msg);
        cloud_cluster_msg.header=pub_header_msg;
        pub_reference_cloud_cluster.publish(cloud_cluster_msg);
      }
      pcl::SampleConsensusModelLine<pcl::PointXYZI>::Ptr sim_model(new pcl::SampleConsensusModelLine<pcl::PointXYZI>(cloud_cluster));	
      pcl::RandomSampleConsensus<pcl::PointXYZI> line_fit(sim_model);	
      line_fit.setDistanceThreshold(0.01);
      line_fit.setMaxIterations(100);
      line_fit.computeModel();
      std::vector<int> line_inliers_;
      line_fit.getInliers(line_inliers_);
      if(line_inliers_.size()<10 || line_inliers_.size()<0.8*index_vec.size())
      {
        continue;
      }
      Eigen::VectorXf line_coef_;
      line_fit.getModelCoefficients(line_coef_);
      // hhy add. 2026.09.24
      // RANSAC 初始直线模型受随机采样点影响，导致相邻帧间yaw抖动和漏检问题
      // 使用全部内点重新优化直线系数，避免随机样本斜率抖动使width_diagonal临界失效
      if(line_coef_.size() == 6 && line_inliers_.size() > 2) {
        Eigen::VectorXf optimized_line_coef_;
        sim_model->optimizeModelCoefficients(line_inliers_, line_coef_, optimized_line_coef_);
        if(optimized_line_coef_.size() == 6) {
          line_coef_ = optimized_line_coef_;
        }
      }

      Eigen::Vector3f line_dir_={line_coef_[3],line_coef_[4],line_coef_[5],};
      Eigen::Vector3f line_pt_={line_coef_[0],line_coef_[1],line_coef_[2]};
      std::vector<int> with_line,width_diagonal;
      if(index_vec.front()>1)
      {
        for(int pk=0;pk<index_vec.front()-1;pk++)
        {
          Eigen::Vector4f pt_4f(planar_lidar_filter->points[pk].x,planar_lidar_filter->points[pk].y,planar_lidar_filter->points[pk].z,0);
          float p2line =std::sqrt(pcl::sqrPointToLineDistance(pt_4f,
                                 Eigen::Vector4f(line_coef_(0),line_coef_(1),line_coef_(2),0),
                                 Eigen::Vector4f(line_coef_(3),line_coef_(4),line_coef_(5),0)));
          if(p2line<0.02)
          {
            float dist_left=pcl::euclideanDistance(planar_lidar_filter->points[index_vec.front()],planar_lidar_filter->points[pk]);
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
            float p_p_dist=Eigen::Vector3f(planar_lidar_filter->points[index_vec.front()].x-px_,
                                          planar_lidar_filter->points[index_vec.front()].y-py_,
                                          planar_lidar_filter->points[index_vec.front()].z-pz_).norm();
            if(fabs(p_p_dist-(0.5*config_info.m_targetinfo.lower_length-0.5*config_info.m_targetinfo.params[0]))<0.05)
            {
              width_diagonal.push_back(pk);
            }
          }
        }
      }
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
      ROS_INFO_STREAM("[PerceptionStorage][Perception] width_diagonal "<<width_diagonal.size()<<" with_line "<<with_line.size());
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
      ROS_INFO_STREAM("[PerceptionStorage][Perception] distres "<<fabs(direct_dist-config_info.m_targetinfo.params[0])<<" dd "<<dd);
    }
  }
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
    T_lidar_matrix=T_lidar_matrix.inverse();
  }
  else
  {
    T_lidar_matrix.topLeftCorner<3, 3>()=(Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ())).toRotationMatrix();
    T_lidar_matrix.block<3,1>(0,3)=Eigen::Vector3f(0.5*config_info.m_targetinfo.lower_length,0,0);
  }
  ROS_INFO_STREAM("[PerceptionStorage][Perception] DepthReference "<<reference_init<<" "<<proposal_coef.size()<<" "<<proposal_mid.size());
  // 变换为正视图，方便后续2d转换
  pcl::PointCloud<pcl::PointXYZI>::Ptr trans_lidar(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*concate_points,*trans_lidar,T_lidar_matrix);
  Eigen::Vector3f refer_mid_point;
  if(proposal_mid.size()==1)
  {
    Eigen::Vector4f mid_point(proposal_mid[0].x,proposal_mid[0].y,proposal_mid[0].z,1);
    mid_point=T_lidar_matrix*mid_point;
    refer_mid_point=mid_point.head<3>();
  }
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
  //
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
    float dist_range=((max_dist-min_dist)>config_info.m_targetinfo.lower_width)?(max_dist-0.1):(min_dist+0.6*config_info.m_targetinfo.lower_width);
    for(int i=0;i<dist_arrray.size();i++)
    {
      if(dist_arrray[i]<=dist_range)
      {
        ray_filter->points.emplace_back(theta_x.second.points[i]);
      }
    }
  }
  // 过滤一次
  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud(ray_filter);
  sor.setMeanK(20);
  sor.setStddevMulThresh(1);
  sor.filter(*ray_filter);
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
  base_image=smooth_range;
  cv::threshold(base_image,base_image,1,255,cv::THRESH_BINARY);
  pcl::PointCloud<pcl::PointXYZI>::Ptr back_origin_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*ray_filter,*back_origin_ptr,(T_lidar_matrix.inverse().cast<float>()));
  pcl::PointCloud<pcl::PointXYZI>::Ptr cues_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud(*back_origin_ptr,*cues_ptr);
  //
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
    sensor_msgs::PointCloud2 ray_filter_msg;
    pcl::toROSMsg(*planar_lidar_filter, ray_filter_msg);
    ray_filter_msg.height=planar_lidar_filter->height;
    ray_filter_msg.width=planar_lidar_filter->width;
    ray_filter_msg.header=pub_header_msg;
    pub_lidar_capture.publish(ray_filter_msg);
  }
  ROS_INFO_STREAM("[PerceptionStorage][Perception] LidarData_Finish");

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

void PerceptionStorage::cam_callback(const sensor_msgs::CameraInfo &msg,
                                     std::string &left_or_right)
{
  ROS_INFO_STREAM("[PerceptionStorage][Perception] Heared "<<left_or_right<<" camera_info");
  cv::Mat cameraMatrix(3, 3, CV_64FC1);
  cv::Mat distorsionCoeff(4, 1, CV_64FC1);
  cv::Size size(msg.height, msg.width);
  for(int i=0; i<9; ++i)
  {
    cameraMatrix.at<double>(i%3, i-(i%3)*3) = msg.K[i];
  }
  if(msg.D.size() >= 4)
  {
    for(int i=0; i<4; ++i)
    {
      distorsionCoeff.at<double>(i, 0) = msg.D[i];
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
    cam_info_left.shutdown();
  }
  else if(left_or_right=="right")
  {
    tof_cam_right=CameraInfo(cameraMatrix,distorsionCoeff,size);
    cam_right_ready=true;
    cam_info_right.shutdown();
  }
}

void PerceptionStorage::process_tof(const sensor_msgs::PointCloud2ConstPtr &points,
                                    const sensor_msgs::ImageConstPtr &image,
                                    std::string &which_side)
{
  if(which_side=="left")
  {
   if(active_==ActiveState::LEFT && cam_left_ready)
   {
    ROS_INFO_STREAM("[PerceptionStorage][Perception] Heared_LeftTofMsg");
    int sensor_side_=1;
    MsgPacket msg_pack_(sensor_side_,tof_cam_left,image,points); 
    insert_buffer(msg_pack_);
    pub_repub_points.publish(points);
    pub_repub_image.publish(image);
   }
  }
  else
  {
   if(active_==ActiveState::RIGHT && cam_right_ready)
   {
    ROS_INFO_STREAM("[PerceptionStorage][Perception] Heared_RightTofMsg");
    int sensor_side_=2;
    MsgPacket msg_pack_(sensor_side_,tof_cam_right,image,points);
    insert_buffer(msg_pack_);
    pub_repub_points.publish(points);
    pub_repub_image.publish(image);
   }
  }
}

void PerceptionStorage::process_lidar(const sensor_msgs::PointCloud2ConstPtr &points,
                                      std::string &which_side)
{
  if(which_side=="left")
  {
   if(active_==ActiveState::LEFT)
   { 
    ROS_INFO_STREAM("[PerceptionStorage][Perception] Heared_LeftLidarMsg");
    int sensor_side_=1;
    MsgPacket msg_pack_(sensor_side_,points); 
    insert_buffer(msg_pack_);
    pub_repub_points.publish(points);
   }
  }
  else
  {
   if(active_==ActiveState::RIGHT)
   {
    ROS_INFO_STREAM("[PerceptionStorage][Perception] Heared_RightLidarMsg");
    int sensor_side_=2;
    MsgPacket msg_pack_(sensor_side_,points);
    insert_buffer(msg_pack_);
    pub_repub_points.publish(points);
   }
  }
}

bool PerceptionStorage::perception_service(hecbot_srvs::JString::Request &req, hecbot_srvs::JString::Response &res)
{
  std::unique_lock<std::mutex> lock(_mtx_);
  std::string req_info=req.request;
  Json::Reader reader;
  Json::Value  value;
  reader.parse(req_info,value);
  std::string current_action=value["detect_op"].asString();
  if(current_action=="start")
  {
    std::string sensor_side_=value["side_option"].asString();
    std::string cabinet_model_=value["cabinet_model"].asString();
    ROS_ERROR_STREAM("[PerceptionStorage][Perception] cabinet_model "<<cabinet_model_);
    auto pose_info=value["reference_pose"];
    std::vector<float> pose_v{pose_info["x"].asFloat(),pose_info["y"].asFloat(),pose_info["th"].asFloat()};
    if(sensor_side_=="left" || sensor_side_=="right")
    {
      start(sensor_side_,pose_v);
    }
    else
    {
      ROS_ERROR_STREAM("[PerceptionStorage][Perception] UnknownSide "<<sensor_side_);
      res.success=false;
      return  true;
    } 
  }
  else if(current_action=="stop")
  {
    ROS_INFO_STREAM("[PerceptionStorage][Perception] RequestStop "<<current_action);
    stop();
  }
  res.success=true;
  return  true;
}

void PerceptionStorage::start(std::string &side_,std::vector<float> &pose_)
{
  job_num=0;
  valid_hys=0;
  valid_re=0;
  req_info_=ReqInfo(side_,pose_);
  msg_packet.clear();
  process_queue.clear();
  matrix_base=Eigen::Matrix4d::Identity();
  task_remember_pose.poses.clear();
  if(side_=="left")
  {
    active_=ActiveState::LEFT;
    matrix_base=matrix_left;
  }
  else if(side_=="right")
  {
    active_=ActiveState::RIGHT;
    matrix_base=matrix_right;
  }
  ROS_INFO_STREAM("[PerceptionStorage][Perception] RequestStart "<<side_<<"_"<<pose_[0]<<"_"<<pose_[1]<<"_"<<pose_[2]);
  analyzer_machine->manager();
}

void PerceptionStorage::stop()
{
  std::string s_sensor,s_side;
  if(active_==ActiveState::RIGHT)
  {
    s_side="right";
  }
  else if(active_==ActiveState::LEFT)
  {
    s_side="left";
  }
  if(config_info.m_settinginfo.sensor_type==SensorType::TOF)
  {
    s_sensor="tof";
  }
  else if(config_info.m_settinginfo.sensor_type==SensorType::LIDAR)
  {
    s_sensor="lidar";
  }
  active_=ActiveState::SLEEPY;
  msg_packet.clear();
  process_queue.clear();
  send_event(s_sensor,s_side);
  job_num=0;
  valid_hys=0;
  valid_re=0;
  matrix_base=Eigen::Matrix4d::Identity();
  task_remember_pose.poses.clear();
  // writer info
  auto task_msg=analyzer_machine->writer_task(s_sensor,s_side);
}

void PerceptionStorage::tracker_cbk(const std_msgs::String &msg)
{
  Json::Value json_msg;
  json_msg["time"] = ros::Time::now().toSec();
  json_msg["processor"]="tracker";
  json_msg["message"] = msg.data;
  Json::StyledWriter writer;
  std::string json_str = writer.write(json_msg);
  std_msgs::String ros_msg;
  ros_msg.data=json_str;
  pub_event.publish(ros_msg);
}

void PerceptionStorage::send_event(std::string &sensor,std::string &side)
{
  ROS_INFO_STREAM("[PerceptionStorage][Perception] TaskStatistics "<<job_num<<"_"<<valid_hys<<"_"<<valid_re);
  int event_id=1700;
  // 没有结果
  if(valid_re==0)
  {
    event_id=1701;
  }
  Json::Value json_msg;
  json_msg["time"] = ros::Time::now().toSec();
  json_msg["processor"]="detection";
  std::string str_msg;
  if(event_id==1700)
  {
    str_msg="storage_detection_success#"+sensor+"_"+side;
  }
  else
  {
    str_msg="storage_detection_failed#"+sensor+"_"+side;
    // 没有进行推理=>没有数据
    if(job_num==0)
    {
      str_msg=str_msg+"#"+"no_sensor_message";
    }
    // 推理，但是候选为空=>没有目标
    else if(valid_hys==0)
    {
      str_msg=str_msg+"#"+"no_hypothesis_target";
    }
    // 有候选，但是没有结果=>没有目标或者数据质量不行
    else if(valid_re==0)
    {
      str_msg=str_msg+"#"+"no_valid_target";
    }
  }
  json_msg["message"] = str_msg;
  Json::StyledWriter writer;
  std::string json_str = writer.write(json_msg);
  std_msgs::String ros_msg;
  ros_msg.data=json_str;
  pub_event.publish(ros_msg);
}

bool PerceptionStorage::stop_self()
{
  if(job_num>config_info.m_settinginfo.stop_criteria)
  {
    stop();
    ROS_INFO_STREAM("[PerceptionStorage][Perception] StopByCriteria");
    return true;
  }
  else
  {
    return false;
  }
}

void PerceptionStorage::publish_msgs(std::pair<cv::Mat,std::vector<intermediate>> &interm_info,
                                    std::vector<mpose> &target_info,std::vector<mpose> &result_info,
                                    std_msgs::Header &header_info)
{
  provide_evidence(interm_info,target_info,result_info,header_info);
  pcl::PointCloud<pcl::PointXYZI>::Ptr analyzer_points(new pcl::PointCloud<pcl::PointXYZI>);
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
    pcl::copyPointCloud(*best_pose.target_points,*analyzer_points);
  }
  analyzer_machine->writer_job(interm_info.first,analyzer_points,1);
}

void PerceptionStorage::provide_evidence(std::pair<cv::Mat,std::vector<intermediate>> &interm_info,
                                         std::vector<mpose> &target_info,std::vector<mpose> &result_info,
                                         std_msgs::Header &header_info)
{
  if(pub_naive_region.getNumSubscribers()>0)
  {
    pub_naive_region.publish(detect_machine->roi_region);
  }
  if(pub_depth_detect.getNumSubscribers()>0)
  {
    cv_bridge::CvImage depth_msg;
    depth_msg.header=header_info;
    depth_msg.encoding = sensor_msgs::image_encodings::BGR8;
    depth_msg.image = interm_info.first;
    pub_depth_detect.publish( depth_msg );
  }
  if(pub_hypothesis_points.getNumSubscribers()>0)
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
    sensor_msgs::PointCloud2 intermediate_msg;
    pcl::toROSMsg(*intermediate_points, intermediate_msg);
    intermediate_msg.height=intermediate_points->height;
    intermediate_msg.width=intermediate_points->width;
    intermediate_msg.header=header_info;
    pub_hypothesis_points.publish(intermediate_msg);
  }
  if(pub_hypothesis_pose.getNumSubscribers()>0)
  {
    geometry_msgs::PoseArray hypothesis_pose;
    hypothesis_pose.header=header_info;
    for(auto &pose_i:target_info)
    {
      geometry_msgs::Pose target_pose=pose_i.pose.pose;
      hypothesis_pose.poses.emplace_back(target_pose);
    }
    pub_hypothesis_pose.publish(hypothesis_pose);
  }
  if(pub_evidence_points.getNumSubscribers()>0)
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
    sensor_msgs::PointCloud2 evidence_msg;
    pcl::toROSMsg(*evidence_points, evidence_msg);
    evidence_msg.height=evidence_points->height;
    evidence_msg.width=evidence_points->width;
    evidence_msg.header=header_info;
    pub_evidence_points.publish(evidence_msg);
  }
  if(pub_evidence_pose.getNumSubscribers()>0)
  {
    geometry_msgs::PoseArray evidence_pose;
    evidence_pose.header=header_info;
    for(auto &pose_i:result_info)
    {
      geometry_msgs::Pose target_pose=pose_i.pose.pose;
      evidence_pose.poses.emplace_back(target_pose);
    }
    pub_evidence_pose.publish(evidence_pose);
  }
}

void PerceptionStorage::send_result(mpose & target_pose,std_msgs::Header &header_info)
{
  geometry_msgs::PoseStamped result_pose=target_pose.pose;
  result_pose.header=header_info;
  pub_result.publish(result_pose);
  // 添加到本任务周期记录中
  task_remember_pose.header=header_info;
  geometry_msgs::Pose add_pose=result_pose.pose;
  task_remember_pose.poses.emplace_back(add_pose);
  pub_remember_pose.publish(task_remember_pose);
  Eigen::Matrix4f transform=matrix_base.cast<float>()*target_pose.transform;
  Eigen::Matrix3f rotate_matrix=transform.topLeftCorner<3, 3>();
  Eigen::Vector3f angles=rotate_matrix.eulerAngles(2,1,0);
  Eigen::Vector2d data_1;
  Eigen::Vector3i data_2;
  Eigen::Vector3f data_3(angles[0]/M_PI*180.0,100.0*transform(0,3),100.0*transform(1,3));
  auto analyzer_data=std::make_tuple(data_1,data_2,data_3);
  int analyzer_flag=3;
  analyzer_machine->update(analyzer_data,analyzer_flag);
}

void PerceptionStorage::post_process()
{
  auto detect_intermediate=detect_machine->get_intermediate();
  auto detect_targets=detect_machine->get_targets();
  auto detect_result=detect_machine->get_result();
  valid_hys+=std::min(1,int(detect_targets.size()));
  valid_re+=std::min(1,int(detect_result.size()));
  Eigen::Vector2d data_1;
  Eigen::Vector3i data_2(++job_num,valid_hys,valid_re);
  Eigen::Vector3f data_3;
  auto analyzer_data=std::make_tuple(data_1,data_2,data_3);
  int analyzer_flag=2;
  analyzer_machine->update(analyzer_data,analyzer_flag);
  publish_msgs(detect_intermediate,detect_targets,detect_result,job_header);
}

void PerceptionStorage::running()
{
  ROS_INFO_STREAM("[PerceptionStorage][Perception] DetectionThread_Start "<<" SensorType_"<<sensor_type_str);
  while(ros::ok())
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
      // 预处理
      typedef std::chrono::duration<double, std::ratio<1, 1000>> ms;
      auto t1 = std::chrono::high_resolution_clock::now();
      auto depth_points=couple_data(multiframes);
      auto t2 = std::chrono::high_resolution_clock::now();
      auto pre_cost = std::chrono::duration_cast<ms>(t2 - t1).count();
      // 检测
      t1 = std::chrono::high_resolution_clock::now();
      detect_machine->inference(depth_points,job_header);
      t2 = std::chrono::high_resolution_clock::now();
      auto infer_cost = std::chrono::duration_cast<ms>(t2 - t1).count();
      {
        Eigen::Vector2d time_cost(pre_cost,infer_cost);
        Eigen::Vector3i data_1;
        Eigen::Vector3f data_2;
        auto analyzer_data=std::make_tuple(time_cost,data_1,data_2);
        int analyzer_flag=1;
        analyzer_machine->update(analyzer_data,analyzer_flag);
        analyzer_machine->writer_job(depth_points.depth,depth_points.points,0);
      }
      // 后处理
      post_process();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}
