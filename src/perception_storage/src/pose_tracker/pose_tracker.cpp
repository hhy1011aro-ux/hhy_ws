#include "perception_storage/pose_tracker/pose_tracker.hpp"

namespace pose_tracker
{
  pose_tracker::pose_tracker(ros::NodeHandle& pnh,tf::TransformListener *_tf_):nh_(pnh),tf(_tf_)
  {
    set_config(base_config);
    boost::function<void (sensor_msgs::PointCloud2ConstPtr)> sub_left=boost::bind(&pose_tracker::sensor_cbk,this,_1,"left");
    left_subscriber=nh_.subscribe<sensor_msgs::PointCloud2ConstPtr>(base_config.m_lidarinfo.topic_left,2,sub_left);
    boost::function<void (sensor_msgs::PointCloud2ConstPtr)> sub_right=boost::bind(&pose_tracker::sensor_cbk,this,_1,"right");
    right_subscriber=nh_.subscribe<sensor_msgs::PointCloud2ConstPtr>(base_config.m_lidarinfo.topic_right,2,sub_right);
    cmd_subscriber=nh_.subscribe("/hecbot_perception/storage_goal_pose", 1, &pose_tracker::cmd_cbk, this);
    pub_points=nh_.advertise<sensor_msgs::PointCloud2>("/perception/storage/tracker/measurement", 1);
    pub_reference_pose=nh_.advertise<geometry_msgs::PoseStamped>("/perception/storage/tracker/reference", 1);
    pub_target_points=nh_.advertise<sensor_msgs::PointCloud2>("/perception/storage/tracker/target_points", 1);
    pub_pose=nh_.advertise<geometry_msgs::PoseStamped>("/perception/storage/tracker/pose", 1);
    pub_pose_robot=nh_.advertise<geometry_msgs::PoseStamped>("/perception/storage/tracker/robot_pose", 1);
    pub_tracker_state=nh_.advertise<std_msgs::String>("/perception/storage/tracker/tracker_state", 1);
    this->track_thread=std::thread(&pose_tracker::track,this);
    this->track_thread.detach();
  }
  
  void pose_tracker::set_config(ConfigInfo &config_info)
  {
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
    bool trouble_shoting;
    nh_.getParam("SettingPara/trouble_shoting",trouble_shoting);
    std::string sensor_type;
    nh_.getParam("SettingPara/sensor_type",sensor_type);
    std::vector<int> batch_size_list;
    nh_.getParam("SettingPara/batch_size",batch_size_list);
    bool reference_pose;
    nh_.getParam("SettingPara/reference_pose",reference_pose);
    int stop_criteria;
    nh_.getParam("SettingPara/stop_criteria",stop_criteria);
    SettingInfo setting_info(trouble_shoting,sensor_type,batch_size_list,reference_pose,stop_criteria);
    config_info.m_lidarinfo=lidar_info;
    config_info.m_targetinfo=target_info;
    config_info.m_settinginfo=setting_info;
    // 恒定坐标系变换
    std::string right_frame,left_frame;
    nh_.getParam("FrameNames/front_frame",right_frame);
    nh_.getParam("FrameNames/rear_frame",left_frame);
    side_table.emplace(std::make_pair("right",right_frame));
    side_table.emplace(std::make_pair("left",left_frame));
    transform_right=tool_box::read_tf("base_link",right_frame,3.0);
    transform_table.emplace(std::make_pair("right",transform_right));
    transform_left=tool_box::read_tf("base_link",left_frame,3.0);
    transform_table.emplace(std::make_pair("left",transform_left));
    // 通道宽度
    tunnel_width=0.495;
    // 通道横梁厚度
    tunnel_thickness=0.085;
    // 使用的帧数
    frames_num=5;
    // 世界坐标系
    world_frame="odom";
    // 局部重构
    using_local_map=false;
    // 停止更新的距离
    stop_distance=0.5;
    // 丢失标识
    lost_flag=0;
  }

  void pose_tracker::sensor_cbk(const sensor_msgs::PointCloud2ConstPtr &msg,std::string side_info)
  {
    if(side_info==tracking_info.target_data.side_info)
    {
      geometry_msgs::PoseStamped _pose_,kinematic_pose;
      _pose_.header.frame_id="base_link";
      _pose_.header.stamp=msg->header.stamp;
      _pose_.pose.orientation.w=1.0;
      if(trans_pose_world(_pose_,kinematic_pose,true,"odom"))
      {
        sensor_buffer.emplace_back(sensor_measurement(msg,kinematic_pose,transform_table[side_info]));
      }
      while(sensor_buffer.size()>frames_num)
      {
        sensor_buffer.pop_front();
      }
    }
  }
  
  void pose_tracker::cmd_cbk(const geometry_msgs::PoseStamped &msg)
  {
    std::unique_lock<std::mutex> lock(cmd_mtx);
    sensor_buffer.clear();
    tracking_info=tracking_trajectory();
    lost_flag=0;
    geometry_msgs::PoseStamped pose_base=msg;
    geometry_msgs::PoseStamped pose_world;
    if(trans_pose_world(pose_base,pose_world,false,"odom"))
    {
      pub_reference_pose.publish(pose_world);
      tracking_pose_init(pose_base,pose_world);
      ROS_INFO("[PerceptionStorage][PoseTracker] TrackerInit_OK");
    }
    else
    {
      ROS_ERROR("[PerceptionStorage][PoseTracker] TrackerInit_Failed");
    }
  }

  bool pose_tracker::trans_pose_world(geometry_msgs::PoseStamped &pose_in,geometry_msgs::PoseStamped &pose_out,
                                      bool base,std::string target_name)
  {
    tf::Pose pose(tf::Quaternion(pose_in.pose.orientation.x,pose_in.pose.orientation.y,pose_in.pose.orientation.z,
                                 pose_in.pose.orientation.w),
                  tf::Vector3(pose_in.pose.position.x,pose_in.pose.position.y,pose_in.pose.position.z));
    if(!base)
    {
      tf::Pose de_compensation;
      de_compensation.setIdentity();
      de_compensation.setOrigin(tf::Vector3(base_config.m_lidarinfo.distance_compensation,0,0));
      pose=pose*de_compensation;
    }
    std::string source_name=pose_in.header.frame_id;
    target_name="odom";
    tf::Stamped<tf::Pose> pose_stamped(pose,pose_in.header.stamp,source_name);
    tf::Stamped<tf::Pose> tmp(pose,pose_in.header.stamp,source_name);
    try
    {
      if(tf->waitForTransform(target_name,source_name,pose_in.header.stamp,ros::Duration(0.2)))
      {
        tf->transformPose(target_name,tmp,pose_stamped);
      }
      else
      {
        ROS_ERROR("[PerceptionStorage][PoseTracker] TransPose_Failed");
        return false;
      }
    }
    catch(const std::exception& e)
    {
      ROS_ERROR_STREAM("[PerceptionStorage][PoseTracker] TF "<<e.what());
    }
    pose_out.pose.orientation.w=pose_stamped.getRotation().getW();
    pose_out.pose.orientation.x=pose_stamped.getRotation().getX();
    pose_out.pose.orientation.y=pose_stamped.getRotation().getY();
    pose_out.pose.orientation.z=pose_stamped.getRotation().getZ();
    pose_out.pose.position.x=pose_stamped.getOrigin()[0];
    pose_out.pose.position.y=pose_stamped.getOrigin()[1];
    pose_out.pose.position.z=pose_stamped.getOrigin()[2];
    pose_out.header.stamp=pose_in.header.stamp;
    pose_out.header.frame_id=target_name;

    return true;
  }

  bool pose_tracker::trans_points_world(sensor_measurement &measurement)
  {
    auto trans_pose=measurement.kinematic_pose;
    Eigen::Matrix4d transform_world=Eigen::Matrix4d::Identity();
    Eigen::Quaterniond q_world(trans_pose.pose.orientation.w,
                               trans_pose.pose.orientation.x,
                               trans_pose.pose.orientation.y,
                               trans_pose.pose.orientation.z);
    Eigen::Vector3d t_world(trans_pose.pose.position.x,
                            trans_pose.pose.position.y,
                            trans_pose.pose.position.z);
    transform_world.block<3,3>(0,0)=q_world.toRotationMatrix();
    transform_world.block<3,1>(0,3)=t_world;
    pcl::transformPointCloud(*measurement.point_cloud,*measurement.point_cloud,transform_world);
    // 区域裁剪
    {
      pcl::transformPointCloud(*measurement.point_cloud,*measurement.point_cloud,tracking_info.target_data.transform_target.inverse());
      pcl::CropBox<pcl::PointXYZI> _box_region_;
      _box_region_.setMin(Eigen::Vector4f(-base_config.m_targetinfo.lower_width*0.6,
                                          -base_config.m_targetinfo.lower_length*0.55,
                                          -base_config.m_targetinfo.lower_height,0));
      _box_region_.setMax(Eigen::Vector4f(base_config.m_targetinfo.lower_width*0.6,
                                          base_config.m_targetinfo.lower_length*0.55,
                                          base_config.m_targetinfo.lower_height*1.2,0));
      _box_region_.setInputCloud(measurement.point_cloud);
      pcl::PointCloud<pcl::PointXYZI>::Ptr box_points(new pcl::PointCloud<pcl::PointXYZI>);
      _box_region_.filter(*box_points);
      pcl::transformPointCloud(*box_points,*box_points,tracking_info.target_data.transform_target);
      pcl::copyPointCloud(*box_points,*(measurement.point_cloud));
    }
    measurement.header.frame_id="odom";

    return true;
  }
  
  void pose_tracker::set_trajectory(tracking_trajectory & trajectory_data)
  {
    std::unique_lock<std::mutex> lock(cmd_mtx);
    tracking_info=trajectory_data;
    if(tracking_info.target_data.side_info=="NULL")
    {
      sensor_buffer.clear();
      lost_flag=0;
      ROS_INFO_STREAM("[PerceptionStorage][PoseTracker] TrajectoryReset");
    }
  }

  bool pose_tracker::get_trajectory(tracking_trajectory &trajectory_data)
  {
    std::unique_lock<std::mutex> lock(cmd_mtx);
    trajectory_data=tracking_info;
    
    return (tracking_info.target_data.side_info!="NULL")?true:false;
  }
  
  bool pose_tracker::get_measurement(std::deque<sensor_measurement> &sensor_data)
  {
    if(sensor_buffer.size()<frames_num)
    {
      return false;
    }
    else
    { 
      sensor_data=sensor_buffer;
      sensor_buffer.clear();
      return true;
    }
  }

  bool pose_tracker::tracking_pose_init(geometry_msgs::PoseStamped &reference_target,geometry_msgs::PoseStamped &pose_target)
  {
    tracking_info=tracking_trajectory();
    tracking_info=tracking_trajectory(tracking_target(reference_target,pose_target));
    lost_flag=0;
    {
      static tf::TransformBroadcaster br;
      tf::Transform                   transform;
      tf::Quaternion                  q;
      transform.setOrigin(tf::Vector3(pose_target.pose.position.x,
                                      pose_target.pose.position.y,
                                      pose_target.pose.position.z));
      q.setW(pose_target.pose.orientation.w);
      q.setX(pose_target.pose.orientation.x);
      q.setY(pose_target.pose.orientation.y);
      q.setZ(pose_target.pose.orientation.z);
      transform.setRotation( q );
      br.sendTransform( tf::StampedTransform( transform, pose_target.header.stamp,"odom","target_frame"));
    }
    return true;
  }

  bool pose_tracker::detection(tracking_trajectory &trajectory_data,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr &points_ptr,
                               std_msgs::Header &header,
                               geometry_msgs::Pose &robot_pose,
                               geometry_msgs::Pose &target_pose,
                               int &model_selected)
  {
    geometry_msgs::PoseStamped previous_pose;
    if(trajectory_data.state_array.size()>0)
    {
      previous_pose=trajectory_data.state_array.back().update_pose;
    }
    else
    {
      previous_pose=trajectory_data.target_data.target_pose;
    }
    // 刷新tf
    {
      static tf::TransformBroadcaster br;
      tf::Transform                   transform;
      tf::Quaternion                  q;
      transform.setOrigin(tf::Vector3(previous_pose.pose.position.x,
                                      previous_pose.pose.position.y,
                                      previous_pose.pose.position.z));
      q.setW(previous_pose.pose.orientation.w);
      q.setX(previous_pose.pose.orientation.x);
      q.setY(previous_pose.pose.orientation.y);
      q.setZ(previous_pose.pose.orientation.z);
      transform.setRotation( q );
      br.sendTransform( tf::StampedTransform(transform,header.stamp,"odom","target_frame"));
    }
    // 估算当前时刻相对于参考位置的位姿
    tf::Pose tf_previous_pose;
    tf::poseMsgToTF(trajectory_data.target_data.target_pose.pose,tf_previous_pose);
    tf::Pose tf_ego_pose;
    tf::poseMsgToTF(robot_pose,tf_ego_pose);
    tf::Pose tf_tmp_pose;
    tf_tmp_pose=tf_ego_pose.inverse()*tf_previous_pose;
    tf::Matrix3x3 tf_rotation(tf_tmp_pose.getRotation());
    double roll,pitch,yaw;
    tf_rotation.getRPY(roll,pitch,yaw);
    double dx=tf_tmp_pose.getOrigin().x();
    double dy=tf_tmp_pose.getOrigin().y();
    double dz=tf_tmp_pose.getOrigin().z();
    double y_distance=fabs(dy)-base_config.m_lidarinfo.distance_compensation;
    if(y_distance>0.6)
    {
      model_selected=1;
    }
    else if(y_distance<=0.6 && y_distance>=0.5)
    {
      model_selected=2;
    }
    else
    {
      model_selected=0;
      ROS_ERROR_STREAM("[PerceptionStorage][PoseTracker] y_distance "<<y_distance<<" model "<<model_selected);
      return false;
    }
    // 使用前序姿态做固定转换，对象坐标系
    Eigen::Quaternionf orientation(previous_pose.pose.orientation.w,
                                   previous_pose.pose.orientation.x,
                                   previous_pose.pose.orientation.y,
                                   previous_pose.pose.orientation.z);
    orientation.normalize();
    Eigen::Matrix4f pose_matrix(Eigen::Matrix4f::Identity());
    pose_matrix.block<3,3>(0,0) = orientation.toRotationMatrix();
    pose_matrix.block<3,1>(0,3)=Eigen::Vector3f(previous_pose.pose.position.x,
                                                previous_pose.pose.position.y, 
                                                previous_pose.pose.position.z);
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*points_ptr,*target_points,pose_matrix.inverse());
    pcl::PassThrough<pcl::PointXYZI> pass_through;
    pass_through.setInputCloud(target_points);
    pass_through.setFilterFieldName ("z");
    pass_through.setFilterLimits (-0.5*base_config.m_targetinfo.lower_height,base_config.m_targetinfo.lower_height);
    pass_through.filter(*target_points);
    pass_through.setInputCloud(target_points);
    pass_through.setFilterFieldName ("y");
    pass_through.setFilterLimits (-0.5*base_config.m_targetinfo.lower_length,0.5*base_config.m_targetinfo.lower_length);
    pass_through.filter(*target_points);
    pass_through.setInputCloud(target_points);
    pass_through.setFilterFieldName ("x");
    pass_through.setFilterLimits (-0.5*base_config.m_targetinfo.lower_width,0.5*base_config.m_targetinfo.lower_width);
    pass_through.filter(*target_points);
    pcl::CropBox<pcl::PointXYZI> crop_box;
    crop_box.setMin(Eigen::Vector4f(-0.1,
                                    -0.75*base_config.m_targetinfo.params[0],
                                    -0.5*base_config.m_targetinfo.params[1],0));
    crop_box.setMax(Eigen::Vector4f(0.1,
                                    0.75*base_config.m_targetinfo.params[0],
                                    0.25*base_config.m_targetinfo.params[1],0));
    crop_box.setInputCloud(target_points);
    pcl::PointCloud<pcl::PointXYZI>::Ptr plane_points(new pcl::PointCloud<pcl::PointXYZI>);
    crop_box.filter(*plane_points);
    if(plane_points->size()<10)
    {
      ROS_ERROR("[PerceptionStorage][PoseTracker] No_Plane");
      return false;
    }
    // 通道特征点
    pcl::PointCloud<pcl::PointXYZI>::Ptr tunnel_points(new pcl::PointCloud<pcl::PointXYZI>);
    pass_through.setInputCloud(target_points);
    pass_through.setFilterFieldName ("x");
    pass_through.setFilterLimits (-0.5*base_config.m_targetinfo.lower_width+0.05,0.5*base_config.m_targetinfo.lower_width-0.05);
    pass_through.filter(*tunnel_points);
    pass_through.setInputCloud(tunnel_points);
    pass_through.setFilterFieldName ("z");
    pass_through.setFilterLimits (0.5*base_config.m_targetinfo.params[1]-0.1,0.5*base_config.m_targetinfo.params[1]+0.1);
    pass_through.filter(*tunnel_points);
    pass_through.setInputCloud(tunnel_points);
    pass_through.setFilterFieldName ("y");
    if(model_selected==1)
    {
      pass_through.setFilterLimits (0.5*tunnel_width-0.03,0.5*tunnel_width+0.03);
      pass_through.filter(*tunnel_points);
    }
    else if(model_selected==2)
    {
      pass_through.setFilterLimits (-0.5*tunnel_width-tunnel_thickness-0.02,-0.5*tunnel_width-tunnel_thickness+0.02);
      pass_through.filter(*tunnel_points);
    }
    // 退化为无辅助线
    if(tunnel_points->size()<10) model_selected=3;
    // 数据区分
    for(auto &p_p:plane_points->points)
    {
      p_p.intensity=130;
    }
    for(auto &t_p:tunnel_points->points)
    {
      t_p.intensity=255;
    }
    // 基础滤波
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setMeanK(20);
    sor.setStddevMulThresh(1);
    sor.setInputCloud(plane_points);
    sor.filter(*plane_points);
    auto plane_plane_=filter_tools.naive_plane(plane_points,0.015);
    pcl::PointCloud<pcl::PointXYZI>::Ptr plane_plane_samples(new pcl::PointCloud<pcl::PointXYZI>);
    plane_plane_samples=std::get<0>(plane_plane_);
    if(plane_plane_samples->size()<10)
    {
      ROS_ERROR("[PerceptionStorage][PoseTracker] TargetPlane_Failed");
      return false;
    }
    pcl::copyPointCloud(*plane_plane_samples,*plane_points);
    if(model_selected<3)
    {
      sor.setInputCloud(tunnel_points);
      sor.filter(*tunnel_points);
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr plane_points_f(new pcl::PointCloud<pcl::PointXYZI>); 
    pcl::PointCloud<pcl::PointXYZI>::Ptr tunnel_points_f(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::ModelCoefficients::Ptr naive_feature(new pcl::ModelCoefficients());
    naive_feature->values.resize(4);
    naive_feature->values[0] = 0;
    naive_feature->values[1] = 0;
    naive_feature->values[2] = 1.0;
    naive_feature->values[3] = 0;
    pcl::ProjectInliers<pcl::PointXYZI> naive_projector;
    naive_projector.setModelType(pcl::SACMODEL_PLANE);
    naive_projector.setInputCloud(plane_points);
    naive_projector.setModelCoefficients(naive_feature);
    naive_projector.filter(*plane_points_f);
    if(model_selected<3)
    {
      naive_projector.setInputCloud(tunnel_points);
      naive_projector.setModelCoefficients(naive_feature);
      naive_projector.filter(*tunnel_points_f);
    }
    // 直线
    auto plane_line_=filter_tools.naive_line(plane_points_f,0.015);
    pcl::PointCloud<pcl::PointXYZI>::Ptr plane_line_samples(new pcl::PointCloud<pcl::PointXYZI>);
    plane_line_samples=std::get<0>(plane_line_);
    if(plane_line_samples->size()<5)
    {
      ROS_ERROR("[PerceptionStorage][PoseTracker] No_Line");
      return false;
    }
    Eigen::VectorXf plane_line_feature=std::get<2>(plane_line_);
    Eigen::Vector3f plane_line_direction={plane_line_feature[3],plane_line_feature[4],plane_line_feature[5]};
    Eigen::Vector3f plane_line_point={plane_line_feature[0],plane_line_feature[1],plane_line_feature[2]};
    pcl::PointCloud<pcl::PointXYZI>::Ptr tunnel_line_samples(new pcl::PointCloud<pcl::PointXYZI>);
    Eigen::Vector3f tunnel_line_direction,tunnel_line_point;
    if(model_selected<3)
    {
      auto tunnel_line_=filter_tools.naive_line(tunnel_points_f,0.015);
      tunnel_line_samples=std::get<0>(tunnel_line_);
      Eigen::VectorXf tunnel_line_feature=std::get<2>(tunnel_line_);
      tunnel_line_direction=Eigen::Vector3f(tunnel_line_feature[3],tunnel_line_feature[4],tunnel_line_feature[5]);
      tunnel_line_point=Eigen::Vector3f(tunnel_line_feature[0],tunnel_line_feature[1],tunnel_line_feature[2]);
      if(tunnel_line_samples->size()<5)
      {
        model_selected=3;
      }
    }
    // 转发处理的点云
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr source_plane_ptr(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr source_tunnel_ptr(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::copyPointCloud(*plane_line_samples,*source_plane_ptr);
      pcl::copyPointCloud(*tunnel_line_samples,*source_tunnel_ptr);
      *source_plane_ptr+=*source_tunnel_ptr;
      pcl::transformPointCloud(*source_plane_ptr,*source_plane_ptr,pose_matrix);
      source_plane_ptr->width=source_plane_ptr->size();
      source_plane_ptr->height=1;
      sensor_msgs::PointCloud2 target_msg;
      pcl::toROSMsg(*source_plane_ptr, target_msg);
      target_msg.height=source_plane_ptr->height;
      target_msg.width=source_plane_ptr->width;
      target_msg.is_dense=false;
      target_msg.header=header;
      pub_target_points.publish(target_msg);
    }
    //
    ROS_INFO_STREAM("[PerceptionStorage][PoseTracker] Model_Selected "<<model_selected);
    bool is_target_found=false;
    if(model_selected<3)
    {
      Eigen::Vector2f plane_line_norm(plane_line_direction[0],plane_line_direction[1]);
      if(plane_line_norm[1]<0) plane_line_norm*=-1.0;
      Eigen::Vector2f tunnel_line_norm(tunnel_line_direction[0],tunnel_line_direction[1]);
      if(tunnel_line_norm[1]<0) tunnel_line_norm*=-1.0;
      // 方向偏差一致性高
      if(fabs(plane_line_norm.dot(tunnel_line_norm))<cos(85.0/180.0*M_PI))
      {
        is_target_found=true;
      }
      else
      {
        double line_params[4] = {1,1,1,1};
        ceres::Problem problem;
        for(size_t i = 0; i< plane_line_samples->size(); ++i)
        {
          Eigen::Vector3d obi(plane_line_samples->points[i].x,
                              plane_line_samples->points[i].y,
                              plane_line_samples->points[i].z);
          ceres::CostFunction * costfunction =
                  new ceres::AutoDiffCostFunction<LineFunctor1,1,4>(
                          new LineFunctor1(obi.x(),obi.y()));
          ceres::LossFunction * loss_function = new ceres::CauchyLoss(0.05);
          problem.AddResidualBlock(costfunction, loss_function, line_params);
        }
        for(size_t i = 0; i< tunnel_line_samples->size(); ++i)
        {
          Eigen::Vector3d obi(tunnel_line_samples->points[i].x,
                              tunnel_line_samples->points[i].y,
                              tunnel_line_samples->points[i].z);
          ceres::CostFunction * costfunction =
                  new ceres::AutoDiffCostFunction<LineFunctor2,1,4>(
                          new LineFunctor2(obi.x(),obi.y()));
          ceres::LossFunction * loss_function = new ceres::CauchyLoss(0.05);
          problem.AddResidualBlock(costfunction, loss_function, line_params);
        }
        ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<PerpendicularConstraint, 1, 4>(new PerpendicularConstraint);
        problem.AddResidualBlock(cost_function, nullptr, line_params);
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 50;
        ceres::Solver::Summary summary;
        ceres::Solve (options, &problem, & summary);
        Eigen::Vector3d line1_info_ = {line_params[0],line_params[1],0};
        Eigen::Vector2d line1_info=line1_info_.head(2);
        line1_info.normalize();
        if(line1_info[0]<0) line1_info*=-1.0;
        Eigen::Vector3d line2_info_ = {line_params[2],line_params[3],0};
        Eigen::Vector2d line2_info=line2_info_.head(2);
        line2_info.normalize();
        if(line2_info[0]<0) line2_info*=-1.0;
        ROS_INFO_STREAM("[PerceptionStorage][PoseTracker] angle "<<acos(fabs(plane_line_norm.dot(tunnel_line_norm)))*180.0/M_PI
                                                            <<" "<<acos(fabs(line1_info.dot(line2_info)))/M_PI*180.0);  
        if(fabs(line1_info.dot(line2_info))<cos(85.0/180.0*M_PI))
        {
          is_target_found=true;
        }
      }
    }
    else if(model_selected==3)
    {
      for(auto &p_line_p:plane_line_samples->points)
      {
        Eigen::Vector3f point_data(p_line_p.x, p_line_p.y, p_line_p.z);
        Eigen::Vector3f v = point_data - plane_line_point;
        float distance = v.dot(plane_line_direction);
        Eigen::Vector3f projected_point = plane_line_point + distance * plane_line_direction;
        p_line_p.x=projected_point[0];
        p_line_p.y=projected_point[1];
        p_line_p.z=projected_point[2];
      }
      pcl::PointXYZI pmin,pmax;
      auto target_length=pcl::getMaxSegment(*plane_line_samples,pmin,pmax);
      ROS_INFO_STREAM("[PerceptionStorage][PoseTracker] Target_Length "<<target_length);
      if(fabs(base_config.m_targetinfo.params[0]-target_length)<=0.03)
      {
        is_target_found=true;
      }
    }
    if(!is_target_found)
    {
      ROS_ERROR("[PerceptionStorage][PoseTracker] No_Target");
      return false;
    }
    // 姿态
    auto _norm_center_=filter_tools.target_orient(TargetOrient::SVD,plane_points);
    auto _norm_=std::get<0>(_norm_center_);
    auto _center_=std::get<1>(_norm_center_);
    Eigen::Vector3f refer(1,0,0);
    if(_norm_(0)<0)
    {
      _norm_*=-1.0;
    }
    Eigen::Matrix4f _target_pose_matrix_=tool_box::pose_norm_refer(_norm_,_center_,refer);
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr model_points_ptr(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::transformPointCloud(*plane_points,*model_points_ptr,_target_pose_matrix_.inverse());
      for(auto &p_p:model_points_ptr->points)
      {
        p_p.x=0;
      }
      model_points_ptr->height=1;
      model_points_ptr->width=model_points_ptr->points.size();
      Eigen::Vector4f model_min,model_max;
      pcl::getMinMax3D(*model_points_ptr,model_min,model_max);
      ROS_INFO_STREAM("[PerceptionStorage][PoseTracker] PlaneFeature "<<"X "<<model_min[0]<<" "<<model_max[0]
                                                                      <<" Y "<<model_min[1]<<" "<<model_max[1]
                                                                      <<" L "<<(model_max[1]-model_min[1])
                                                                      <<" Z "<<model_min[2]<<" "<<model_max[2]);
      if(fabs(model_max[1]-model_min[1]-base_config.m_targetinfo.params[0])<=0.015)
      {
        Eigen::Vector4f model_center((model_min[0]+model_max[0])/2.0,(model_min[1]+model_max[1])/2.0,(model_min[2]+model_max[2])/2.0,1);
        model_center=_target_pose_matrix_*model_center;
        _center_=model_center.head(3);
      }
      else
      {
        ROS_ERROR_STREAM("[PerceptionStorage][PoseTracker] Length_Error "<<(model_max[1]-model_min[1]));
        return false;
      }
    }
    Eigen::Vector3f x_v=_target_pose_matrix_.col(0).head(3);
    Eigen::Vector3f z_v(0,0,1);
    Eigen::Vector3f y_v=z_v.cross(x_v);
    z_v=x_v.cross(y_v);
    Eigen::Matrix3f r_matrix;
    r_matrix.col(0)=x_v;
    r_matrix.col(1)=y_v;
    r_matrix.col(2)=z_v;
    geometry_msgs::Pose detection_pose;
    Eigen::Quaternionf quat(r_matrix);
    quat.normalize();
    detection_pose.orientation.w=quat.w();
    detection_pose.orientation.x=quat.x();
    detection_pose.orientation.y=quat.y();
    detection_pose.orientation.z=quat.z();
    if(base_config.m_lidarinfo.distance_compensation!=0)
    {
      _center_=_center_-base_config.m_lidarinfo.distance_compensation*x_v;
    }
    detection_pose.position.x=_center_[0];
    detection_pose.position.y=_center_[1];
    detection_pose.position.z=_center_[2];
    geometry_msgs::Pose trans_pose;
    tf::Pose tf_detection_pose,tf_trans_pose,tf_tmp;
    tf::poseMsgToTF(detection_pose,tf_detection_pose);
    tf::poseMsgToTF(previous_pose.pose,tf_tmp);
    tf_trans_pose=tf_tmp*tf_detection_pose;
    tf::poseTFToMsg(tf_trans_pose,trans_pose);
    // 返回姿态数据
    target_pose=trans_pose;

    return true;
  }

  void pose_tracker::tracking_pose(tracking_trajectory &trajectory_data,std::deque<sensor_measurement> &process_measurement)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_points(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<geometry_msgs::PoseStamped> ego_motion;
    pcl::PassThrough<pcl::PointXYZI> pass_filter;
    auto process_stamp=process_measurement.back().header.stamp;
    while(!process_measurement.empty())
    {
      auto one_frame=process_measurement.back();
      trans_points_world(one_frame);
      if(using_local_map)
      {
        if(raw_points->size()==0)
        {
          pcl::copyPointCloud(*(one_frame.point_cloud),*raw_points);
        }
        else
        {
          auto T_start=ros::Time().now();
          nano_gicp::NanoGICP<pcl::PointXYZI,pcl::PointXYZI> mgicp;
          mgicp.setMaxCorrespondenceDistance(0.5);
          mgicp.setNumThreads(0);
          mgicp.setCorrespondenceRandomness(15);
          mgicp.setMaximumIterations(35);
          mgicp.setTransformationEpsilon(0.01);
          mgicp.setEuclideanFitnessEpsilon(0.01);
          mgicp.setRANSACIterations(5);
          mgicp.setRANSACOutlierRejectionThreshold(1.0);
          mgicp.setInputTarget(raw_points);
          mgicp.calculateTargetCovariances();
          mgicp.setInputSource(one_frame.point_cloud);
          mgicp.calculateSourceCovariances();
          pcl::PointCloud<pcl::PointXYZI>::Ptr _aligned_ (new pcl::PointCloud<pcl::PointXYZI>);
          mgicp.align(*_aligned_);
          if(mgicp.hasConverged() && mgicp.getFitnessScore() < 1.0)
          {
            Eigen::Matrix4d trans_add=Eigen::Matrix4d::Identity();
            trans_add=mgicp.getFinalTransformation().cast<double>();
            *raw_points+=*_aligned_;
          }
          auto T_end=ros::Time().now();
          ego_motion.emplace_back(process_measurement.back().kinematic_pose);
        }
      }
      else
      {
        *raw_points+=*(one_frame.point_cloud);
        ego_motion.emplace_back(process_measurement.back().kinematic_pose);
      }
      process_measurement.pop_back();
    }
    raw_points->height=1;
    raw_points->width=raw_points->points.size();
    // 转发观测数据
    sensor_msgs::PointCloud2 raw_msg;
    pcl::toROSMsg(*raw_points, raw_msg);
    raw_msg.height=raw_points->height;
    raw_msg.width=raw_points->width;
    raw_msg.is_dense=false;
    raw_msg.header.stamp=process_stamp;
    raw_msg.header.frame_id="odom";
    pub_points.publish(raw_msg);
    // detection
    geometry_msgs::Pose robot_pose=ego_motion.front().pose;
    geometry_msgs::Pose target_pose;
    int model_flag=1;
    if(detection(trajectory_data,raw_points,ego_motion.front().header,robot_pose,target_pose,model_flag))
    {
      tf::Pose tf_target_pose;
      tf::poseMsgToTF(target_pose,tf_target_pose);
      tf::Pose tf_ego_pose;
      tf::poseMsgToTF(robot_pose,tf_ego_pose);
      tf::Pose tf_tmp_pose;
      tf_tmp_pose=tf_ego_pose.inverse()*tf_target_pose;
      tf_tmp_pose=tf_tmp_pose.inverse();
      tf::Matrix3x3 tf_rotation(tf_tmp_pose.getRotation());
      double roll,pitch,yaw;
      tf_rotation.getRPY(roll,pitch,yaw);
      double dx=tf_tmp_pose.getOrigin().x();
      double dy=tf_tmp_pose.getOrigin().y();
      double dz=tf_tmp_pose.getOrigin().z();
      ROS_INFO_STREAM("[PerceptionStorage][PoseTracker] RobotInTarget_Pose "<<" x "<<dx<<" y "<<dy<<" z "<<dz
                                                                            <<" yaw "<<yaw*180.0/M_PI
                                                                            <<" pitch "<<pitch*180.0/M_PI
                                                                            <<" roll "<<roll*180.0/M_PI);
      // 变化量
      bool back_ward=false;
      bool kinematic_valid=true;
      if(trajectory_data.state_array.size()>1)
      {
        auto last_inverse_pose=trajectory_data.state_array[trajectory_data.state_array.size()-1].inverse_pose;
        double dx_last=last_inverse_pose.getOrigin().x();
        double residual_x=dx_last-dx;
        ROS_INFO_STREAM("[PerceptionStorage][PoseTracker] residual_x "<<residual_x);
        if(residual_x<0)
        {
          back_ward=true;
          ROS_INFO_STREAM("[PerceptionStorage][PoseTracker] BackWard");
        }
        // 运动一致性对比,只考虑x
        auto tf_delat_target=last_inverse_pose.inverse()*tf_tmp_pose;
        tf::Pose tf_last_kinematic_pose;
        tf::poseMsgToTF(trajectory_data.state_array[trajectory_data.state_array.size()-1].kinematic_pose.pose,tf_last_kinematic_pose);
        auto tf_delata_world=tf_last_kinematic_pose.inverse()*tf_ego_pose;
        Eigen::Vector2d trans_world(tf_delata_world.getOrigin().x(),tf_delata_world.getOrigin().y());
        Eigen::Vector2d trans_target(tf_delat_target.getOrigin().x(),tf_delat_target.getOrigin().y());
        tf::Matrix3x3 _rotation_world(tf_delata_world.getRotation());
        double _roll_world, _pitch_world, _yaw_world;
        _rotation_world.getRPY(_roll_world, _pitch_world, _yaw_world);
        tf::Matrix3x3 _rotation_target(tf_delat_target.getRotation());
        double _roll_target, _pitch_target, _yaw_target;
        _rotation_target.getRPY(_roll_target, _pitch_target, _yaw_target);
        ROS_INFO_STREAM("[PerceptionStorage][PoseTracker] motion_consistency "<<trans_world[0]<<" "
                                                                              <<trans_target[0]<<" "
                                                                              <<_yaw_world*180.0/M_PI<<" "
                                                                              <<_yaw_target*180.0/M_PI);
        if(fabs((trans_world-trans_target)[0])>0.02 || (_yaw_target*_yaw_world)<0)
        {
          kinematic_valid=false;
        }
      }
      if(kinematic_valid)
      {
        geometry_msgs::PoseStamped target_pose_stamped;
        target_pose_stamped.pose=target_pose;
        target_pose_stamped.header.stamp=process_stamp;
        target_pose_stamped.header.frame_id="odom";
        trajectory_data.state_array.emplace_back(tracking_state(ego_motion.back(),target_pose_stamped,tf_tmp_pose,true));
        pub_pose.publish(target_pose_stamped);
        lost_flag=0;
        geometry_msgs::PoseStamped robot_pose_stamped;
        geometry_msgs::Pose _robot_pose_;
        tf::poseTFToMsg(tf_tmp_pose,_robot_pose_);
        robot_pose_stamped.pose=_robot_pose_;
        robot_pose_stamped.header.stamp=process_stamp;
        robot_pose_stamped.header.frame_id="target_frame";
        pub_pose_robot.publish(robot_pose_stamped);
        {
          static tf::TransformBroadcaster br;
          tf::Transform                   transform;
          tf::Quaternion                  q;
          transform.setOrigin(tf::Vector3(target_pose.position.x,
                                          target_pose.position.y,
                                          target_pose.position.z));
          q.setW(target_pose.orientation.w);
          q.setX(target_pose.orientation.x);
          q.setY(target_pose.orientation.y);
          q.setZ(target_pose.orientation.z);
          transform.setRotation( q );
          br.sendTransform( tf::StampedTransform(transform,process_stamp,"odom","target_frame"));
        }
      }
      float _dist_=fabs(dx)-base_config.m_lidarinfo.distance_compensation;
      if(_dist_<stop_distance || back_ward)
      {
        trajectory_data=tracking_trajectory();
        ROS_INFO_STREAM("[PerceptionStorage][PoseTracker] ResetState "<<" range "<< _dist_
                                                                      <<" backward "<<back_ward);
        std::string tracker_state="Reset_Trajectory Motion "+std::to_string(_dist_)+" "+std::to_string(back_ward);
        std_msgs::String msg;
        msg.data=tracker_state;
        pub_tracker_state.publish(msg);
      }
      set_trajectory(trajectory_data);
    }
    else
    {
      lost_flag++;
      if(model_flag==0)
      {
        lost_flag+=2;
      }
    }
    if(lost_flag>2)
    {
      ROS_INFO_STREAM("[PerceptionStorage][PoseTracker] LostTracker "<< lost_flag);
      trajectory_data=tracking_trajectory();
      set_trajectory(trajectory_data);
      std::string tracker_state="Reset_Trajectory Lost "+std::to_string(lost_flag);
      std_msgs::String msg;
      msg.data=tracker_state;
      pub_tracker_state.publish(msg);
    }
  }

  void pose_tracker::track()
  {
    while(ros::ok())
    {
      tracking_trajectory trajectory_data;
      if(get_trajectory(trajectory_data))
      {
        std::deque<sensor_measurement> process_measurement;
        if(get_measurement(process_measurement))
        {
          tracking_pose(trajectory_data,process_measurement);
        }
      }
      ros::Duration(0.1).sleep();
    }
  }
}
