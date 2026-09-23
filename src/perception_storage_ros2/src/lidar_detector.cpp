#include "perception_storage_ros2/detector.hpp"

lidar_detector::lidar_detector(ConfigInfo &config_)
{
  lidarinfo=config_.m_lidarinfo;
  targetinfo=config_.m_targetinfo;
  settinginfo=config_.m_settinginfo;
  depth_resolution=0.0075;
  free_distance=0.495;
  float length=floor(targetinfo.params[0]/depth_resolution);
  float angle=45.0;
  bool loosely=false;
  segmentor_machine=std::make_shared<depth_segmentor>(length,angle,loosely);
  filter_machine=std::make_shared<points_filter>();
  // roi
  float x_min=-1.2*targetinfo.lower_length;
  float x_max=0.2*targetinfo.lower_length;
  float y_min=-2.5*targetinfo.lower_width;
  float y_max=-0.1;
  float z_min=-targetinfo.lower_height;
  float z_max=1.2*targetinfo.lower_height;
  Eigen::Vector3f vertex_p1(x_max,y_min,z_max);
  Eigen::Vector3f vertex_p2(x_min,y_min,z_max);
  Eigen::Vector3f vertex_p3(x_min,y_min,z_min);
  Eigen::Vector3f vertex_p4(x_max,y_min,z_min);
  Eigen::Vector3f vertex_p5(x_max,y_max,z_max);
  Eigen::Vector3f vertex_p6(x_min,y_max,z_max);
  Eigen::Vector3f vertex_p7(x_min,y_max,z_min);
  Eigen::Vector3f vertex_p8(x_max,y_max,z_min);
  region_vertex={vertex_p1,vertex_p2,
                 vertex_p2,vertex_p3,
                 vertex_p3,vertex_p4,
                 vertex_p4,vertex_p1,
                 vertex_p1,vertex_p5,
                 vertex_p2,vertex_p6,
                 vertex_p3,vertex_p7,
                 vertex_p4,vertex_p8,
                 vertex_p5,vertex_p6,
                 vertex_p6,vertex_p7,
                 vertex_p7,vertex_p8,
                 vertex_p8,vertex_p5};
}

lidar_detector::~lidar_detector()
{

}

bool lidar_detector::inference(depth_mapping &data,std_msgs::msg::Header &header_info)
{
  interm_result.second.clear();
  interm_result.first=cv::Mat();
  detect_targets.clear();
  detect_result.clear();
  set_roi(header_info);
  // ELSED 线段提取
  auto hyp_info=segmentor_machine->segmentation(data);
  auto points_ptr=data.points;
  auto mapping_vt=data.mapping;
  auto cues_ptr=data.cues_points;
  for(auto &hyp:hyp_info.second)
  {
    int rows=hyp.img_rows;
    int cols=hyp.img_cols;
    // 逐行遍历左右边界间的所有像素，通过 `mapping_table` 找对应三维点，将有效点组成一个候选 patch
    pcl::PointCloud<pcl::PointXYZI>::Ptr slice_points(new pcl::PointCloud<pcl::PointXYZI>);
    for(auto &pp:hyp.edge_points)
    {
      auto left_p=pp.first;
      auto right_p=pp.second;
      int index_start=left_p.y*cols+left_p.x;
      int index_end=right_p.y*cols+right_p.x;
      for(int i=index_start;i<=index_end;i++)
      {
        if(mapping_vt[i]>0)
        {
          slice_points->points.emplace_back(points_ptr->at(mapping_vt[i]));
        }
      }
    }
    if(slice_points->size()>0)
    {
      interm_result.second.emplace_back(intermediate(slice_points));
      // 有效区域可视化
      auto vpoints=hyp.four_points;
      cv::line(hyp_info.first,vpoints[0],vpoints[3],cv::Scalar(255,0,255),2,cv::LINE_AA,0);
      cv::line(hyp_info.first,vpoints[1],vpoints[2],cv::Scalar(0,255,255),2,cv::LINE_AA,0);
    }
  }
  interm_result.first=hyp_info.first;
  for(auto & patch:interm_result.second)
  {
    // 平面和尺寸初筛，RANSAC 平面拟合，计算三维 OBB，须位于 [0.147, 0.297]
    if(geomery_filter(patch.source_points))
    {
      // 通过后，用平面点的 SVD 最小奇异向量作为法向，点均值作为中心。若法向 y 分量为负则翻转。
      // 以该法向、全局 z 轴构造初始正交姿态，生成 `detect_targets` 中的候选位姿。
      auto _norm_center_=filter_machine->target_orient(TargetOrient::SVD,patch.source_points);
      auto _norm_=std::get<0>(_norm_center_);
      auto _center_=std::get<1>(_norm_center_);
      Eigen::Vector3f refer(1,0,0);
      if (_norm_(1)<0)
      {
        _norm_*=-1.0;
      }
      Eigen::Matrix4f pose_matrix=tool_box::pose_norm_refer(_norm_,_center_,refer);
      Eigen::Vector3f x_v=pose_matrix.col(0).head(3);
      Eigen::Vector3f z_v(0,0,1);
      Eigen::Vector3f y_v=z_v.cross(x_v);
      z_v=x_v.cross(y_v);
      Eigen::Matrix3f r_matrix;
      r_matrix.col(0)=x_v;r_matrix.col(1)=y_v;r_matrix.col(2)=z_v;
      pose_matrix.topLeftCorner<3, 3>() =r_matrix;
      detect_targets.emplace_back(mpose(pose_matrix,_norm_,patch.source_points));
      mpose refine_target;
      // 从完整线索点云中搜索货架边框
      if(refine_filter(detect_targets.back(),cues_ptr,refine_target))
      {
        detect_result.emplace_back(refine_target);
      }
    }
    else
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("perception_storage"), "[PerceptionStorage][LidarDetector] GeomeryFilter_Out");
    }
  }
  return true;
}

bool lidar_detector::geomery_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr & patch)
{
  auto _plane_=filter_machine->naive_plane(patch,0.015);
  auto _plane_points_=std::get<0>(_plane_);
  if(_plane_points_->points.size()>0)
  {
    // 计算候选点云的包围盒
    pcl::MomentOfInertiaEstimation <pcl::PointXYZI> moment_extractor;
    moment_extractor.setInputCloud(_plane_points_);
    moment_extractor.compute();
    std::vector<float> moment_of_inertia;
    std::vector<float> eccentricity;
    pcl::PointXYZI min_point_AABB,max_point_AABB,min_point_OBB,max_point_OBB,position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector,mass_center;
    // 惯性矩特征
    moment_extractor.getMomentOfInertia(moment_of_inertia);
    // 偏心率用于描述点云沿不同方向的形状差异
    moment_extractor.getEccentricity(eccentricity);
    // 原始点云坐标系下的轴对齐包围盒
    moment_extractor.getAABB(min_point_AABB,max_point_AABB);
    // OBB坐标系
    moment_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    // 三个主方向对应的特征值和特征向量
    moment_extractor.getEigenValues(major_value, middle_value, minor_value);
    moment_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
    // 点云质心
    moment_extractor.getMassCenter(mass_center);
    Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
    Eigen::Quaternionf quat(rotational_matrix_OBB);
    Eigen::Vector3f euler_angles = rotational_matrix_OBB.eulerAngles(1,0,2);
    float box_width = max_point_OBB.x - min_point_OBB.x;
    float box_height = max_point_OBB.y - min_point_OBB.y;
    float box_depth = max_point_OBB.z - min_point_OBB.z;
    if(std::max(box_width,box_height)>targetinfo.params[0]*1.414
    || std::max(box_width,box_height)<targetinfo.params[0]*0.7)
    {
      return false;
    }
    else
    {
      pcl::copyPointCloud<pcl::PointXYZI>(*_plane_points_,*patch);
      return true;
    }
  }
  else
  {
    return false;
  }

}

bool lidar_detector::refine_filter(mpose &target,pcl::PointCloud<pcl::PointXYZI>::Ptr &cues,mpose &result)
{
  pcl::PassThrough<pcl::PointXYZI> pass;
  // 辅助板点集
  pcl::PointCloud<pcl::PointXYZI>::Ptr face_tag(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud<pcl::PointXYZI>(*target.target_points,*face_tag);
  // 过滤掉与货架底部接触的点，噪声影响大
  pcl::transformPointCloud(*face_tag,*face_tag,target.transform.inverse());
  pcl::PointXYZI _tag_min_,_tag_max_;
  pcl::getMinMax3D(*face_tag,_tag_min_,_tag_max_);
  float z_up_limit =0.5*_tag_max_.z;
  pass.setInputCloud(face_tag);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits(_tag_min_.z,z_up_limit);
  pass.filter(*face_tag);
  pcl::transformPointCloud(*face_tag,*face_tag,target.transform);
  // 特征点转化到目标坐标系处理
  pcl::PointCloud<pcl::PointXYZI>::Ptr face_cues(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*cues,*face_cues,target.transform.inverse());
  pcl::PointCloud<pcl::PointXYZI>::Ptr large_side(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr short_side(new pcl::PointCloud<pcl::PointXYZI>);
  pass.setInputCloud(face_cues);
  // 长边框
  pass.setFilterFieldName ("x");
  pass.setFilterLimits(0.5*targetinfo.lower_width-0.05,0.5*targetinfo.lower_width+0.05);
  pass.filter(*large_side);
  pass.setInputCloud(large_side);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits(-0.3*targetinfo.lower_length,0.3*targetinfo.lower_length);
  pass.filter(*large_side);
  // 远端导向结构
  pass.setInputCloud(face_cues);
  pass.setFilterFieldName ("y");
  float disturbance=0.02;
  pass.setFilterLimits(0.5*free_distance-disturbance,0.5*free_distance+disturbance);
  pass.filter(*short_side);
  pass.setInputCloud(short_side);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits(-0.25*targetinfo.lower_width,0.75*targetinfo.lower_width);
  pass.filter(*short_side);
  // 转回雷达系
  pcl::transformPointCloud(*large_side,*large_side,target.transform);
  pcl::transformPointCloud(*short_side,*short_side,target.transform);
  // large side
  std::map<int,pcl::PointCloud<pcl::PointXYZI>> select_large;
  for(auto &raw_p:large_side->points)
  {
    auto theta=pcl::rad2deg(atan2(raw_p.y,raw_p.x));
    int theta_bin=floor(theta/0.2); 
    if(select_large.find(theta_bin)!=select_large.end())
    {
      select_large[theta_bin].points.emplace_back(raw_p);
    }
    else
    { 
      pcl::PointCloud<pcl::PointXYZI> temple_pc;
      temple_pc.points.emplace_back(raw_p);
      select_large.insert(std::make_pair(theta_bin,temple_pc));
    } 
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr fliter_large(new pcl::PointCloud<pcl::PointXYZI>);
  for(auto & theta_x:select_large)
  {
    std::vector<float> dist_arrray;
    float min_dist=std::numeric_limits<float>::max();
    float max_dist=std::numeric_limits<float>::min();
    for(auto & x_p:theta_x.second.points)
    {
      auto _dist_=pow(x_p.x*x_p.x+x_p.y*x_p.y,0.5);
      dist_arrray.emplace_back(_dist_);
      min_dist=(_dist_<min_dist)?_dist_:min_dist;
      max_dist=(_dist_>max_dist)?_dist_:max_dist;
    }
    float dist_range=min_dist+std::min(0.025,(max_dist-min_dist)*0.1);
    for(int i=0;i<dist_arrray.size();i++)
    {
      if(dist_arrray[i]<=dist_range)
      {
        fliter_large->points.emplace_back(theta_x.second.points[i]);
      }
    }
  }
  if(fliter_large->points.size()<3) 
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("perception_storage"), "[PerceptionStorage][LidarDetector] LargeSide_Points "<<fliter_large->points.size());
    return false;
  }
  // short side
  auto short_plane_=filter_machine->naive_plane(short_side,0.015);
  auto short_plane_points_=std::get<0>(short_plane_);
  std::map<int,pcl::PointCloud<pcl::PointXYZI>> select_short;
  for(auto &raw_p:short_plane_points_->points)
  {
    auto theta=pcl::rad2deg(atan2(raw_p.y,raw_p.x));
    int theta_bin=floor(theta/0.03); 
    if(select_short.find(theta_bin)!=select_short.end())
    {
      select_short[theta_bin].points.emplace_back(raw_p);
    }
    else
    { 
      pcl::PointCloud<pcl::PointXYZI> temple_pc;
      temple_pc.points.emplace_back(raw_p);
      select_short.insert(std::make_pair(theta_bin,temple_pc));
    } 
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr fliter_short(new pcl::PointCloud<pcl::PointXYZI>);
  for(auto & theta_x:select_short)
  {
    std::vector<float> dist_arrray;
    float min_dist=std::numeric_limits<float>::max();
    float max_dist=std::numeric_limits<float>::min();
    for(auto & x_p:theta_x.second.points)
    {
      auto _dist_=pow(x_p.x*x_p.x+x_p.y*x_p.y,0.5);
      dist_arrray.emplace_back(_dist_);
      min_dist=(_dist_<min_dist)?_dist_:min_dist;
      max_dist=(_dist_>max_dist)?_dist_:max_dist;
    }
    float dist_range=min_dist+std::min(0.02,(max_dist-min_dist)*0.1);
    for(int i=0;i<dist_arrray.size();i++)
    {
      if(dist_arrray[i]<=dist_range)
      {
        fliter_short->points.emplace_back(theta_x.second.points[i]);
      }
    }
  }
  bool using_short=false;
  if(fliter_short->points.size()>3) using_short=true;
  // 投影到XOY平面
  pcl::ModelCoefficients::Ptr zero_feature(new pcl::ModelCoefficients());
  zero_feature->values.resize(4);
  zero_feature->values[0] = 0;
  zero_feature->values[1] = 0;
  zero_feature->values[2] = 1.0;
  zero_feature->values[3] = 0;
  pcl::ProjectInliers<pcl::PointXYZI> cues_projector;
  cues_projector.setModelType(pcl::SACMODEL_PLANE);
  // large投影
  pcl::PointCloud<pcl::PointXYZI>::Ptr large_projected(new pcl::PointCloud<pcl::PointXYZI>);
  cues_projector.setInputCloud(fliter_large);
  cues_projector.setModelCoefficients(zero_feature);
  cues_projector.filter(*large_projected);
  // short投影
  pcl::PointCloud<pcl::PointXYZI>::Ptr short_projected(new pcl::PointCloud<pcl::PointXYZI>);
  if(using_short)
  {
    cues_projector.setInputCloud(fliter_short);
    cues_projector.setModelCoefficients(zero_feature);
    cues_projector.filter(*short_projected);
  }
  // tagt投影
  pcl::PointCloud<pcl::PointXYZI>::Ptr tag_projected(new pcl::PointCloud<pcl::PointXYZI>);
  cues_projector.setInputCloud(face_tag);
  cues_projector.setModelCoefficients(zero_feature);
  cues_projector.filter(*tag_projected);
  // 框架投影线段
  auto large_line_=filter_machine->naive_line(large_projected,0.0075);
  if(std::get<1>(large_line_).size()<3)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("perception_storage"), "[PerceptionStorage][LidarDetector] LargeSideLine_Points "<<std::get<1>(large_line_).size());
    return false;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr large_projected_line(new pcl::PointCloud<pcl::PointXYZI>);
  large_projected_line=std::get<0>(large_line_);
  pcl::PointCloud<pcl::PointXYZI>::Ptr large_line_raw(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud<pcl::PointXYZI>(*fliter_large,std::get<1>(large_line_),*large_line_raw);
  Eigen::VectorXf large_projected_coef_=std::get<2>(large_line_);
  Eigen::Vector4f large_direction_={large_projected_coef_[3],large_projected_coef_[4],large_projected_coef_[5],0};
  if(large_direction_[0]<0) large_direction_*=-1.0;
  // short线段
  auto short_line_=filter_machine->naive_line(short_projected,0.0075);
  pcl::PointCloud<pcl::PointXYZI>::Ptr short_projected_line(new pcl::PointCloud<pcl::PointXYZI>);
  Eigen::VectorXf short_projected_coef_;
  Eigen::Vector4f short_direction_;
  if(std::get<1>(short_line_).size()>3)
  {
    short_projected_line=std::get<0>(short_line_);
    pcl::PointCloud<pcl::PointXYZI>::Ptr short_line_raw(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud<pcl::PointXYZI>(*fliter_short,std::get<1>(short_line_),*short_line_raw);
    short_projected_coef_=std::get<2>(short_line_);
    short_direction_={short_projected_coef_[3],short_projected_coef_[4],short_projected_coef_[5],0};
    if(short_direction_[1]<0) short_direction_*=-1.0;
  }
  // tag线段
  auto tag_line_=filter_machine->naive_line(tag_projected,0.0075);
  pcl::PointCloud<pcl::PointXYZI>::Ptr tag_projected_line(new pcl::PointCloud<pcl::PointXYZI>);
  tag_projected_line=std::get<0>(tag_line_);
  pcl::PointCloud<pcl::PointXYZI>::Ptr tag_line_raw(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud<pcl::PointXYZI>(*face_tag,std::get<1>(tag_line_),*tag_line_raw);
  Eigen::VectorXf tag_projected_coef_=std::get<2>(tag_line_);
  Eigen::Vector4f tag_direction_={tag_projected_coef_[3],tag_projected_coef_[4],tag_projected_coef_[5],0};
  if(tag_direction_[0]<0) tag_direction_*=-1.0;
  // 条件
  int c1=0,c2=0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cues_evidence(new pcl::PointCloud<pcl::PointXYZI>);
  if(large_projected_line->size()>3)
  {
    *cues_evidence+=*large_projected_line;
    c1=1;
  }
  if(short_projected_line->size()>3)
  {
    *cues_evidence+=*short_projected_line;
    c2=1;
  }
  if(tag_projected_line->size()>3)
  {
    *cues_evidence+=*tag_projected_line;
  }
  int flag_num=c1*pow(2,1)+c2*pow(2,0);
  if(flag_num<2)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("perception_storage"), "[PerceptionStorage][LidarDetector] SupportScore "<<flag_num);
    return false;
  }
  // 优化
  mpose refine_pose;
  if(finetuning_filter(short_projected_coef_,short_projected_line,short_direction_,
                      large_projected_coef_,large_projected_line,large_direction_,
                      tag_projected_coef_,tag_projected_line,tag_direction_,
                      flag_num,target,refine_pose))
  {
    result=refine_pose;
    return true;
  }
  else
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("perception_storage"), "[PerceptionStorage][LidarDetector] FinetuningFilter_Out");
    return false;
  }
}

bool lidar_detector::finetuning_filter(Eigen::VectorXf &short_coef_,pcl::PointCloud<pcl::PointXYZI>::Ptr &short_line,Eigen::Vector4f &short_direction,
                                       Eigen::VectorXf &large_coef_,pcl::PointCloud<pcl::PointXYZI>::Ptr &large_line,Eigen::Vector4f &large_direction,
                                       Eigen::VectorXf &tag_coef_,pcl::PointCloud<pcl::PointXYZI>::Ptr &tag_line,Eigen::Vector4f &tag_direction,
                                       int &flag_index,mpose &base_pose,mpose &refine_pose)
{ 
  int fine_state=0;
  // 大致计算目标长度
  pcl::PointXYZI pmin,pmax;
  auto tag_length=pcl::getMaxSegment(*tag_line,pmin,pmax);
  // 与真实值差异太大
  if(fabs(tag_length-targetinfo.params[0])>0.03)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("perception_storage"), "[PerceptionStorage][LidarDetector] DeltaTargetLength "<<fabs(tag_length-targetinfo.params[0]));
    return false;
  }
  Eigen::Vector4f tag_center;
  pcl::compute3DCentroid(*tag_line, tag_center);
  Eigen::Vector3f refer(1,0,0);
  int confidence=0;
  bool large_condition=false;
  bool short_condition=false;
  {
    // 角度
    auto _theta_L=fabs(pcl::rad2deg(fabs(pcl::normAngle(pcl::getAngle3D(large_direction,tag_direction)))));
    if(_theta_L>90.0)
    {
      _theta_L=180.0-_theta_L;
    }
    // 距离
    auto dist_large_info=tool_box::dist_points_line(tag_line,large_coef_);
    auto max_t2l=std::get<0>(dist_large_info);
    auto min_t2l=std::get<1>(dist_large_info);
    auto mean_t2l=std::get<2>(dist_large_info);
    auto dist_large =std::sqrt(pcl::sqrPointToLineDistance(tag_center,
                                 Eigen::Vector4f(large_coef_(0),large_coef_(1),large_coef_(2),0),
                                 Eigen::Vector4f(large_coef_(3),large_coef_(4),large_coef_(5),0)));
    RCLCPP_INFO_STREAM(rclcpp::get_logger("perception_storage"), "[PerceptionStorage][LidarDetector] Target2LargeSide "<<"Angle "<<_theta_L<<" Dist "<<mean_t2l<<" "<<dist_large);
    if(_theta_L<5.0)
    {
      confidence+=2;
      if(fabs(mean_t2l-0.5*targetinfo.lower_width)<0.05)
      {
        confidence+=2;
        large_condition=true;
      }
    }
    // 短边
    if(flag_index>2)
    {
      auto _theta_S=fabs(pcl::rad2deg(fabs(pcl::normAngle(pcl::getAngle3D(short_direction,large_direction)))));
      // 距离
      auto dist_short_info=tool_box::dist_points_line(tag_line,short_coef_);
      auto max_t2s=std::get<0>(dist_short_info);
      auto min_t2s=std::get<1>(dist_short_info);
      auto mean_t2s=std::get<2>(dist_short_info);
      auto dist_short =std::sqrt(pcl::sqrPointToLineDistance(tag_center,
                                 Eigen::Vector4f(short_coef_(0),short_coef_(1),short_coef_(2),0),
                                 Eigen::Vector4f(short_coef_(3),short_coef_(4),short_coef_(5),0)));
      RCLCPP_INFO_STREAM(rclcpp::get_logger("perception_storage"), "[PerceptionStorage][LidarDetector] Target2ShortSide "<<"Angle "<<_theta_S<<" Dist "<<mean_t2s<<" "<<dist_short);
      if(fabs(_theta_S-90.0)<5.0)
      {
        confidence++;
        if(fabs(mean_t2s-0.5*free_distance)<0.05)
        {
          confidence++;
          short_condition=true;
        }
      }
    }
    if(confidence<3)
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("perception_storage"), "[PerceptionStorage][LidarDetector] Confidence "<<confidence);
      return false;
    }
    else
    {
      // 调整输出,没有足够辅助线索的判定无效
      if(large_condition)
      {
        // 调整朝向
        Eigen::Matrix4f fine_matrix(Eigen::Matrix4f::Identity());
        Eigen::Vector3f y_v=-large_direction.head(3);
        Eigen::Vector3f z_v(0,0,1);
        Eigen::Vector3f x_v=y_v.cross(z_v);
        z_v=x_v.cross(y_v);
        Eigen::Matrix3f r_matrix;
        r_matrix.col(0)=x_v;r_matrix.col(1)=y_v;r_matrix.col(2)=z_v;
        Eigen::Vector3f refined_angles = (r_matrix.eulerAngles(2,1,0))*180.0/M_PI;
        if(fabs(refined_angles(0))<60.0)
        {
          RCLCPP_INFO_STREAM(rclcpp::get_logger("perception_storage"), "[PerceptionStorage][LidarDetector] BadDirection");
          return false;
        }
        fine_matrix.topLeftCorner<3, 3>() =r_matrix;
        RCLCPP_INFO_STREAM(rclcpp::get_logger("perception_storage"), "[PerceptionStorage][LidarDetector] RefineOrientation_ByLargeSide");
        Eigen::Vector3f tag_center_compensation=Eigen::Vector3f(tag_center(0),tag_center(1),tag_center(2))-lidarinfo.distance_compensation*x_v;
        fine_matrix.block<3,1>(0,3)=tag_center_compensation;
        pcl::PointCloud<pcl::PointXYZI>::Ptr fine_points(new pcl::PointCloud<pcl::PointXYZI>);
        *fine_points+=*large_line;
        *fine_points+=*tag_line;
        if(short_condition)
        {
          Eigen::Vector4f _intersection_point_;
          bool flag_intersection=pcl::lineWithLineIntersection(short_coef_,large_coef_,_intersection_point_,1e-4);
          if(flag_intersection)
          {
            Eigen::Vector3f ap=_intersection_point_.head(3)-Eigen::Vector3f(tag_center(0),tag_center(1),tag_center(2));
            double t=ap.dot(y_v)/y_v.dot(y_v);
            Eigen::Vector3f project_intersection=Eigen::Vector3f(tag_center(0),tag_center(1),tag_center(2))+t*y_v;
            auto _gap_=(project_intersection-Eigen::Vector3f(tag_center(0),tag_center(1),tag_center(2))).norm();
            if(fabs(_gap_-0.5*free_distance)<=0.01)
            {
              float move_dist=_gap_-0.5*free_distance;
              // 减小修正量
              Eigen::Vector3f move_vector=0.75*move_dist*y_v;
              Eigen::Vector3f new_tag_center=Eigen::Vector3f(tag_center(0),tag_center(1),tag_center(2))+move_vector;
              new_tag_center=new_tag_center-lidarinfo.distance_compensation*x_v;
              fine_matrix.block<3,1>(0,3)=new_tag_center;
              *fine_points+=*short_line;
              RCLCPP_INFO_STREAM(rclcpp::get_logger("perception_storage"), "[PerceptionStorage][LidarDetector] RefinePosition_ByShortIntersectionLarge");
            }
          }
        }
        refine_pose=mpose(fine_matrix,x_v,fine_points);
        refine_pose.confident_index=confidence;
        return true;
      }
      else
      {
        return false;
      }
    }
  }
}

std::pair<cv::Mat,std::vector<intermediate>> lidar_detector::get_intermediate()
{
  return interm_result;
}

std::vector<mpose> lidar_detector::get_targets()
{
  return detect_targets;
}

std::vector<mpose> lidar_detector::get_result()
{
  return detect_result;
}

void lidar_detector::set_roi(std_msgs::msg::Header &header_info)
{
  visualization_msgs::msg::Marker cube_region;
  cube_region.header=header_info;
  cube_region.action = visualization_msgs::msg::Marker::ADD;
  cube_region.type = visualization_msgs::msg::Marker::LINE_LIST;
  cube_region.ns = "cube_region";
  cube_region.id = 1;
  cube_region.pose.orientation.w = 1;
  cube_region.scale.x = 0.05;
  cube_region.scale.y = 0.05;
  cube_region.scale.z = 0.05;
  cube_region.color.r = 0.9;
  cube_region.color.g = 0.9;
  cube_region.color.b = 0;
  cube_region.color.a = 1;
  for(int i=0;i<12;i++)
  {
    auto start_point=region_vertex[i*2];
    auto end_point=region_vertex[i*2+1];
    geometry_msgs::msg::Point p_start,p_end;
    p_start.x=start_point(0);
    p_start.y=start_point(1);
    p_start.z=start_point(2);
    p_end.x=end_point(0);
    p_end.y=end_point(1);
    p_end.z=end_point(2);
    cube_region.points.push_back(p_start);
    cube_region.points.push_back(p_end);
  }
  roi_region.markers.push_back(cube_region);
}
