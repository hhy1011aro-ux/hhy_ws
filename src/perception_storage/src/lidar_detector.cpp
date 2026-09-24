#include "perception_storage/detector.hpp"

#include <cmath>

namespace
{
// 在已排序的一维投影坐标中做线性插值，避免分位点只能落在离散采样点上。
float quantileFromSorted(const std::vector<float> &values,float quantile)
{
  const float position=quantile*static_cast<float>(values.size()-1);
  const size_t lower_index=static_cast<size_t>(std::floor(position));
  const size_t upper_index=static_cast<size_t>(std::ceil(position));
  const float weight=position-static_cast<float>(lower_index);
  return values[lower_index]*(1.0f-weight)+values[upper_index]*weight;
}

// 使用优化后的标签直线重新筛选完整候选点，并以主方向Q2/Q98端点中点估计标签中心。
// 两端各裁去2%的极端点，可以降低离群点和RANSAC内点子集变化对中心位置的影响。
bool computeQuantileLineCenter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &candidate_points,
                               const Eigen::VectorXf &line_coefficients,
                               float distance_threshold,
                               Eigen::Vector4f &center,
                               size_t &selected_count,
                               float &robust_span)
{
  selected_count=0;
  robust_span=0.0f;
  if(!candidate_points || candidate_points->empty() || line_coefficients.size()!=6)
  {
    return false;
  }

  const Eigen::Vector3f line_origin=line_coefficients.head<3>();
  Eigen::Vector3f line_direction=line_coefficients.segment<3>(3);
  const float direction_norm=line_direction.norm();
  if(!std::isfinite(direction_norm) || direction_norm<1e-6f)
  {
    return false;
  }
  line_direction/=direction_norm;

  // 只保留到优化直线距离不超过门限的候选，再转成沿直线方向的一维坐标。
  std::vector<float> longitudinal_coordinates;
  longitudinal_coordinates.reserve(candidate_points->size());
  for(const auto &point:candidate_points->points)
  {
    if(!pcl::isFinite(point))
    {
      continue;
    }
    const Eigen::Vector3f point_vector(point.x,point.y,point.z);
    const Eigen::Vector3f relative=point_vector-line_origin;
    const float perpendicular_distance=relative.cross(line_direction).norm();
    if(perpendicular_distance<=distance_threshold)
    {
      longitudinal_coordinates.emplace_back(relative.dot(line_direction));
    }
  }

  selected_count=longitudinal_coordinates.size();
  if(selected_count<4)
  {
    return false;
  }

  // 对称分位端点的中点与方向正负无关，因此无需额外统一标签线方向。
  std::sort(longitudinal_coordinates.begin(),longitudinal_coordinates.end());
  const float q02=quantileFromSorted(longitudinal_coordinates,0.02f);
  const float q98=quantileFromSorted(longitudinal_coordinates,0.98f);
  robust_span=q98-q02;
  if(!std::isfinite(q02) || !std::isfinite(q98) || robust_span<=0.0f)
  {
    return false;
  }

  center.head<3>()=line_origin+0.5f*(q02+q98)*line_direction;
  center(3)=1.0f;
  return center.allFinite();
}
}

lidar_detector::lidar_detector(ConfigInfo &config_)
{
  // 发布精调原始点和投影点 hhy add. 2026.09.24
  ros::NodeHandle nh;
  pub_refine_raw_points=nh.advertise<sensor_msgs::PointCloud2>("/perception/debug/refine/raw_points",1);
  pub_refine_projected_points=nh.advertise<sensor_msgs::PointCloud2>("/perception/debug/refine/projected_points",1);

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

bool lidar_detector::inference(depth_mapping &data,std_msgs::Header &header_info)
{
  // 调试点云与当前检测输入使用同一Header，便于按时间戳关联日志和结果。
  refine_debug_header=header_info;
  interm_result.second.clear();
  interm_result.first=cv::Mat();
  detect_targets.clear();
  detect_result.clear();
  set_roi(header_info);
  auto hyp_info=segmentor_machine->segmentation(data);
  auto points_ptr=data.points;
  auto mapping_vt=data.mapping;
  auto cues_ptr=data.cues_points;
  // 统计每个2D候选实际采样和成功映射的像素数量，定位空patch原因。
  size_t hypothesis_index=0;
  for(auto &hyp:hyp_info.second)
  {
    int rows=hyp.img_rows;
    int cols=hyp.img_cols;
    pcl::PointCloud<pcl::PointXYZI>::Ptr slice_points(new pcl::PointCloud<pcl::PointXYZI>);
    size_t sampled_pixel_count=0;
    size_t mapped_pixel_count=0;
    for(auto &pp:hyp.edge_points)
    {
      auto left_p=pp.first;
      auto right_p=pp.second;
      int index_start=left_p.y*cols+left_p.x;
      int index_end=right_p.y*cols+right_p.x;
      for(int i=index_start;i<=index_end;i++)
      {
        sampled_pixel_count++;
        if(mapping_vt[i]>0)
        {
          mapped_pixel_count++;
          slice_points->points.emplace_back(points_ptr->at(mapping_vt[i]));
        }
      }
    }
    // 输出候选级映射摘要，重点识别“有边界但采样数为0”的四点顺序问题。
    ROS_INFO_STREAM("[PerceptionStorage][LidarDetector] HypothesisMapping "
                    <<"Stamp "<<header_info.stamp.toSec()
                    <<" Seq "<<header_info.seq
                    <<" Index "<<hypothesis_index
                    <<" EdgePairs "<<hyp.edge_points.size()
                    <<" SampledPixels "<<sampled_pixel_count
                    <<" MappedPixels "<<mapped_pixel_count
                    <<" PatchPoints "<<slice_points->size());
    if(slice_points->size()>0)
    {
      interm_result.second.emplace_back(intermediate(slice_points));
      // 有效区域可视化
      auto vpoints=hyp.four_points;
      cv::line(hyp_info.first,vpoints[0],vpoints[3],cv::Scalar(255,0,255),2,cv::LINE_AA,0);
      cv::line(hyp_info.first,vpoints[1],vpoints[2],cv::Scalar(0,255,255),2,cv::LINE_AA,0);
    }
    hypothesis_index++;
  }
  interm_result.first=hyp_info.first;
  // 按帧累计几何过滤、精调和最终结果数量，形成检测流水线阶段摘要。
  size_t geometry_pass_count=0;
  size_t refine_pass_count=0;
  size_t patch_index=0;
  for(auto & patch:interm_result.second)
  {
    if(geomery_filter(patch.source_points))
    {
      geometry_pass_count++;
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
      if(refine_filter(detect_targets.back(),cues_ptr,refine_target))
      {
        detect_result.emplace_back(refine_target);
        refine_pass_count++;
      }
    }
    else
    {
      ROS_INFO_STREAM("[PerceptionStorage][LidarDetector] GeomeryFilter_Out "
                      <<"Patch "<<patch_index
                      <<" Points "<<patch.source_points->size());
    }
    patch_index++;
  }
  // 该日志用于将无结果帧定位到候选、映射、几何过滤或精调阶段。
  ROS_INFO_STREAM("[PerceptionStorage][LidarDetector] InferenceSummary "
                  <<"Stamp "<<header_info.stamp.toSec()
                  <<" Seq "<<header_info.seq
                  <<" Frame "<<header_info.frame_id
                  <<" DepthHypotheses "<<hyp_info.second.size()
                  <<" MappedPatches "<<interm_result.second.size()
                  <<" GeometryPass "<<geometry_pass_count
                  <<" RefinePass "<<refine_pass_count
                  <<" Results "<<detect_result.size()
                  <<" MidPixel "<<data.mid_pixel[0]<<" "<<data.mid_pixel[1]);
}

bool lidar_detector::geomery_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr & patch)
{
  // 保留过滤前点数，失败时同时输出平面点数和包围盒尺寸。
  size_t source_points_size=patch->size();
  auto _plane_=filter_machine->naive_plane(patch,0.015);
  auto _plane_points_=std::get<0>(_plane_);
  if(_plane_points_->points.size()>0)
  {
    pcl::MomentOfInertiaEstimation <pcl::PointXYZI> moment_extractor;
    moment_extractor.setInputCloud(_plane_points_);
    moment_extractor.compute();
    std::vector<float> moment_of_inertia;
    std::vector<float> eccentricity;
    pcl::PointXYZI min_point_AABB,max_point_AABB,min_point_OBB,max_point_OBB,position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector,mass_center;
    moment_extractor.getMomentOfInertia(moment_of_inertia);
    moment_extractor.getEccentricity(eccentricity);
    moment_extractor.getAABB(min_point_AABB,max_point_AABB);
    moment_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    moment_extractor.getEigenValues(major_value, middle_value, minor_value);
    moment_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
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
      // 记录实际跨度及上下限，区分目标过大、过小和点云不足。
      ROS_INFO_STREAM("[PerceptionStorage][LidarDetector] GeometrySizeReject "
                      <<"SourcePoints "<<source_points_size
                      <<" PlanePoints "<<_plane_points_->size()
                      <<" BoxWidth "<<box_width
                      <<" BoxHeight "<<box_height
                      <<" BoxDepth "<<box_depth
                      <<" Span "<<std::max(box_width,box_height)
                      <<" Min "<<targetinfo.params[0]*0.7
                      <<" Max "<<targetinfo.params[0]*1.414);
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
    // 单独标记平面拟合为空，避免与包围盒尺寸失败混在同一日志中。
    ROS_INFO_STREAM("[PerceptionStorage][LidarDetector] GeometryPlaneReject "
                    <<"SourcePoints "<<source_points_size
                    <<" PlanePoints "<<_plane_points_->size());
    return false;
  }

}

bool lidar_detector::refine_filter(mpose &target,pcl::PointCloud<pcl::PointXYZI>::Ptr &cues,mpose &result)
{
  // 将不同几何类别合并到一个调试点云，intensity 0/1/2分别表示标签、长边和短边。
  auto append_labeled_points=[](const pcl::PointCloud<pcl::PointXYZI>::Ptr &source,
                                float intensity,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr &destination)
  {
    if(!source) return;
    for(auto point:source->points)
    {
      point.intensity=intensity;
      destination->points.emplace_back(point);
    }
  };
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
  // 仅有订阅者时组装调试消息，避免正常运行时产生不必要的点云复制开销。
  if(pub_refine_raw_points.getNumSubscribers()>0)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_debug_points(new pcl::PointCloud<pcl::PointXYZI>);
    append_labeled_points(target.target_points,0.0,raw_debug_points);
    append_labeled_points(large_side,1.0,raw_debug_points);
    append_labeled_points(short_side,2.0,raw_debug_points);
    raw_debug_points->width=raw_debug_points->points.size();
    raw_debug_points->height=1;
    raw_debug_points->is_dense=false;
    sensor_msgs::PointCloud2 raw_debug_msg;
    pcl::toROSMsg(*raw_debug_points,raw_debug_msg);
    raw_debug_msg.header=refine_debug_header;
    pub_refine_raw_points.publish(raw_debug_msg);
  }
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
    ROS_INFO_STREAM("[PerceptionStorage][LidarDetector] LargeSide_Points "<<fliter_large->points.size());
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
  // 发布XOY投影后的三类点，用于直接观察参与各条直线拟合的二维点带。
  if(pub_refine_projected_points.getNumSubscribers()>0)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr projected_debug_points(new pcl::PointCloud<pcl::PointXYZI>);
    append_labeled_points(tag_projected,0.0,projected_debug_points);
    append_labeled_points(large_projected,1.0,projected_debug_points);
    append_labeled_points(short_projected,2.0,projected_debug_points);
    projected_debug_points->width=projected_debug_points->points.size();
    projected_debug_points->height=1;
    projected_debug_points->is_dense=false;
    sensor_msgs::PointCloud2 projected_debug_msg;
    pcl::toROSMsg(*projected_debug_points,projected_debug_msg);
    projected_debug_msg.header=refine_debug_header;
    pub_refine_projected_points.publish(projected_debug_msg);
  }
  // 框架投影线段
  // 长边方向直接决定最终yaw，使用全部RANSAC内点重估系数以降低随机样本方向噪声。
  auto large_line_=filter_machine->naive_line(large_projected,0.0075,true);
  if(std::get<1>(large_line_).size()<3)
  {
    ROS_INFO_STREAM("[PerceptionStorage][LidarDetector] LargeSideLine_Points "<<std::get<1>(large_line_).size());
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
  // 标签线的优化系数用于重新筛选完整候选点和计算Q2/Q98稳健中心。
  auto tag_line_=filter_machine->naive_line(tag_projected,0.0075,true);
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
    ROS_INFO_STREAM("[PerceptionStorage][LidarDetector] SupportScore "<<flag_num);
    return false;
  }
  // 优化
  mpose refine_pose;
  // 同时传入完整标签候选和RANSAC内点：前者计算稳健中心，后者保留原长度及距离判据。
  if(finetuning_filter(short_projected_coef_,short_projected_line,short_direction_,
                      large_projected_coef_,large_projected_line,large_direction_,
                      tag_projected_coef_,tag_projected,tag_projected_line,tag_direction_,
                      flag_num,target,refine_pose))
  {
    result=refine_pose;
    return true;
  }
  else
  {
    ROS_INFO_STREAM("[PerceptionStorage][LidarDetector] FinetuningFilter_Out");
    return false;
  }
}

bool lidar_detector::finetuning_filter(Eigen::VectorXf &short_coef_,pcl::PointCloud<pcl::PointXYZI>::Ptr &short_line,Eigen::Vector4f &short_direction,
                                       Eigen::VectorXf &large_coef_,pcl::PointCloud<pcl::PointXYZI>::Ptr &large_line,Eigen::Vector4f &large_direction,
                                       Eigen::VectorXf &tag_coef_,pcl::PointCloud<pcl::PointXYZI>::Ptr &tag_candidates,
                                       pcl::PointCloud<pcl::PointXYZI>::Ptr &tag_line,Eigen::Vector4f &tag_direction,
                                       int &flag_index,mpose &base_pose,mpose &refine_pose)
{ 
  int fine_state=0;
  // 大致计算目标长度
  pcl::PointXYZI pmin,pmax;
  auto tag_length=pcl::getMaxSegment(*tag_line,pmin,pmax);
  // 输出实测长度和误差，便于确认精调是在长度判据前还是判据后失败。
  ROS_INFO_STREAM("[PerceptionStorage][LidarDetector] TagLength "
                  <<"Measured "<<tag_length
                  <<" Expected "<<targetinfo.params[0]
                  <<" Error "<<fabs(tag_length-targetinfo.params[0]));
  // 与真实值差异太大
  if(fabs(tag_length-targetinfo.params[0])>0.03)
  {
    ROS_INFO_STREAM("[PerceptionStorage][LidarDetector] DeltaTargetLength "<<fabs(tag_length-targetinfo.params[0]));
    return false;
  }
  Eigen::Vector4f tag_center;
  size_t tag_center_point_count=0;
  float tag_center_robust_span=0.0f;
  // 优先采用完整候选点的Q2/Q98端点中点，消除RANSAC内点质心随子集切换而跳变的问题。
  if(computeQuantileLineCenter(tag_candidates,tag_coef_,0.0075f,tag_center,
                               tag_center_point_count,tag_center_robust_span))
  {
    ROS_INFO_STREAM("[PerceptionStorage][LidarDetector] TagCenterQ2Q98 "
                    <<"Candidates "<<tag_candidates->size()
                    <<" Selected "<<tag_center_point_count
                    <<" RobustSpan "<<tag_center_robust_span
                    <<" Center "<<tag_center(0)<<" "<<tag_center(1)<<" "<<tag_center(2));
  }
  else
  {
    // 输入或模型异常时回退到原内点质心，避免中心优化本身造成整帧无结果。
    pcl::compute3DCentroid(*tag_line,tag_center);
    ROS_WARN_STREAM("[PerceptionStorage][LidarDetector] TagCenterQ2Q98Fallback "
                    <<"Candidates "<<(tag_candidates ? tag_candidates->size():0)
                    <<" Selected "<<tag_center_point_count
                    <<" Coefficients "<<tag_coef_.size());
  }
  Eigen::Vector3f refer(1,0,0);
  int confidence=0;
  bool large_condition=false;
  bool short_condition=false;
  // 保存各门限的中间状态和测量值，供FineConditionSummary统一输出，不改变原置信度计算。
  bool large_angle_condition=false;
  bool large_distance_condition=false;
  bool short_angle_condition=false;
  bool short_distance_condition=false;
  float theta_large=-1.0;
  float mean_large_distance=-1.0;
  float theta_short=-1.0;
  float mean_short_distance=-1.0;
  {
    // 角度
    auto _theta_L=fabs(pcl::rad2deg(fabs(pcl::normAngle(pcl::getAngle3D(large_direction,tag_direction)))));
    if(_theta_L>90.0)
    {
      _theta_L=180.0-_theta_L;
    }
    theta_large=_theta_L;
    float large_heading=pcl::rad2deg(atan2(large_direction(1),large_direction(0)));
    float tag_heading=pcl::rad2deg(atan2(tag_direction(1),tag_direction(0)));
    // 同时记录长边、标签线方向和内点数，用于关联yaw噪声与内点集合变化。
    ROS_INFO_STREAM("[PerceptionStorage][LidarDetector] DirectionTrace "
                    <<"LargeHeading "<<large_heading
                    <<" TagHeading "<<tag_heading
                    <<" RelativeAngle "<<theta_large
                    <<" LargeInliers "<<large_line->size()
                    <<" TagInliers "<<tag_line->size());
    // 距离
    auto dist_large_info=tool_box::dist_points_line(tag_line,large_coef_);
    auto max_t2l=std::get<0>(dist_large_info);
    auto min_t2l=std::get<1>(dist_large_info);
    auto mean_t2l=std::get<2>(dist_large_info);
    mean_large_distance=mean_t2l;
    auto dist_large =std::sqrt(pcl::sqrPointToLineDistance(tag_center,
                                 Eigen::Vector4f(large_coef_(0),large_coef_(1),large_coef_(2),0),
                                 Eigen::Vector4f(large_coef_(3),large_coef_(4),large_coef_(5),0)));
    ROS_INFO_STREAM("[PerceptionStorage][LidarDetector] Target2LargeSide "<<"Angle "<<_theta_L<<" Dist "<<mean_t2l<<" "<<dist_large);
    large_angle_condition=_theta_L<5.0;
    if(large_angle_condition)
    {
      confidence+=2;
      large_distance_condition=fabs(mean_t2l-0.5*targetinfo.lower_width)<0.05;
      if(large_distance_condition)
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
      theta_short=_theta_S;
      mean_short_distance=mean_t2s;
      auto dist_short =std::sqrt(pcl::sqrPointToLineDistance(tag_center,
                                 Eigen::Vector4f(short_coef_(0),short_coef_(1),short_coef_(2),0),
                                 Eigen::Vector4f(short_coef_(3),short_coef_(4),short_coef_(5),0)));
      ROS_INFO_STREAM("[PerceptionStorage][LidarDetector] Target2ShortSide "<<"Angle "<<_theta_S<<" Dist "<<mean_t2s<<" "<<dist_short);
      short_angle_condition=fabs(_theta_S-90.0)<5.0;
      if(short_angle_condition)
      {
        confidence++;
        short_distance_condition=fabs(mean_t2s-0.5*free_distance)<0.05;
        if(short_distance_condition)
        {
          confidence++;
          short_condition=true;
        }
      }
    }
    // 汇总所有精调门限的原始量、判定结果和置信度，直接定位结果被拒绝的条件。
    ROS_INFO_STREAM("[PerceptionStorage][LidarDetector] FineConditionSummary "
                    <<"LargeAngle "<<theta_large
                    <<" LargeAngleOK "<<large_angle_condition
                    <<" LargeDistance "<<mean_large_distance
                    <<" LargeDistanceExpected "<<0.5*targetinfo.lower_width
                    <<" LargeDistanceOK "<<large_distance_condition
                    <<" LargeCondition "<<large_condition
                    <<" ShortAvailable "<<(flag_index>2)
                    <<" ShortAngle "<<theta_short
                    <<" ShortAngleOK "<<short_angle_condition
                    <<" ShortDistance "<<mean_short_distance
                    <<" ShortDistanceExpected "<<0.5*free_distance
                    <<" ShortDistanceOK "<<short_distance_condition
                    <<" ShortCondition "<<short_condition
                    <<" Confidence "<<confidence);
    if(confidence<3)
    {
      ROS_INFO_STREAM("[PerceptionStorage][LidarDetector] Confidence "<<confidence);
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
        // 验证最终输出yaw与长边拟合方向的固定几何关系，排除旋转矩阵构造引入跳变。
        ROS_INFO_STREAM("[PerceptionStorage][LidarDetector] OutputDirection "
                        <<"OutputYaw "<<refined_angles(0)
                        <<" LargeHeading "<<large_heading);
        if(fabs(refined_angles(0))<60.0)
        {
          ROS_INFO_STREAM("[PerceptionStorage][LidarDetector] BadDirection");
          return false;
        }
        fine_matrix.topLeftCorner<3, 3>() =r_matrix;
        ROS_INFO_STREAM("[PerceptionStorage][LidarDetector] RefineOrientation_ByLargeSide");
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
              ROS_INFO_STREAM("[PerceptionStorage][LidarDetector] RefinePosition_ByShortIntersectionLarge");
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

void lidar_detector::set_roi(std_msgs::Header &header_info)
{
  visualization_msgs::Marker cube_region;
  cube_region.header=header_info;
  cube_region.action = visualization_msgs::Marker::ADD;
  cube_region.type = visualization_msgs::Marker::LINE_LIST;
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
    geometry_msgs::Point p_start,p_end;
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
