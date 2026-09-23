#include "perception_storage/detector.hpp"

tof_detector::tof_detector(ConfigInfo &config_)
{
  lidarinfo=config_.m_lidarinfo;
  targetinfo=config_.m_targetinfo;
  settinginfo=config_.m_settinginfo;
  // 4.5mm
  depth_resolution=0.0045;
  float length=floor(targetinfo.params[0]/depth_resolution);
  float angle=45;
  bool loosely=false;
  segmentor_machine=std::make_shared<depth_segmentor>(length,angle,loosely);
  filter_machine=std::make_shared<points_filter>();
  // roi
  float x_min=-0.75*targetinfo.lower_length;
  float x_max=0.75*targetinfo.lower_length;
  float z_min=0.1;
  float z_max=2.5*targetinfo.lower_width;
  float y_min=-targetinfo.lower_height;
  float y_max=0.5*targetinfo.lower_height+0.4*targetinfo.upper_height;
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

tof_detector::~tof_detector()
{

}

bool tof_detector::inference(depth_mapping &data,std_msgs::Header &header_info)
{
  ROS_INFO_STREAM("[PerceptionStorage][TofDetector] Inference_Start");
  interm_result.second.clear();
  interm_result.first=cv::Mat();
  detect_targets.clear();
  detect_result.clear();
  auto hyp_info=segmentor_machine->segmentation(data);
  ROS_INFO_STREAM("[PerceptionStorage][TofDetector] DepthProposals "<<hyp_info.second.size());
  auto points_ptr=data.points;
  auto mapping_vt=data.mapping;
  int mpoint_type=1;
  set_roi(header_info);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cues_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::CropBox<pcl::PointXYZI> pcl_box_filter;
  pcl_box_filter.setMin(Eigen::Vector4f(region_vertex[3][0],region_vertex[3][1],region_vertex[3][2],1.0));
  pcl_box_filter.setMax(Eigen::Vector4f(region_vertex[9][0],region_vertex[9][1],region_vertex[9][2],1.0));
  pcl_box_filter.setInputCloud(points_ptr);
  pcl_box_filter.filter(*cues_ptr);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud(cues_ptr);
  sor.setMeanK(20);
  sor.setStddevMulThresh(1);
  sor.filter(*cues_ptr);
  // 解析深度图检测结果
  for(auto &hyp:hyp_info.second)
  {
    int rows=hyp.img_rows;
    int cols=hyp.img_cols;
    // 利用区域中心计算分辨率
    auto vpoints=hyp.four_points;
    int region_cx=round((vpoints[0].x+vpoints[1].x+vpoints[2].x+vpoints[3].x)/4.0);
    int region_cy=round((vpoints[0].y+vpoints[1].y+vpoints[2].y+vpoints[3].y)/4.0);
    int region_center_index=region_cy*cols+region_cx;
    int valid_k=0,valid_k_=0;
    float vres=0,vres_=0;
    for(int k=-2;k<2;k++)
    {
      // 垂直
      int refer_k=region_center_index+k*cols;
      int refer_kp=region_center_index+(k+1)*cols;
      if(mapping_vt[refer_k]>0 && mapping_vt[refer_kp]>0)
      {
        auto p_k=points_ptr->at(mapping_vt[refer_k]);
        auto p_kp=points_ptr->at(mapping_vt[refer_kp]);
        float res_k=0;
        res_k=(Eigen::Vector3f(p_k.x-p_kp.x,p_k.y-p_kp.y,p_k.z-p_kp.z)).norm();
        if(res_k>0.001 && res_k<0.05)
        {
          vres+=res_k;
          valid_k++;
        }
      }
      // 水平
      int refer_k_=region_center_index+k;
      int refer_kp_=region_center_index+(k+1);
      if(mapping_vt[refer_k_]>0 && mapping_vt[refer_kp_]>0)
      {
        auto p_k_=points_ptr->at(mapping_vt[refer_k_]);
        auto p_kp_=points_ptr->at(mapping_vt[refer_kp_]);
        float res_k_=0;
        res_k_=(Eigen::Vector3f(p_k_.x-p_kp_.x,p_k_.y-p_kp_.y,p_k_.z-p_kp_.z)).norm();
        if(res_k_>0.001 && res_k_<0.05)
        {
          vres_+=res_k_;
          valid_k_++;
        }
      }
    }
    if(valid_k>0) 
    {
      vres=vres/float(valid_k);
    }
    else
    {
      vres=0.01;
    }
    int threshold_region_rows=round(targetinfo.params[1]/vres*0.5);
    if(valid_k_>0) 
    {
      vres_=vres_/float(valid_k_);
    }
    else
    {
      vres_=0.01;
    }
    int threshold_region_cols=round(targetinfo.params[0]/vres_*0.5);
    // 有效行数太少
    if(hyp.edge_points.size()<threshold_region_rows)
    {
      continue;
    }
    std::map<int,std::vector<mpoint>> target_lines;
    int min_index=rows;
    int max_index=-1;
    for(auto &pp:hyp.edge_points)
    {
      auto left_p=pp.first;
      auto right_p=pp.second;
      // 本行数据点
      if((right_p.x-left_p.x+1)<threshold_region_cols || (right_p.x-left_p.x+1)>4.0*threshold_region_cols)
      {
        continue;
      }
      // padding
      int padding_left=((left_p.x-3)<0)?0:(left_p.x-3);
      int padding_right=((right_p.x+3)>=cols)?(cols-1):(right_p.x+3);
      int index_start=left_p.y*cols+padding_left;
      int index_end=right_p.y*cols+padding_right;
      pcl::PointCloud<pcl::PointXYZI>::Ptr slice_line(new pcl::PointCloud<pcl::PointXYZI>);
      for(int i=index_start;i<=index_end;i++)
      {
        if(mapping_vt[i]>0)
        {
          slice_line->points.emplace_back(points_ptr->at(mapping_vt[i]));
        }
      }
      if(slice_line->size()<threshold_region_cols)
      {
        continue;
      }
      // 线性分析
      pcl::SampleConsensusModelLine<pcl::PointXYZI>::Ptr line_model(new pcl::SampleConsensusModelLine<pcl::PointXYZI>(slice_line));	
	    pcl::RandomSampleConsensus<pcl::PointXYZI> line_fit(line_model);	
      line_fit.setDistanceThreshold(0.015);
	    line_fit.setMaxIterations(100);
	    line_fit.computeModel();
      std::vector<int> _line_inliers_;
	    line_fit.getInliers(_line_inliers_);
      // 有效点太少
      if(_line_inliers_.size()<threshold_region_cols*0.5 || float(_line_inliers_.size())/float(slice_line->size())<0.5)
      {
        continue;
      }
      Eigen::VectorXf _line_coef_;
      line_fit.getModelCoefficients(_line_coef_);
      Eigen::Vector3f _line_direction_={_line_coef_[3],_line_coef_[4],_line_coef_[5]};
      Eigen::Vector4f line_dir_4f={_line_coef_[3],_line_coef_[4],_line_coef_[5],0};
      Eigen::Vector4f line_pt_4f={_line_coef_[0],_line_coef_[1],_line_coef_[2],0};
      float dot = _line_direction_.normalized().dot(Eigen::Vector3f::UnitZ());
      // 与正方向夹角太大
      if(std::abs(dot) > std::cos(45.0 * M_PI / 180.0)) continue;
      auto valid_length=pcl::euclideanDistance(slice_line->at(_line_inliers_.front()),slice_line->at(_line_inliers_.back()));
      // 长度偏差大
      if(valid_length<0.5*targetinfo.params[0] || valid_length>1.5*targetinfo.params[0])
      {
        continue;
      }
      // 对候选线段左右延伸，测试归属大平面的可能性
      auto last_left_point=slice_line->at(_line_inliers_.front());
      auto last_right_point=slice_line->at(_line_inliers_.back());
      int last_left_index=index_start+_line_inliers_.front();
      int last_right_index=index_start+_line_inliers_.back();
      int left_col=std::max(last_left_index%cols-1,0);
      int right_col=std::min(last_right_index%cols+1,cols-1);
      // 本行符合条件的最左边点
      for(int left_index=left_col;left_index>=0;left_index--)
      {
        int pindex=left_p.y*cols+left_index;
        if(mapping_vt[pindex]<=0) continue;
        auto ppoint=points_ptr->at(mapping_vt[pindex]);
        Eigen::Vector4f pt_4f = {ppoint.x,ppoint.y,ppoint.z,0};
        // 到直线距离
        if(sqrt(pcl::sqrPointToLineDistance(pt_4f,line_pt_4f,line_dir_4f))<=0.015)
        {
          // 与端点距离
          if(pcl::euclideanDistance(ppoint,last_left_point)<0.03)
          {
            last_left_point=ppoint;
            last_left_index=pindex;
          }
        }
      }
      auto left_extend=pcl::euclideanDistance(last_left_point,slice_line->at(_line_inliers_.front()));
      if((left_extend+valid_length)<0.5*targetinfo.params[0] || (left_extend+valid_length)>2.0*targetinfo.params[0])
      {
        continue;
      }
      // 右侧
      for(int right_index=right_col;right_index<=(cols-1);right_index++)
      {
        int pindex=left_p.y*cols+right_index;
        if(mapping_vt[pindex]<=0) continue;
        auto ppoint=points_ptr->at(mapping_vt[pindex]);
        Eigen::Vector4f pt_4f = {ppoint.x,ppoint.y,ppoint.z,0};
        if(sqrt(pcl::sqrPointToLineDistance(pt_4f,line_pt_4f,line_dir_4f))<=0.015)
        {
          if(pcl::euclideanDistance(ppoint,last_right_point)<0.03)
          {
            last_right_point=ppoint;
            last_right_index=pindex;
          }
        }
      }
      auto right_extend=pcl::euclideanDistance(last_right_point,slice_line->at(_line_inliers_.back()));
      if((left_extend+valid_length+right_extend)<0.5*targetinfo.params[0] || (left_extend+valid_length+right_extend)>2.0*targetinfo.params[0])
      {
        continue;
      }
      // 确认行数据
      std::vector<mpoint> hyp_line;
      for(int pick=last_left_index;pick<=last_right_index;pick++)
      {
        if(mapping_vt[pick]>0)
        {
          auto pock_point=points_ptr->at(mapping_vt[pick]);
          hyp_line.emplace_back(mpoint(pock_point,pick,cols,rows,mpoint_type));
        }
      }
      if(hyp_line.size()>3)
      {
        target_lines.insert(std::make_pair(left_p.y,hyp_line));
        min_index=(min_index>left_p.y)?left_p.y:min_index;
        max_index=(max_index<left_p.y)?left_p.y:max_index;
      }
    }
    // 有效行数量要求
    if(max_index<=min_index) continue;
    float skipped_rate=fabs(fabs(max_index-min_index+1)-target_lines.size())/float(target_lines.size());
    pcl::PointCloud<pcl::PointXYZI>::Ptr top_bottom_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    for(auto top_point:target_lines[min_index])
    {
      top_bottom_ptr->push_back(top_point.point);
    }
    for(auto bottom_point:target_lines[max_index])
    {
      top_bottom_ptr->push_back(bottom_point.point);
    }
    pcl::PointXYZI _boundary_min_,_boundary_max_;
    pcl::getMinMax3D(*top_bottom_ptr,_boundary_min_,_boundary_max_);
    auto delta_height=fabs(_boundary_max_.y-_boundary_min_.y);
    if(target_lines.size()<threshold_region_rows || skipped_rate>0.5 || delta_height<0.5*targetinfo.params[1] || delta_height>2.0*targetinfo.params[1])
    {
      continue;
    }
    interm_result.second.emplace_back(intermediate(target_lines));
    // 有效区域可视化
    cv::line(hyp_info.first,vpoints[0],vpoints[3],cv::Scalar(255,0,255),2,cv::LINE_AA,0);
    cv::line(hyp_info.first,vpoints[1],vpoints[2],cv::Scalar(0,255,255),2,cv::LINE_AA,0);
  }
  interm_result.first=hyp_info.first;
  ROS_INFO_STREAM("[PerceptionStorage][TofDetector] DepthProposalsFilter "<<interm_result.second.size());
  // 对候选过滤
  std::vector<intermediate> support_intermidare;
  for(auto &hpy_target:interm_result.second)
  {
    if(base_filter(hpy_target))
    {
      intermediate support_target;
      if(support_filter(hpy_target,support_target))
      {
        support_intermidare.emplace_back(support_target);
      }
    }
  }
  interm_result.second=support_intermidare;
  ROS_INFO_STREAM("[PerceptionStorage][TofDetector] Intermidare "<<support_intermidare.size());
  // 计算候选目标姿态
  for(auto & patch:interm_result.second)
  {
    auto _plane_=filter_machine->naive_plane(patch.source_points,0.01);
    auto _plane_points_=std::get<0>(_plane_);
    auto _norm_center_=filter_machine->target_orient(TargetOrient::SVD,_plane_points_);
    auto _norm_=std::get<0>(_norm_center_);
    auto _center_=std::get<1>(_norm_center_);
    Eigen::Vector3f refer(1,0,0);
    if (_norm_(2)>0)
    {
      _norm_*=-1.0;
    }
    Eigen::Matrix4f pose_matrix=tool_box::pose_norm_refer(_norm_,_center_,refer);
    Eigen::Vector3f x_v=pose_matrix.col(0).head(3);
    Eigen::Vector3f z_v(0,1,0);
    Eigen::Vector3f y_v=z_v.cross(x_v);
    z_v=x_v.cross(y_v);
    Eigen::Matrix3f r_matrix;
    r_matrix.col(0)=x_v;r_matrix.col(1)=y_v;r_matrix.col(2)=z_v;
    pose_matrix.topLeftCorner<3, 3>() =r_matrix;
    pose_matrix(1,3)=0.0;
    detect_targets.emplace_back(mpose(pose_matrix,_norm_,_plane_points_));
    // 最终目标
    mpose refine_target;
    if(refine_filter(detect_targets.back(),cues_ptr,refine_target))
    {
      detect_result.emplace_back(refine_target);
    }
  }
}

bool tof_detector::base_filter(intermediate &target)
{
  auto _plane_=filter_machine->naive_plane(target.source_points,0.015);
  auto _plane_points_=std::get<0>(_plane_);
  if(_plane_points_->points.size()>3)
  {
    pcl::MomentOfInertiaEstimation <pcl::PointXYZI> feature_pca;
    feature_pca.setInputCloud(_plane_points_);
    feature_pca.compute();
    std::vector<float> moment_of_inertia;
    std::vector<float> eccentricity;
    pcl::PointXYZI min_point_AABB,max_point_AABB,min_point_OBB,max_point_OBB,position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector,mass_center;
    feature_pca.getMomentOfInertia(moment_of_inertia);
    feature_pca.getEccentricity(eccentricity);
    feature_pca.getAABB(min_point_AABB,max_point_AABB);
    feature_pca.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_pca.getEigenValues(major_value, middle_value, minor_value);
    feature_pca.getEigenVectors(major_vector, middle_vector, minor_vector);
    feature_pca.getMassCenter(mass_center);
    Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
    Eigen::Quaternionf quat(rotational_matrix_OBB);
    Eigen::Vector3f euler_angles = rotational_matrix_OBB.eulerAngles(1,0,2);
    float box_width = max_point_OBB.x - min_point_OBB.x;
    float box_height = max_point_OBB.y - min_point_OBB.y;
    float box_depth = max_point_OBB.z - min_point_OBB.z;
    // 物理尺寸
    if(box_depth>0.05 ||
      box_width>1.5*std::max(targetinfo.params[0],targetinfo.params[1]) ||
      box_width<0.5*targetinfo.params[0])
    {
      return false;
    }
  }
  else
  {
    return false;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr slice_left(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr slice_right(new pcl::PointCloud<pcl::PointXYZI>);
  for(auto &line:target.support_lines)
  {
    slice_left->push_back(line.second.front().point);
    slice_right->push_back(line.second.back().point);
  }
  // 左边界分析
  pcl::SampleConsensusModelLine<pcl::PointXYZI>::Ptr vl_model(new pcl::SampleConsensusModelLine<pcl::PointXYZI>(slice_left));	
  pcl::RandomSampleConsensus<pcl::PointXYZI> vl_fit(vl_model);	
  vl_fit.setDistanceThreshold(0.01);
  vl_fit.setMaxIterations(100);
  vl_fit.computeModel();
  std::vector<int> vl_inliers_;
  vl_fit.getInliers(vl_inliers_);
  if(vl_inliers_.size()<10)
  {
    return false;
  }
  Eigen::VectorXf vl_coef_;
  vl_fit.getModelCoefficients(vl_coef_);
  // 右边界分析
  pcl::SampleConsensusModelLine<pcl::PointXYZI>::Ptr vr_model(new pcl::SampleConsensusModelLine<pcl::PointXYZI>(slice_right));
  pcl::RandomSampleConsensus<pcl::PointXYZI> vr_fit(vr_model);
  vr_fit.setDistanceThreshold(0.01);
  vr_fit.setMaxIterations(100);
  vr_fit.computeModel();
  std::vector<int> vr_inliers_;
  vr_fit.getInliers(vr_inliers_);
  if(vr_inliers_.size()<10)
  {
    return false;
  }
  Eigen::VectorXf vr_coef_;
  vr_fit.getModelCoefficients(vr_coef_);
  Eigen::Vector4f _left_direction_={vl_coef_[3],vl_coef_[4],vl_coef_[5],0};
  Eigen::Vector4f _right_direction_={vr_coef_[3],vr_coef_[4],vr_coef_[5],0};
  auto _theta_=pcl::rad2deg(fabs(pcl::getAngle3D(_left_direction_,_right_direction_)));
  if(_theta_>90.0)
  {
    _theta_=180.0-_theta_;
  }
  if(_theta_>15)
  {
    return false;
  }
  return true;
}

bool tof_detector::support_filter(intermediate &target,intermediate &support_target)
{
  std::vector<Eigen::VectorXf> cof_list;
  std::vector<int> origin_list;
  for(auto &one_line:target.support_lines)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    for(auto & line_point_i:one_line.second)
    {
      temp_ptr->push_back(line_point_i.point);
    }
    pcl::SampleConsensusModelLine<pcl::PointXYZI>::Ptr refine_model(new pcl::SampleConsensusModelLine<pcl::PointXYZI>(temp_ptr));
    pcl::RandomSampleConsensus<pcl::PointXYZI> refine_fit(refine_model);	
    refine_fit.setDistanceThreshold(0.01);
    refine_fit.setMaxIterations(100);
    refine_fit.computeModel();
    std::vector<int> refine_inliers_;
    refine_fit.getInliers(refine_inliers_);
    Eigen::VectorXf refine_coef_;
    refine_fit.getModelCoefficients(refine_coef_);
    cof_list.emplace_back(refine_coef_);
    origin_list.emplace_back(one_line.first);
  }
  std::vector<int> support_id;
  std::vector<std::vector<int>> support_id_list;
  for(int i=0;i<cof_list.size();i++)
  {
    Eigen::Vector4f feature_i={cof_list[i][3],cof_list[i][4],cof_list[i][5],0};
    Eigen::Vector4f position_i={cof_list[i][0],cof_list[i][1],cof_list[i][2],0};
    std::vector<int> parallel_list;
    parallel_list.emplace_back(i);
    for(int j=i+1;j<cof_list.size();j++)
    {
      Eigen::Vector4f feature_j={cof_list[j][3],cof_list[j][4],cof_list[j][5],0};
      Eigen::Vector4f position_j={cof_list[j][0],cof_list[j][1],cof_list[j][2],0};
      auto _theta_=pcl::rad2deg(fabs(pcl::getAngle3D(feature_i,feature_j)));
      if(_theta_<1.5 || _theta_>178.5)
      {
        parallel_list.emplace_back(j);
      }
    }
    // 互信组合
    if(parallel_list.size()>=5)
    {
      support_id_list.emplace_back(parallel_list);
    }
  }
  if(support_id_list.size()==0)
  {
    return false;
  }
  support_id=support_id_list[0];
  for(auto &max_support:support_id_list)
  {
    if(max_support.size()>support_id.size()) support_id=max_support;
  }
  std::map<int,std::vector<mpoint>> support_info; 
  for(auto & valid_id:support_id)
  {
    int raw_index=origin_list[valid_id];
    std::vector<mpoint> support_points=target.support_lines[raw_index];
    support_info.insert(std::make_pair(raw_index,support_points));
  }
  support_target=intermediate(support_info);
}

bool tof_detector::refine_filter(mpose &target,pcl::PointCloud<pcl::PointXYZI>::Ptr &cues,mpose &result)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr face_cues(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*cues,*face_cues,target.transform.inverse());
  // 货架边界侧
  pcl::PointCloud<pcl::PointXYZI>::Ptr side_cues(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(face_cues);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits(0.5*targetinfo.lower_width-0.075,0.5*targetinfo.lower_width+0.075);
  pass.filter(*side_cues);
  pass.setInputCloud(side_cues);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits(-0.3*targetinfo.lower_length,0.3*targetinfo.lower_length);
  pass.filter(*side_cues);
  pass.setInputCloud(side_cues);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits(0.5*targetinfo.lower_height,targetinfo.lower_height+0.3*targetinfo.upper_height);
  pass.filter(*side_cues);
  if(side_cues->points.size()<10) return false;
  pcl::transformPointCloud(*side_cues,*side_cues,target.transform);
  std::map<int,pcl::PointCloud<pcl::PointXYZI>> select_side;
  for(auto &raw_p:side_cues->points)
  {
    auto theta=pcl::rad2deg(atan2(raw_p.z,raw_p.x));
    int theta_bin=floor(theta/0.25);
    if(select_side.find(theta_bin)!=select_side.end())
    {
      select_side[theta_bin].points.emplace_back(raw_p);
    }
    else
    { 
      pcl::PointCloud<pcl::PointXYZI> temple_pc;
      temple_pc.points.emplace_back(raw_p);
      select_side.insert(std::make_pair(theta_bin,temple_pc));
    } 
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr fliter_side(new pcl::PointCloud<pcl::PointXYZI>);
  for(auto & theta_x:select_side)
  {
    std::vector<float> dist_arrray;
    float min_dist=std::numeric_limits<float>::max();
    float max_dist=std::numeric_limits<float>::min();
    for(auto & x_p:theta_x.second.points)
    {
      auto _dist_=x_p.x*x_p.x+x_p.y*x_p.y;
      dist_arrray.emplace_back(_dist_);
      min_dist=(_dist_<min_dist)?_dist_:min_dist;
      max_dist=(_dist_>max_dist)?_dist_:max_dist;
    }
    float dist_range=min_dist+(max_dist-min_dist)*0.1;
    for(int i=0;i<dist_arrray.size();i++)
    {
      if(dist_arrray[i]<=dist_range)
      {
        fliter_side->points.emplace_back(theta_x.second.points[i]);
      }
    }
  }
  if(fliter_side->points.size()<5) return false;
  pcl::ModelCoefficients::Ptr zero_feature(new pcl::ModelCoefficients());
  zero_feature->values.resize(4);
  zero_feature->values[0] = 0;
  zero_feature->values[1] = 1.0;
  zero_feature->values[2] = 0;
  zero_feature->values[3] = 0;
  pcl::ProjectInliers<pcl::PointXYZI> cues_projector;
  cues_projector.setModelType(pcl::SACMODEL_PLANE);
  cues_projector.setInputCloud(fliter_side);
  cues_projector.setModelCoefficients(zero_feature);
  pcl::PointCloud<pcl::PointXYZI>::Ptr side_projected(new pcl::PointCloud<pcl::PointXYZI>);
  cues_projector.filter(*side_projected);
  pcl::PointCloud<pcl::PointXYZI>::Ptr face_tag(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud<pcl::PointXYZI>(*target.target_points,*face_tag);
  cues_projector.setInputCloud(face_tag);
  cues_projector.setModelCoefficients(zero_feature);
  pcl::PointCloud<pcl::PointXYZI>::Ptr tag_projected(new pcl::PointCloud<pcl::PointXYZI>);
  cues_projector.filter(*tag_projected);
  // 直线特征
  auto side_line_=filter_machine->naive_line(side_projected,0.009);
  pcl::PointCloud<pcl::PointXYZI>::Ptr side_projected_line(new pcl::PointCloud<pcl::PointXYZI>);
  side_projected_line=std::get<0>(side_line_);
  pcl::PointCloud<pcl::PointXYZI>::Ptr side_line_raw(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud<pcl::PointXYZI>(*side_projected_line,std::get<1>(side_line_),*side_line_raw);
  if(side_line_raw->points.size()<5) return false;
  Eigen::VectorXf side_projected_coef_=std::get<2>(side_line_);
  Eigen::Vector4f side_direction_={side_projected_coef_[3],side_projected_coef_[4],side_projected_coef_[5],0};
  if(side_direction_[0]>0) side_direction_*=-1.0;
  auto tag_line_=filter_machine->naive_line(tag_projected,0.009);
  pcl::PointCloud<pcl::PointXYZI>::Ptr tag_projected_line(new pcl::PointCloud<pcl::PointXYZI>);
  tag_projected_line=std::get<0>(tag_line_);
  pcl::PointCloud<pcl::PointXYZI>::Ptr tag_line_raw(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud<pcl::PointXYZI>(*face_tag,std::get<1>(tag_line_),*tag_line_raw);
  if(tag_line_raw->points.size()<5) return false;
  Eigen::VectorXf tag_projected_coef_=std::get<2>(tag_line_);
  Eigen::Vector4f tag_direction_={tag_projected_coef_[3],tag_projected_coef_[4],tag_projected_coef_[5],0};
  if(tag_direction_[0]>0) tag_direction_*=-1.0;
  Eigen::Vector4f norm_direction_={target.direction[0],target.direction[1],target.direction[2],0};
  auto _theta_=fabs(90.0-fabs(pcl::rad2deg(fabs(pcl::getAngle3D(side_direction_,norm_direction_)))));
  Eigen::Vector4f tag_center;
  pcl::compute3DCentroid(*tag_line_raw, tag_center);
  if(_theta_<=5.0)
  {
    Eigen::Matrix4f fine_matrix(Eigen::Matrix4f::Identity());
    Eigen::Vector3f y_v=side_direction_.head(3);
    Eigen::Vector3f z_v(0,1,0);
    Eigen::Vector3f x_v=y_v.cross(z_v);
    z_v=x_v.cross(y_v);
    Eigen::Matrix3f r_matrix;
    r_matrix.col(0)=x_v;r_matrix.col(1)=y_v;r_matrix.col(2)=z_v;
    fine_matrix.topLeftCorner<3, 3>() =r_matrix;
    fine_matrix.block<3,1>(0,3)=target.transform.block<3,1>(0,3);
    //fine_matrix.block<3,1>(0,3)=Eigen::Vector3f(tag_center(0),tag_center(1),tag_center(2));
    pcl::PointCloud<pcl::PointXYZI>::Ptr fine_points(new pcl::PointCloud<pcl::PointXYZI>);
    *fine_points+=*side_line_raw;
    *fine_points+=*tag_line_raw;
    result=mpose(fine_matrix,x_v,fine_points);
    result.confident_index=2;
  }
  // else if(_theta_<=10.0)
  // {
  //   Eigen::Matrix4f fine_matrix(Eigen::Matrix4f::Identity());
  //   Eigen::Vector3f x_v=target.direction;
  //   Eigen::Vector3f z_v(0,1,0);
  //   Eigen::Vector3f y_v=z_v.cross(x_v);
  //   z_v=x_v.cross(y_v);
  //   Eigen::Matrix3f r_matrix;
  //   r_matrix.col(0)=x_v;r_matrix.col(1)=y_v;r_matrix.col(2)=z_v;
  //   fine_matrix.topLeftCorner<3, 3>() =r_matrix;
  //   fine_matrix.block<3,1>(0,3)=Eigen::Vector3f(tag_center(0),tag_center(1),tag_center(2));
  //   pcl::PointCloud<pcl::PointXYZI>::Ptr fine_points(new pcl::PointCloud<pcl::PointXYZI>);
  //   *fine_points+=*tag_line_raw;
  //   result=mpose(fine_matrix,x_v,fine_points);
  // }
  else
  {
    return false;
  }
  return true;
}

std::pair<cv::Mat,std::vector<intermediate>> tof_detector::get_intermediate()
{
  return interm_result;
}

std::vector<mpose> tof_detector::get_targets()
{
  return detect_targets;
}

std::vector<mpose> tof_detector::get_result()
{
  return detect_result;
}

void tof_detector::set_roi(std_msgs::Header &header_info)
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