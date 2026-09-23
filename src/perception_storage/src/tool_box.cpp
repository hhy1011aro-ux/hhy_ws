#include "perception_storage/tool_box.hpp"

namespace tool_box
{

void sample_points(const cv::Mat& src, cv::Mat& dst, size_t count)
{
  CV_Assert(count >= 2);
  CV_Assert(src.cols == 1 || src.rows == 1);
  CV_Assert(src.total() >= count);
  CV_Assert(src.type() == CV_8UC3);
  dst.create(1, (int)count, CV_8UC3);
  cv::Mat dists((int)src.total(), (int)src.total(), CV_32FC1, cv::Scalar(0));
  if (dists.empty())
  {
    std::cerr << "such big matrix cann't be created." << std::endl;
  }     
  for (int i = 0; i < dists.rows; i++)
  {
    for (int j = i; j < dists.cols; j++)
    {
      float dist = (float)norm(src.at<cv::Point3_<uchar> >(i) - src.at<cv::Point3_<uchar> >(j));
      dists.at<float>(j, i) = dists.at<float>(i, j) = dist;
    }
  }
  double maxVal;
  cv::Point maxLoc;
  cv::minMaxLoc(dists, 0, &maxVal, 0, &maxLoc);
  dst.at<cv::Point3_<uchar> >(0) = src.at<cv::Point3_<uchar> >(maxLoc.x);
  dst.at<cv::Point3_<uchar> >(1) = src.at<cv::Point3_<uchar> >(maxLoc.y);
  cv::Mat activedDists(0, dists.cols, dists.type());
  cv::Mat candidatePointsMask(1, dists.cols, CV_8UC1, cv::Scalar(255));
  activedDists.push_back(dists.row(maxLoc.y));
  candidatePointsMask.at<uchar>(0, maxLoc.y) = 0;
  for (size_t i = 2; i < count; i++)
  {
    activedDists.push_back(dists.row(maxLoc.x));
    candidatePointsMask.at<uchar>(0, maxLoc.x) = 0;
    cv::Mat minDists;
    cv::reduce(activedDists, minDists, 0, CV_REDUCE_MIN);
    cv::minMaxLoc(minDists, 0, &maxVal, 0, &maxLoc, candidatePointsMask);
    dst.at<cv::Point3_<uchar> >((int)i) = src.at<cv::Point3_<uchar> >(maxLoc.x);
  }
}

std::vector<cv::Scalar> color_tables(int count)
{
  std::vector<cv::Scalar> colors;
  colors.resize(count);
  if (count == 1)
  {
    colors[0] = cv::Scalar(0, 0, 255);
    return colors;
  }
  if (count == 2)
  {
    colors[0] = cv::Scalar(0, 0, 255);
    colors[1] = cv::Scalar(0, 255, 0);
    return colors;
  }
  int factor=100;
  cv::Mat bgr(1, (int)(count * factor), CV_8UC3);
  cv::randu(bgr, 0, 256);
  cv::Mat lab;
  cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);
  cv::Mat lab_subset;
  sample_points(lab, lab_subset, count);
  cv::Mat bgr_subset;
  cv::cvtColor(lab_subset, bgr_subset, cv::COLOR_BGR2Lab);
  CV_Assert(bgr_subset.total() == count);
  for (size_t i = 0; i < count; i++)
  {
    cv::Point3_<uchar> c = bgr_subset.at<cv::Point3_<uchar> >((int)i);
    colors[i] = cv::Scalar(c.x, c.y, c.z);
  }
  return colors;
}

void rect_template(TargetInfo &target_info,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &template_points)
{
  float y_start=-target_info.params[0]*0.5;
  int y_steps=ceil(target_info.params[0]/0.005);
  float z_start=-target_info.params[1]*0.5;
  int z_steps=ceil(target_info.params[1]/0.005);
  for(int i=0;i<y_steps;i++)
  {  
    for(int j=0;j<z_steps;j++)
    {
      pcl::PointXYZRGB tp;
      tp.x=0;
      tp.y=y_start+i*0.005;
      tp.z=z_start+j*0.005;
      int r=0;int g=0;int b=0;
      if(tp.y>=0 && fabs(tp.z)<0.01) 
      {
        r=0;g=255;b=0;
      }
      if(tp.z>=0 && fabs(tp.y)<0.01) 
      {
        r=0;g=0;b=255;
      }
      tp.r=r;tp.g=g;tp.b=b;
      template_points->points.emplace_back(tp);
    }
  }
  int x_steps=ceil(1.0/0.005);
  for(int k=0;k<x_steps;k++)
  {
    pcl::PointXYZRGB xp;
    xp.x=k*0.005;
    xp.y=0;
    xp.z=0;
    xp.r=255;xp.g=0;xp.b=0;
    template_points->points.emplace_back(xp);
  }
}

// 对归一化的点计算畸变量
void distortion(Eigen::Vector2d &p_u,Eigen::Vector2d &d_u,cv::Mat &dist_coeff)
{
  double k1 =dist_coeff.at<double>(0,0);
  double k2 =dist_coeff.at<double>(0,1);
  double p1 =dist_coeff.at<double>(0,2);
  double p2 =dist_coeff.at<double>(0,3);
  double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;
  //
  mx2_u = p_u(0) * p_u(0);
  my2_u = p_u(1) * p_u(1);
  mxy_u = p_u(0) * p_u(1);
  rho2_u = mx2_u + my2_u;
  rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
  d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
         p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
}

// 相机坐标系的3d点转化到像素坐标系
void resume_pixel(Eigen::Vector3d &P_3d,Eigen::Vector2d &p_2d,cv::Mat &camera_matrix,cv::Mat &dist_coeff)
{
  Eigen::Vector2d p_u, p_d;
  // 归一化
  p_u << P_3d(0)/P_3d(2),
         P_3d(1)/P_3d(2);
  // 恢复畸变
  Eigen::Vector2d d_u;
  distortion(p_u, d_u,dist_coeff);
  p_d = p_u + d_u;
  double fx =camera_matrix.at<double>(0,0);
  double fy =camera_matrix.at<double>(1,1);
  double cx =camera_matrix.at<double>(0,2);
  double cy =camera_matrix.at<double>(1,2);
  // 像素坐标
  p_2d << fx* p_d(0) + cx,
          fy* p_d(1) + cy;
}

// 从像素到等效3d点
void resume_point(Eigen::Vector2d &p_2d,Eigen::Vector3d &P_3d,cv::Mat &camera_matrix,cv::Mat &dist_coeff)
{
  double m_inv_K11 = 1.0 / camera_matrix.at<double>(0,0);
  double m_inv_K13 = -camera_matrix.at<double>(0,2) / camera_matrix.at<double>(0,0);
  double m_inv_K22 = 1.0 / camera_matrix.at<double>(1,1);
  double m_inv_K23 = -camera_matrix.at<double>(0,2) / camera_matrix.at<double>(1,1);
  double mx_d, my_d, mx_u, my_u;
  mx_d = m_inv_K11 * p_2d(0) + m_inv_K13;
  my_d = m_inv_K22 * p_2d(1) + m_inv_K23;
  int n = 8;
  Eigen::Vector2d d_u;
  Eigen::Vector2d tpu(mx_d, my_d);
  distortion(tpu,d_u,dist_coeff);
  mx_u = mx_d - d_u(0);
  my_u = my_d - d_u(1);
  for (int i = 1; i < n; ++i)
  { 
    Eigen::Vector2d tpu_(mx_u, my_u);
    distortion(tpu_,d_u,dist_coeff);
    mx_u = mx_d - d_u(0);
    my_u = my_d - d_u(1);
  }
  P_3d << mx_u, 
          my_u, 
          1.0;
  P_3d.normalize();
}

void draw_elsed(cv::Mat &img,depth_runner::Segments segs,const cv::Scalar &color,int thickness) 
{
  int lineType = cv::LINE_AA;
  int shift = 0;
  for (const depth_runner::Segment &seg: segs)
  {
    cv::line(img,cv::Point2f(seg[0],seg[1]),cv::Point2f(seg[2],seg[3]), color, thickness, lineType, shift);
  }
}

void max_box(depth_runner::Segment &seg1,depth_runner::Segment &seg2,std::vector<cv::Point2f> &points)
{
  std::vector<cv::Point2f> four_points;
  base_box(seg1,seg2,four_points);
  // 
  auto min_y=std::min(four_points[0].y,four_points[2].y);
  auto max_y=std::max(four_points[1].y,four_points[3].y);
  //
  auto min_x_left=(four_points[0].x-four_points[1].x)/(four_points[0].y-four_points[1].y)*(min_y-four_points[1].y)+four_points[1].x;
  auto max_x_left=(four_points[1].x-four_points[0].x)/(four_points[1].y-four_points[0].y)*(max_y-four_points[0].y)+four_points[0].x;
  auto min_x_right=(four_points[2].x-four_points[3].x)/(four_points[2].y-four_points[3].y)*(min_y-four_points[3].y)+four_points[3].x;
  auto max_x_right=(four_points[3].x-four_points[2].x)/(four_points[3].y-four_points[2].y)*(max_y-four_points[2].y)+four_points[2].x;
  points={cv::Point2f(min_x_left,min_y),cv::Point2f(max_x_left,max_y),cv::Point2f(min_x_right,min_y),cv::Point2f(max_x_right,max_y)};
}

void min_box(depth_runner::Segment &seg1,depth_runner::Segment &seg2,std::vector<cv::Point2f> &points)
{
  std::vector<cv::Point2f> four_points;
  base_box(seg1,seg2,four_points);
  // 有重叠
  if((four_points[0].y>=four_points[3].y && four_points[0].y<=four_points[2].y) || 
     (four_points[1].y>=four_points[3].y && four_points[1].y<=four_points[2].y))
  {
    auto t_y=std::max(four_points[0].y,four_points[2].y);
    auto d_y=std::min(four_points[1].y,four_points[3].y);
    cv::Point2f tl;
    tl.y=t_y;
    tl.x=(four_points[0].x-four_points[1].x)/(four_points[0].y-four_points[1].y)*(t_y-four_points[1].y)+four_points[1].x;
    cv::Point2f dl;
    dl.y=d_y;
    dl.x=(four_points[1].x-four_points[0].x)/(four_points[1].y-four_points[0].y)*(d_y-four_points[0].y)+four_points[0].x;
    cv::Point2f tr;
    tr.y=t_y;
    tr.x=(four_points[2].x-four_points[3].x)/(four_points[2].y-four_points[3].y)*(t_y-four_points[3].y)+four_points[3].x;
    cv::Point2f dr;
    dr.y=d_y;
    dr.x=(four_points[3].x-four_points[2].x)/(four_points[3].y-four_points[3].y)*(d_y-four_points[2].y)+four_points[2].x;
    points={tl,dl,tr,dr};
  }
}

void base_box(depth_runner::Segment &seg1,depth_runner::Segment &seg2,std::vector<cv::Point2f> &points)
{
  cv::Point2f p1_1=cv::Point2f(seg1[0],seg1[1]);
  cv::Point2f p1_2=cv::Point2f(seg1[2],seg1[3]); 
  cv::Point2f p2_1=cv::Point2f(seg2[0],seg2[1]);
  cv::Point2f p2_2=cv::Point2f(seg2[2],seg2[3]);
  // 左右划分
  if((p1_1.x+p1_2.x)>((p2_1.x+p2_2.x)))
  {
    p1_1=cv::Point2f(seg2[0],seg2[1]);
    p1_2=cv::Point2f(seg2[2],seg2[3]); 
    p2_1=cv::Point2f(seg1[0],seg1[1]);
    p2_2=cv::Point2f(seg1[2],seg1[3]);
  }
  // 上下划分
  if(p1_1.y>p1_2.y)
  {
    std::swap(p1_1,p1_2);
  }
  if(p2_1.y>p2_2.y)
  {
    std::swap(p2_1,p2_2);
  }
  points={p1_1,p1_2,p2_1,p2_2};
}

Eigen::Matrix4d read_tf(const std::string& target_frame, const std::string& source_frame, double time_out)
{
  static tf::TransformListener listener;
  ros::Time::waitForValid();
  tf::StampedTransform transform;
  transform.setIdentity();
  bool find_tf=false;
  while(!find_tf)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    try
    {
      listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(time_out));
      listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
      find_tf=true;
      break;
    }
    catch(const std::exception& e)
    { 
      ROS_ERROR("Time out,Waiting %f second(s) for static transform from %s to %s", time_out, source_frame.c_str(), target_frame.c_str());
      continue;
    }
  }
 
  tf::StampedTransform add_transform;
  tf::Quaternion quaternion; 
  double yaw=0.0;
  double pitch=0.0;
  double roll=M_PI/2.0;
  if(transform.getOrigin().getX()<0)
  {
    roll=-M_PI/2.0;
  }
  quaternion.setEuler(yaw,pitch,roll);
  add_transform.setRotation(quaternion);
  auto _transform_=transform;
  Eigen::Isometry3d trans_matrix(Eigen::Isometry3d::Identity());
  tf::poseTFToEigen(_transform_, trans_matrix);
  Eigen::Matrix4d matrix(Eigen::Matrix4d::Identity());
  matrix=trans_matrix.matrix().cast<double>();

  return matrix;
}

Eigen::Vector3f matrix2euler(Eigen::Matrix3f &R)
{
  Eigen::Vector3f vector;
  assert(isRotationMatirx(R));
  double sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
  bool singular = sy < 1e-6;
  double x, y, z;
  if (!singular)
  {
    x = atan2( R(2,1), R(2,2));
    y = atan2(-R(2,0), sy);
    z = atan2( R(1,0), R(0,0));
  }
  else
  {
    x = atan2(-R(1,2), R(1,1));
    y = atan2(-R(2,0), sy);
    z = 0;
  }
  vector << x, y, z;
  return vector;
}

Eigen::Matrix4f pose_norm_refer(Eigen::Vector3f &norm,Eigen::Vector3f &center,Eigen::Vector3f &refer)
{
  Eigen::Vector3f _n_ = refer.cross(norm);
  float _theta_ = acos(refer.dot(norm));
  Eigen::AngleAxisf _rotation_vector_(_theta_, _n_);
  Eigen::Matrix3f _rotation_matrix_=_rotation_vector_.toRotationMatrix();
  Eigen::Matrix3f normal_matrix;
  normal_matrix << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  Eigen::Matrix3f _rotation_matrix_sum_;
  _rotation_matrix_sum_ = normal_matrix * _rotation_matrix_;
  Eigen::Matrix4f pose_matrix(Eigen::Matrix4f::Identity());
  pose_matrix.topLeftCorner<3, 3>() =_rotation_matrix_sum_;
  pose_matrix.block<3,1>(0,3)=center;
  return pose_matrix;
}

std::tuple<float,float,float> dist_points_line(pcl::PointCloud<pcl::PointXYZI>::Ptr &points,Eigen::VectorXf &line_feature)
{
  float mean_dist=0;
  float max_dist=std::numeric_limits<float>::min();
  float min_dist=std::numeric_limits<float>::max();
  for(auto & tp:points->points)
  {
    Eigen::Vector4f tp4f(tp.x,tp.y,tp.z,0);
    auto _dist_ =std::sqrt(pcl::sqrPointToLineDistance(tp4f,
                                          Eigen::Vector4f(line_feature(0),line_feature(1),line_feature(2),0),
                                          Eigen::Vector4f(line_feature(3),line_feature(4),line_feature(5),0)));
    mean_dist+=_dist_;
    max_dist=(_dist_>max_dist)?_dist_:max_dist;
    min_dist=(_dist_<min_dist)?_dist_:min_dist;
  }
  mean_dist/=points->points.size();

  return std::make_tuple(max_dist,min_dist,mean_dist);
}

}