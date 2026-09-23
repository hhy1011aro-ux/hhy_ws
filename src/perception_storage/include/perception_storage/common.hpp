#ifndef COMMON_HPP
#define COMMON_HPP

#include <iostream>
#include <algorithm>
#include <fstream>
#include <vector>
#include <queue> 
#include <tuple>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ceres/ceres.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Header.h>
#include "perception_storage/depth_runner/depth_elsed.h"

using ceres::CostFunction;
using ceres::Problem;
using ceres::SizedCostFunction;
using ceres::Solve;
using ceres::Solver;

enum class SensorType
{
  TOF,
  LIDAR,
  FUSHION
};

enum class ActiveState
{
  SLEEPY,
  LEFT,
  RIGHT
};

enum class TargetOrient 
{ 
  NONE, 
  QR,
  SVD,
  CERES
};

struct SensorInfo
{
  SensorType sensor_type;
  // 消息名称
  std::string topic_left;
  std::string topic_right;
  // 额外的消息
  // 灰度图
  std::string gray_left;
  std::string gray_right;
  // 内参
  std::string cam_left;
  std::string cam_right;
  // fov/深度图尺寸
  float width;
  float height;
  // 坐标系翻转
  bool axis_flip;
  // 距离补偿
  float distance_compensation;
  SensorInfo()
  {

  };
  
  SensorInfo(const SensorType & _type_,const std::string &_left_,const std::string & _right_,
             const float & _width_,const float & _height_,const bool & _flip_,const float & _distance_compensation_):
             sensor_type(_type_),topic_left(_left_),topic_right(_right_),
             width(_width_),height(_height_),axis_flip(_flip_),distance_compensation(_distance_compensation_)
  {

  };

  SensorInfo(const SensorType & _type_,const std::string &_left_,const std::string & _right_,
             const std::string &_gray_left_,const std::string &_gray_right_,const std::string &_cam_left_,const std::string &_cam_right_,
             const float & _width_,const float & _height_,const bool & _flip_,const float & _distance_compensation_):
             sensor_type(_type_),topic_left(_left_),topic_right(_right_),
             gray_left(_gray_left_),gray_right(_gray_right_),cam_left(_cam_left_),cam_right(_cam_right_),
             width(_width_),height(_height_),axis_flip(_flip_),distance_compensation(_distance_compensation_)
  {

  };
};

struct TargetInfo
{
  // 上部
  float upper_length;
  float upper_width;
  float upper_height;
  // 底部
  float lower_length;
  float lower_width;
  float lower_height;
  // 特征信息
  std::string feature_name;
  std::vector<float> params;
  TargetInfo()
  {

  };
  TargetInfo(const float & _upper_length_,const float & _upper_width_,const float & _upper_height_,
             const float & _lower_length_,const float & _lower_width_,const float & _lower_height_,
             const std::string &_feature_name_,const std::vector<float> &_params_):
             upper_length(_upper_length_),upper_width(_upper_width_),upper_height(_upper_height_),
             lower_length(_lower_length_),lower_width(_lower_width_),lower_height(_lower_height_),
             feature_name(_feature_name_),params(_params_)
  {

  };
};

struct SettingInfo
{
  bool trouble_shoting;
  SensorType sensor_type;
  int batch_size;
  bool reference_pose;
  int stop_criteria;
  SettingInfo()
  {

  };
  SettingInfo(const bool &_trouble_shoting_,const std::string &_sensortype_,const std::vector<int> &_batch_size_,
              const bool &_reference_pose_,const int &_stop_criteria_):
              trouble_shoting(_trouble_shoting_),reference_pose(_reference_pose_),
              stop_criteria(_stop_criteria_)
  {
    if(_sensortype_=="tof") 
    {
      sensor_type=SensorType::TOF;
      batch_size=_batch_size_[0];
    }
    if(_sensortype_=="lidar") 
    {
      sensor_type=SensorType::LIDAR;
      batch_size=_batch_size_[1];
    }
    if(_sensortype_=="fushion")
    {
      sensor_type=SensorType::FUSHION;
      batch_size=_batch_size_[1];
    } 
  };
};

struct PubInfo
{
  std::string service_name;
  std::string result_name;
  std::string reference_name;
  std::string refer_region_name;
  std::string naive_region_name;
  std::string raw_name;
  std::string depth_name;
  std::string hyp_points_name;
  std::string hyp_pose_name;
  std::string evidence_points_name;
  std::string evidence_pose_name;
  std::string remember_pose_name;
  std::string repub_points_name;
  std::string repub_image_name;
  std::string lidar_capture_name;
  PubInfo()
  {

  };
  PubInfo(const std::string &_service_name_,const std::string &_result_name_,
         const std::string &_reference_name_,const std::string &_refer_region_name_,const std::string &_naive_region_name_,
         const std::string &_raw_name_,const std::string &_depth_name_,
         const std::string &_hyp_points_name_,const std::string &_hyp_pose_name_,
         const std::string &_evidence_points_name_,const std::string &_evidence_pose_name_,
         const std::string &_remember_pose_name_,
         const std::string &_repub_points_name_,const std::string &_repub_image_name_,const std::string &_lidar_capture_name_):
         service_name(_service_name_),result_name(_result_name_),
         reference_name(_reference_name_),refer_region_name(_refer_region_name_),naive_region_name(_naive_region_name_),
         raw_name(_raw_name_),depth_name(_depth_name_),
         hyp_points_name(_hyp_points_name_),hyp_pose_name(_hyp_pose_name_),
         evidence_points_name(_evidence_points_name_),evidence_pose_name(_evidence_pose_name_),
         remember_pose_name(_remember_pose_name_),
         repub_points_name(_repub_points_name_),repub_image_name(_repub_image_name_),lidar_capture_name(_lidar_capture_name_)
  {

  };
};

struct ConfigInfo
{
  SensorInfo m_lidarinfo;
  SensorInfo m_tofinfo;
  TargetInfo m_targetinfo;
  SettingInfo m_settinginfo;
  PubInfo m_pubinfo;
  ConfigInfo()
  {

  };
};

struct CameraInfo
{
  cv::Mat cameraMatrix;
  cv::Mat distorsionCoeff;
  cv::Size size;
  CameraInfo()
  {

  };
  CameraInfo(const cv::Mat &_matrix_,const cv::Mat &_dist_,const cv::Size &_size_)
  {
    cameraMatrix=_matrix_;
    distorsionCoeff=_dist_;
    size=_size_;
  }
};

struct TofData
{
  CameraInfo camera_info;
  cv::Mat gray_image;
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud;
  std_msgs::Header msg_header;
  TofData()
  {

  };
  TofData(CameraInfo &_camera_info_,const sensor_msgs::ImageConstPtr &_image_,const sensor_msgs::PointCloud2ConstPtr &_points_):camera_info(_camera_info_)
  {
    // points
    point_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*_points_, *point_cloud);
    msg_header=_points_->header;
    // image
    if(_image_->encoding=="16UC1")
    {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(_image_, sensor_msgs::image_encodings::TYPE_16UC1);
      gray_image = cv_ptr->image;
      cv::Mat temp_img;
      gray_image.convertTo(temp_img, CV_8UC1, 1.0 / 256);
      cv::equalizeHist(temp_img,temp_img);
      gray_image=temp_img;
    }
    else if(_image_->encoding=="8UC1")
    {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(_image_, sensor_msgs::image_encodings::TYPE_8UC1);
      gray_image = cv_ptr->image;
      cv::equalizeHist(gray_image,gray_image);
    }
  };
};

struct LidarDta
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud;
  std_msgs::Header msg_header;
  LidarDta()
  {

  };
  LidarDta(const sensor_msgs::PointCloud2ConstPtr &_points_)
  {
    point_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*_points_, *point_cloud);
    msg_header=_points_->header;
  };
};

// 消息打包
struct MsgPacket
{
  // left or right
  ActiveState enum_side;
  // sensor type(1,tof数据；2,雷达数据)
  int data_type;
  // tof数据
  TofData tof_data;
  // lidar数据
  LidarDta lidar_data;

  MsgPacket()
  {

  };
  // itof
  MsgPacket(const int &side_info,
            CameraInfo &_camera_info_,
            const sensor_msgs::ImageConstPtr &_image_,
            const sensor_msgs::PointCloud2ConstPtr &_tof_points_)
  {
    if(side_info==1) enum_side=ActiveState::LEFT;
    if(side_info==2) enum_side=ActiveState::RIGHT;
    tof_data=TofData(_camera_info_,_image_,_tof_points_);
    data_type=1;
  };
  // lidar
  MsgPacket(const int &side_info,
            const sensor_msgs::PointCloud2ConstPtr &_lidar_points_)
  {
    if(side_info==1) enum_side=ActiveState::LEFT;
    if(side_info==2) enum_side=ActiveState::RIGHT;
    lidar_data=LidarDta(_lidar_points_);
    data_type=2;
  };
};

// 深度图与点云组合
struct depth_mapping
{
  cv::Mat depth;
  pcl::PointCloud<pcl::PointXYZI>::Ptr points;
  std::vector<int> mapping;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cues_points;
  Eigen::Vector2i mid_pixel;
  depth_mapping()
  {
    mid_pixel=Eigen::Vector2i(-1,-1);
  };
  depth_mapping(const cv::Mat &_depth_,const pcl::PointCloud<pcl::PointXYZI>::Ptr &_points_,const std::vector<int> &_mapping_):
                depth(_depth_),points(_points_),mapping(_mapping_)
  {
    mid_pixel=Eigen::Vector2i(-1,-1);
  };
  depth_mapping(const cv::Mat &_depth_,const pcl::PointCloud<pcl::PointXYZI>::Ptr &_points_,
                const std::vector<int> &_mapping_,const pcl::PointCloud<pcl::PointXYZI>::Ptr &_cues_):
                depth(_depth_),points(_points_),mapping(_mapping_),cues_points(_cues_)
  {
    mid_pixel=Eigen::Vector2i(-1,-1);
  };
  depth_mapping(const cv::Mat &_depth_,const pcl::PointCloud<pcl::PointXYZI>::Ptr &_points_,
                const std::vector<int> &_mapping_,const pcl::PointCloud<pcl::PointXYZI>::Ptr &_cues_,Eigen::Vector2i &_mid_pixel_):
                depth(_depth_),points(_points_),mapping(_mapping_),cues_points(_cues_),mid_pixel(_mid_pixel_)
  {

  };

};

struct hypothesis
{
  std::vector<cv::Point2f> four_points;
  int img_rows;
  int img_cols;
  std::vector<std::pair<cv::Point,cv::Point>> edge_points; 

  hypothesis()
  {

  };

  hypothesis(std::vector<cv::Point2f> & _four_points_,int & _img_rows_,int & _img_cols_):
             four_points(_four_points_),img_rows(_img_rows_),img_cols(_img_cols_)
  {
    int min_y=(round(four_points[0].y)<0)?0:(round(four_points[0].y));
    int max_y=(round(four_points[1].y)>(img_rows-1))?(img_rows-1):(round(four_points[1].y));
    int valid_y=max_y-min_y+1;
    for(int i=0;i<valid_y;i++)
    {
      auto x_left=(four_points[0].x-four_points[1].x)/(four_points[0].y-four_points[1].y)*(max_y-i-four_points[1].y)+four_points[1].x; 
      auto x_right=(four_points[2].x-four_points[3].x)/(four_points[2].y-four_points[3].y)*(max_y-i-four_points[3].y)+four_points[3].x;
      edge_points.emplace_back(std::make_pair(cv::Point(round(x_left),max_y-i),cv::Point(round(x_right),max_y-i)));
    }
  };
};

struct mpoint
{ 
  pcl::PointXYZI point;
  int index;
  int row;
  int col;
  int width;
  int height;
  mpoint()
  {

  };
  mpoint(const pcl::PointXYZI &point_,const int &index_,const int &row_,const int &col_):
        point(point_),index(index_),row(row_),col(col_)
  {

  };
  mpoint(const pcl::PointXYZI &point_,const int &index_,const int  &width_,const int  &height_,const int &type_):
        point(point_),index(index_),width(width_),height(height_)
  {
    row=floor(index/width);
    col=index%width;
  };
};

struct intermediate
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_points;
  std::map<int,std::vector<mpoint>> support_lines;
  intermediate()
  {

  };
  intermediate(const pcl::PointCloud<pcl::PointXYZI>::Ptr & _source_points_):source_points(_source_points_)
  {

  };
  intermediate(const std::map<int,std::vector<mpoint>> &_support_lines_):support_lines(_support_lines_)
  {
    source_points.reset(new pcl::PointCloud<pcl::PointXYZI>());
    for(auto &line:support_lines)
    {
      for(auto &point:line.second)
      {
        source_points->points.emplace_back(point.point);
      }
    }
  };
};

struct mpose
{
  geometry_msgs::PoseStamped pose;
  Eigen::Matrix4f transform;
  Eigen::Vector3f direction;
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_points;
  int confident_index=0;
  mpose()
  {

  };
  mpose(geometry_msgs::PoseStamped &_pose_,Eigen::Matrix4f &_transform_,Eigen::Vector3f &_direction_,
        pcl::PointCloud<pcl::PointXYZI>::Ptr &_target_points_):
        pose(_pose_),transform(_transform_),direction(_direction_),target_points(_target_points_)
  {

  };
  mpose(Eigen::Matrix4f &_transform_,
        Eigen::Vector3f &_direction_,
        pcl::PointCloud<pcl::PointXYZI>::Ptr &_target_points_):transform(_transform_),direction(_direction_),target_points(_target_points_)
  {
    Eigen::Quaternionf quat(transform.topLeftCorner<3, 3>());
    quat.normalize();
    pose.pose.orientation.x = quat.x();
    pose.pose.orientation.y = quat.y();
    pose.pose.orientation.z = quat.z();
    pose.pose.orientation.w = quat.w();
    pose.pose.position.x = transform(0,3);
    pose.pose.position.y = transform(1,3);
    pose.pose.position.z = transform(2,3);
  };
};

// ceres cost
struct PLANE_FITTING_COST
{
  PLANE_FITTING_COST(double x, double y, double z) : _x(x), _y(y), _z(z) 
  {
  }
  template<typename T> bool
  operator()(const T *const abcd,T *residual) const
  {
    residual[0] = T(abs(abcd[0] * _x + abcd[1] * _y + abcd[2] * _z + abcd[3])
                    / sqrt(abcd[0] * abcd[0] + abcd[1] * abcd[1] + abcd[2] * abcd[2]));
    return true;
  }
  const double _x, _y, _z;
};

// 外部请求的信息记录
struct ReqInfo
{
  // 收到任务的时间
  ros::Time task_time;
  // 使用哪一边的传感器
  ActiveState side_setting;
  // 提供的参考位置
  std::vector<float> reference_pose;
  
  ReqInfo()
  {};

  ReqInfo(const std::string &side_info,const std::vector<float> &pose_info)
  {
    task_time=ros::Time::now();
    if(side_info=="left") side_setting=ActiveState::LEFT;
    if(side_info=="right") side_setting=ActiveState::RIGHT;
    reference_pose=pose_info;
  };   
};

struct TaskInfo
{
  std::string sensor;
  std::string side;
  int infer_n;
  int hyp_n;
  int result_n;
  TaskInfo()
  {

  };
  TaskInfo(std::string &sensor_,std::string &side_,int &infern_,int &hypn_,int resultn_):sensor(sensor_),side(side_),infer_n(infern_),hyp_n(hypn_),result_n(resultn_)
  {

  }
};

// 固定颜色表
static std::vector<cv::Scalar> static_render=
{ 
  cv::Scalar  {100, 20, 50}, 
  cv::Scalar  {0, 30, 255}, 
  cv::Scalar  {10, 255, 60}, 
  cv::Scalar  {80, 10, 100},
  cv::Scalar  {0, 255, 200}, 
  cv::Scalar  {10, 60, 60}, 
  cv::Scalar  {255, 0, 128}, 
  cv::Scalar  {60, 128, 128}, 
  cv::Scalar  {15, 99, 98}, 
  cv::Scalar  {102, 159, 227}, 
  cv::Scalar  {245, 180, 173}, 
  cv::Scalar  {37, 229, 49}, 
  cv::Scalar  {110, 81, 166}, 
  cv::Scalar  {238, 29, 94}, 
  cv::Scalar  {232, 57, 157}, 
  cv::Scalar  {17, 209, 187}, 
  cv::Scalar  {117, 165, 112}, 
  cv::Scalar  {38, 25, 29}, 
  cv::Scalar  {9, 201, 164}, 
  cv::Scalar  {149, 110, 55}, 
  cv::Scalar  {238, 108, 3}, 
  cv::Scalar  {113, 69, 235}, 
  cv::Scalar  {47, 52, 61},     
  cv::Scalar  {217, 129, 225}, 
  cv::Scalar  {19, 58, 205}, 
  cv::Scalar  {46, 234, 98}, 
  cv::Scalar  {206, 249, 79}, 
  cv::Scalar  {134, 45, 252}, 
  cv::Scalar  {96, 61, 59}, 
  cv::Scalar  {149, 203, 31}, 
  cv::Scalar  {119, 178, 190}, 
  cv::Scalar  {92, 168, 242}, 
  cv::Scalar  {137, 78, 217}, 
  cv::Scalar  {186, 167, 91}, 
  cv::Scalar  {28, 186, 21}, 
  cv::Scalar  {105, 233, 1}, 
  cv::Scalar  {203, 56, 122}, 
  cv::Scalar  {27, 62, 167}, 
  cv::Scalar  {25, 158, 100}, 
  cv::Scalar  {211, 180, 176}, 
  cv::Scalar  {114, 44, 99}, 
  cv::Scalar  {49, 8, 139}, 
  cv::Scalar  {164, 145, 217}, 
  cv::Scalar  {126, 76, 130}, 
  cv::Scalar  {89, 231, 188}, 
  cv::Scalar  {85, 162, 91}, 
  cv::Scalar  {180, 140, 20}, 
  cv::Scalar  {209, 157, 229}
};

#endif
