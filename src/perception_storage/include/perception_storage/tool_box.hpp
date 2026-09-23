#ifndef TOOLS_HPP
#define TOOLS_HPP

#include <iostream>
#include <algorithm>
#include <fstream>
#include <vector>
#include <queue> 
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/intersections.h>
#include <pcl/common/transforms.h>
#include <pcl/common/impl/intersections.hpp>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include "perception_storage/common.hpp"

namespace tool_box
{
  void sample_points(const cv::Mat& src, cv::Mat& dst, size_t count);
  std::vector<cv::Scalar> color_tables(int count);  
  void rect_template(TargetInfo &target_info,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &template_points);
  void distortion(Eigen::Vector2d &p_u,Eigen::Vector2d &d_u,cv::Mat &dist_coeff);
  void resume_pixel(Eigen::Vector3d &P_3d,Eigen::Vector2d &p_2d,cv::Mat &camera_matrix,cv::Mat &dist_coeff);
  void resume_point(Eigen::Vector2d &p_2d,Eigen::Vector3d &P_3d,cv::Mat &camera_matrix,cv::Mat &dist_coeff);
  void draw_elsed(cv::Mat &img,depth_runner::Segments segs,const cv::Scalar &color,int thickness);
  void max_box(depth_runner::Segment &seg1,depth_runner::Segment &seg2,std::vector<cv::Point2f> &points);
  void min_box(depth_runner::Segment &seg1,depth_runner::Segment &seg2,std::vector<cv::Point2f> &points);
  void base_box(depth_runner::Segment &seg1,depth_runner::Segment &seg2,std::vector<cv::Point2f> &points);
  Eigen::Matrix4d read_tf(const std::string& target_frame, const std::string& source_frame, double time_out);
  Eigen::Vector3f matrix2euler(Eigen::Matrix3f &R);
  Eigen::Matrix4f pose_3d_3d();
  Eigen::Matrix4f pose_norm_refer(Eigen::Vector3f &norm,Eigen::Vector3f &center,Eigen::Vector3f &refer);
  std::tuple<float,float,float> dist_points_line(pcl::PointCloud<pcl::PointXYZI>::Ptr &points,Eigen::VectorXf &line_feature);
};

#endif