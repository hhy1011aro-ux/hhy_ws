#ifndef PERCEPTION_TRACKER_HPP
#define PERCEPTION_TRACKER_HPP

#include <string>
#include <iostream>
#include <map>
#include <algorithm>
#include <fstream>
#include <vector>
#include <queue>
#include <stdio.h>
#include <stdlib.h>
#include <mutex>
#include <thread>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <jsoncpp/json/json.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h> 
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <pcl/filters/passthrough.h> 
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <pcl/common/intersections.h>
#include <pcl/common/transforms.h>
#include <pcl/common/impl/intersections.hpp>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/registration/gicp.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <pcl_ros/transforms.h>
#include "perception_storage/common.hpp"
#include "perception_storage/tool_box.hpp"
#include "perception_storage/points_filter.hpp"
#include "perception_storage/pose_tracker/nano_gicp/nano_gicp.hpp"

namespace pose_tracker
{
  struct tracking_target
  {
    tracking_target():side_info("NULL")
    {

    }
    
    tracking_target(geometry_msgs::PoseStamped reference_pose,geometry_msgs::PoseStamped pose):target_pose(pose)
    {
      side_info=(reference_pose.pose.position.y>0)?"left":"right";
      transform_target=Eigen::Matrix4f::Identity();
      Eigen::Quaternionf _q_(target_pose.pose.orientation.w,
                             target_pose.pose.orientation.x,
                             target_pose.pose.orientation.y,
                             target_pose.pose.orientation.z);
      Eigen::Vector3f _t_(target_pose.pose.position.x,
                          target_pose.pose.position.y,
                          target_pose.pose.position.z);
      transform_target.block<3,3>(0,0)=_q_.toRotationMatrix();
      transform_target.block<3,1>(0,3)=_t_;
    }

    std::string side_info;
    geometry_msgs::PoseStamped target_pose;
    Eigen::Matrix4f transform_target;
  };
  
  struct tracking_state
  {
    tracking_state()
    {

    }

    tracking_state(geometry_msgs::PoseStamped _kinematic_pose_,geometry_msgs::PoseStamped _update_pose_,
                   tf::Pose _inverse_pose_,bool _is_tracked_):
                   kinematic_pose(_kinematic_pose_),update_pose(_update_pose_),inverse_pose(_inverse_pose_),is_tracked(_is_tracked_)
    {

    }

    geometry_msgs::PoseStamped kinematic_pose;
    geometry_msgs::PoseStamped update_pose;
    tf::Pose inverse_pose;
    bool is_tracked=false;
  };
  
  struct tracking_trajectory
  {
    tracking_trajectory()
    {

    }

    tracking_trajectory(tracking_target _target_data_):target_data(_target_data_)
    {
      
    }
    
    tracking_target target_data;
    std::vector<tracking_state> state_array;
  };

  struct sensor_measurement
  {
    sensor_measurement()
    {

    }

    sensor_measurement(const sensor_msgs::PointCloud2ConstPtr &_points_,geometry_msgs::PoseStamped &_kinematic_pose_,Eigen::Matrix4d &_trans_mat_):
                       kinematic_pose(_kinematic_pose_)
    {
      point_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*_points_, *point_cloud);
      pcl::transformPointCloud(*point_cloud,*point_cloud,_trans_mat_);
      header=_points_->header;
      header.frame_id="base_link";
    }

    std_msgs::Header header;
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud;
    geometry_msgs::PoseStamped kinematic_pose;
  };
  
  //ax+by+1=0,垂直向量
  struct LineFunctor1
  {
    LineFunctor1(double x, double y):x_(x), y_(y)
    {}

    template <typename T>
    bool operator()(const T* const params, T* residual) const
    {
      residual[0] = params[0] * T(x_) + params[1] * T(y_) + T(1.);
      return true;
    }

    private:
      const double x_, y_;
  };

  struct LineFunctor2
  {
    LineFunctor2(double x, double y):x_(x), y_(y)
    {}

    template <typename T>
    bool operator()(const T* const params, T* residual) const
    {
      residual[0] = params[2] * T(x_) + params[3] * T(y_) + T(1.);
      return true;
    }

    private:
      const double x_, y_;
  };

  struct PerpendicularConstraint
  {
    template <typename T>
    bool operator()(const T* const params, T* residual) const
    {
      residual[0] = params[0] * params[2] + params[1] * params[3];
      return true;
    }
  };

  class pose_tracker
  {
    public:
      pose_tracker(ros::NodeHandle& pnh,tf::TransformListener *_tf_);
      void sensor_cbk(const sensor_msgs::PointCloud2ConstPtr &msg,std::string side_info);
      void cmd_cbk(const geometry_msgs::PoseStamped &msg);
      bool trans_pose_world(geometry_msgs::PoseStamped &pose_in,geometry_msgs::PoseStamped &pose_out,
                            bool base,std::string target_name);
      bool trans_points_world(sensor_measurement &measurement);
      void track();
      bool tracking_pose_init(geometry_msgs::PoseStamped &reference_target,geometry_msgs::PoseStamped &pose_target);
      bool detection(tracking_trajectory &trajectory_data,pcl::PointCloud<pcl::PointXYZI>::Ptr &points_ptr,
                     std_msgs::Header &header,geometry_msgs::Pose &robot_pose,geometry_msgs::Pose &target_pose,int &model_selected);
      void tracking_pose(tracking_trajectory &trajectory_data,std::deque<sensor_measurement> &process_measurement);
      
    private:
      ros::NodeHandle nh_;
      ros::Subscriber left_subscriber;
      ros::Subscriber right_subscriber;
      ros::Subscriber cmd_subscriber;
      ros::Publisher pub_reference_pose,pub_pose,pub_pose_robot;
      ros::Publisher pub_points,pub_target_points;
      ros::Publisher pub_tracker_state;
      points_filter filter_tools;
      ConfigInfo base_config;
      tf::TransformListener *tf;
      tracking_trajectory tracking_info;
      int lost_flag;
      double tunnel_width;
      double tunnel_thickness;
      int frames_num;
      std::deque<sensor_measurement> sensor_buffer;
      std::thread track_thread;
      std::mutex cmd_mtx;
      std::map<std::string,std::string> side_table;
      Eigen::Matrix4d transform_right=Eigen::Matrix4d::Identity();
      Eigen::Matrix4d transform_left=Eigen::Matrix4d::Identity();
      std::map<std::string,Eigen::Matrix4d> transform_table;
      std::string world_frame;
      bool using_local_map;
      float stop_distance;

      void set_config(ConfigInfo &config_info);
      void set_trajectory(tracking_trajectory & trajectory_data);
      bool get_trajectory(tracking_trajectory &trajectory_data);
      bool get_measurement(std::deque<sensor_measurement> &sensor_data);
  };
 
};

#endif
