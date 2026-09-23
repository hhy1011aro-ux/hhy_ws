#ifndef FILTER_HPP
#define FILTER_HPP

#include <string>
#include <iostream>
#include <map>
#include <algorithm>
#include <fstream>
#include <vector>
#include <queue> 
#include <tuple>
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
#include <pcl/keypoints/uniform_sampling.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/tf.h>
#include <pcl_ros/transforms.h>
#include "perception_storage/common.hpp"

class points_filter
{
  public:
    points_filter();
    void norm_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr & source_points,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr & target_points,
                     float threshold,Eigen::Vector3f reference_dir);
    void sampling_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr & source_points,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr & target_points,
                         std::string method,float scale);                 
    std::tuple<pcl::PointCloud<pcl::PointXYZI>::Ptr,
               pcl::PointIndices::Ptr,
               pcl::ModelCoefficients::Ptr> naive_plane(pcl::PointCloud<pcl::PointXYZI>::Ptr & raw_points,float threshold);
    std::tuple<pcl::PointCloud<pcl::PointXYZI>::Ptr,
               std::vector<int>,
               Eigen::VectorXf> naive_line(pcl::PointCloud<pcl::PointXYZI>::Ptr & raw_points,float threshold);
    std::tuple<Eigen::Vector3f,Eigen::Vector3f> target_orient(const TargetOrient orient_method,pcl::PointCloud<pcl::PointXYZI>::Ptr & target_points);
};

#endif
