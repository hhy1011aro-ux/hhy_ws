#ifndef DETECTOR_BASE_HPP
#define DETECTOR_BASE_HPP

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
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/StdVector>
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
#include <pcl/features/integral_image_normal.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/common/common.h>
#include <pcl/common/intersections.h>
#include <pcl/common/transforms.h>
#include <pcl/common/impl/intersections.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include "perception_storage/common.hpp"
#include "perception_storage/tool_box.hpp"
#include "perception_storage/depth_runner/depth_elsed.h"
#include "perception_storage/depth_runner/depth_segmentor.hpp"
#include "perception_storage/points_filter.hpp"

class detector_base
{
  public:
    virtual ~detector_base()=default;
    virtual bool inference(depth_mapping &data,std_msgs::Header &header_info)=0;
    virtual std::pair<cv::Mat,std::vector<intermediate>> get_intermediate()=0;
    virtual std::vector<mpose> get_targets()=0;
    virtual std::vector<mpose> get_result()=0;
    // roi可视化
    visualization_msgs::MarkerArray roi_region;
};

#endif