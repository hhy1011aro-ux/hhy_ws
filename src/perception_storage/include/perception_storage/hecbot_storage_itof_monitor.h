/**
 * 基于itof传感器，spider相对货架位置监控功能
*/

#ifndef hecbot_STORAGE_ITOF_H_
#define hecbot_STORAGE_ITOT_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Transform.h>
#include <jsoncpp/json/json.h> 
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/String.h"
// #include <hecbot_perception_msgs/ChargerInfo.h>
// #include <hecbot_perception_msgs/ChargerIntegrateInfo.h>
// #include <hecbot_perception_msgs/ChargeDetection.h>
// #include <hecbot_perception_msgs/ChargeDetectionRequest.h>
// #include <hecbot_perception_msgs/ChargeDetectionResponse.h>
// #include <hecbot_msgs/Point.h>
#include <hecbot_srvs/JString.h>
#include <hecbot_srvs/JStringRequest.h>
#include <hecbot_srvs/JStringResponse.h>
#include <thread>
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;

class StorageItof
{
public:
    struct avg_filter
    {
        double x, y, th;
    };
    StorageItof();
    ~StorageItof();
    //参考点
    // bool setReferance(tf::Stamped<tf::Pose> &referance);  
    //坐标系转换  
    
    void coordinateTransform(const std::string &target_frame, const std::string &source_frame, const tf::TransformListener &listener, tf::StampedTransform &transform);
    //统计滤波，过滤立群点
    PointCloudT::Ptr stasticFilter(const PointCloudT::Ptr &cloud_in, const PointCloudT::Ptr &cloud_out);
    //提取感兴趣区域
    PointCloudT::Ptr extractROI(const PointCloudT::Ptr &cloud_in);
    PointCloudT::Ptr extract_horizon_line(const PointCloudT::Ptr &cloud_in);
    PointCloudT::Ptr extract_vertical_line(const PointCloudT::Ptr &cloud_in);
    PointCloudT::Ptr extractStorageleg(const PointCloudT::Ptr &cloud_in);
    double averageZ(const PointCloudT::Ptr &cloud_in);
    double ROI_left_right(const PointCloudT::Ptr &cloud_in);
    double maxZ(const PointCloudT::Ptr &cloud_in);
    double maxZ(const PointCloudT::Ptr &cloud_in,const PointCloudT::Ptr &cloud_in1);
    double minZ(const PointCloudT::Ptr &cloud_in);
    double minZ(const PointCloudT::Ptr &cloud_in,const PointCloudT::Ptr &cloud_in1);
    
    //tf矩阵转成变换矩阵形式
    Eigen::Matrix4f transformTf(const tf::StampedTransform& transform); 
    //点云发布函数
    void cloudptrPub(PointCloudT::Ptr cloud_in, ros::Publisher &pub);  
    //点云的回调函数
    void cloudleftCallback(const sensor_msgs::PointCloud2::ConstPtr &msgs); 
    void cloudrightCallback(const sensor_msgs::PointCloud2::ConstPtr &msgs);
    void storageCallback(const std_msgs::String::ConstPtr &_str);
    void poseCallback_(const geometry_msgs::PoseStamped pose);
    void testcallback(const sensor_msgs::PointCloud2::ConstPtr &msgs);//20231027
    void process();
    void cal_thread_func();
    double four_point_z_cal(const PointCloudT::Ptr &cloud_in,double range);
    double projection_cal(const PointCloudT::Ptr &cloud_in,double range);
    PointCloudT::Ptr extract_part_ROI(const PointCloudT::Ptr &cloud_in,double x_in,double y_in);
    PointCloudT::Ptr extractROI_center(const PointCloudT::Ptr &cloud_in);
    int storage_monitor;
    double right_th_max,right_th_min,left_th_max,left_th_min;
    
private:
    ros::Subscriber cloud_left_sub_,cloud_right_sub_;         //点云订阅方
    ros::Subscriber pose_sub_;         
    ros::Publisher itof_forward_pub;
    ros::Subscriber storage_sub_;  
    ros::Subscriber test_sub_;//20231027
    
    ros::Publisher ori_pub_,horizon_pub,vertical_pub,collision_pub_;
    
    ros::Publisher base_link_pub_;   //充电桩平面  
    tf::TransformListener tf_;  //tf坐标变换监听器
    tf::TransformListener listener_base_link2itof_right,listener_base_link2itof_left;  //
    tf::TransformListener listener_itof_left2base_link,listener_itof_right2base_link;  //itof->base_link                      
    std::string itof_frame_left,itof_frame_right;               //itof的坐标名字
    std::string cloud_topic_left,cloud_topic_right;            //itof话题

    double point_in_x, point_in_y,point_in_z;  //输入参数确定ROI区域
    double z_left, z_right,x_min,x_max,y_min,y_max;//20230811
    double x_mid,y_mid,z_mid;
    bool begin_collision_detection;//20230804
    geometry_msgs::PoseStamped pose_center;//20230811
    geometry_msgs::PoseStamped pose_center_in_BL;//20230811
    geometry_msgs::PoseStamped pose_center_in_itof;//20230811
    bool need_collision_detect_=false;//20230809
    bool first_time=true;//20230811
    double board_length = 0.210;
    double board_height = 0.230;
    geometry_msgs::PoseStamped pose_;//20230811
    double x_range_min,x_range_max,th_range_min,th_range_max;
    std::string frame_id_,state_selection;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ori;    //原始点云
    double z_avg;
    int cnt;

    tf::StampedTransform transform_itof_left2base_link; 
    tf::StampedTransform transform_itof_right2base_link; 
    tf::StampedTransform transform_base_link2itof_right; 
    tf::StampedTransform transform_base_link2itof_left;
};

#endif
