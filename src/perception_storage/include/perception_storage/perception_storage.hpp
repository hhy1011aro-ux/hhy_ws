#ifndef PERCEPTION_STORAGE_HPP
#define PERCEPTION_STORAGE_HPP

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
#include <pcl_ros/transforms.h>

#include <hecbot_srvs/JString.h>
#include <hecbot_srvs/JStringRequest.h>
#include <hecbot_srvs/JStringResponse.h>
#include "perception_storage/common.hpp"
#include "perception_storage/detector.hpp"
#include "perception_storage/points_filter.hpp"
#include "perception_storage/analyzer.hpp"

struct coord
{
  int phi;
  int theta;

  coord(Eigen::Vector3f point,float step)
  {
    phi=floor(atan2(point.y(),point.x())/step);
    theta=floor(atan2(point.z(),point.head(2).norm())/step);
  }

  bool operator<(const coord& rhs) const
  {
    if (phi != rhs.phi) 
      return phi < rhs.phi;
    return theta < rhs.theta;
  }
  
  bool operator==(const coord& rhs) const
  {
    return phi == rhs.phi && theta == rhs.theta;
  }
};

struct compare_point
{
  int index;
  float range;
  pcl::PointXYZI point;

  compare_point()
  {

  }
  compare_point(int _index_,pcl::PointXYZI _point_):index(_index_),point(_point_)
  {
    range=Eigen::Vector3f(_point_.x,_point_.y,_point_.z).norm();
  }
  compare_point(pcl::PointXYZI _point_):point(_point_)
  {
    range=fabs(_point_.y);
  }
  compare_point(pcl::PointXYZI _point_,bool using_theta):point(_point_)
  {
    if(using_theta)
    {
      range=atan2(point.x,fabs(point.y));
    }
  }
};

bool compere_func(compare_point a,compare_point b)
{
  return a.range<b.range;
};

namespace std
{
  template <> 
  struct hash<coord>
  {
    size_t operator()(const coord& c) const
    {
      return hash<int>()(c.phi) ^ hash<int>()(c.theta); 
    }
  }; 
}

class PerceptionStorage
{
  public:
    PerceptionStorage(ros::NodeHandle& pnh,ros::NodeHandle& nh);
    ~PerceptionStorage();
    // 初始化
    void init();
    // 读取参数
    bool read_params();
    // 配置订阅
    void set_processor();
    // 配置发布
    void set_publishers();
    // 添加到buffer
    void insert_buffer(MsgPacket &msg);
    // 从buffer获取数据
    void parse_data(std::deque<MsgPacket> &msgs);
    // 获得一帧待处理数据
    bool parse_frame(std::deque<MsgPacket> &msgs);
    // 深度图与对应的结构化点云
    depth_mapping couple_data(std::deque<MsgPacket> &multiframes);
    depth_mapping couple_lidar(std::deque<MsgPacket> &multiframes);
    depth_mapping couple_tof(std::deque<MsgPacket> &multiframes);
    // 开始一个工作周期
    void start(std::string &side_,std::vector<float> &pose_);
    // 结束一个工作周期
    void stop();
    // 自己判断停止
    bool stop_self();
    // 有数据就进行检测操作
    void running();
    // 发布
    void publish_msgs(std::pair<cv::Mat,std::vector<intermediate>> &interm_info,
                      std::vector<mpose> &target_info,std::vector<mpose> &result_info,
                      std_msgs::Header &header_info);
    // 中间线索的展示
    void provide_evidence(std::pair<cv::Mat,std::vector<intermediate>> &interm_info,
                         std::vector<mpose> &target_info,std::vector<mpose> &result_info,
                         std_msgs::Header &header_info);
    // 后处理
    void post_process();
    // 发布检测结果
    void send_result(mpose & target_pose,std_msgs::Header &header_info);
    // 发布事件
    void tracker_cbk(const std_msgs::String &msg);
    void send_event(std::string &sensor,std::string &side);

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle _nh_;
    std::mutex _mtx_;
    // 参数配置
    ConfigInfo config_info;
    std::string sensor_type_str;
    // 激活控制量
    ActiveState active_;
    // 一次工作请求信息
    ReqInfo req_info_;
    // 一次激活任务后的工作次数
    int job_num;
    // 当前任务传感器数据header
    std_msgs::Header job_header;
    // 静态变换
    std::string frame_front,frame_rear,frame_right,frame_left;
    Eigen::Matrix4d matrix_left;
    Eigen::Matrix4d matrix_right;
    Eigen::Matrix4d matrix_base;
    // 计数标志位
    int valid_hys=0;
    int valid_re=0;
    // 服务
    ros::ServiceServer perception_srv_;
    ros::ServiceClient event_client_;
    // tof
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                            sensor_msgs::Image> SyncPolicy_tof;
    typedef message_filters::Synchronizer<SyncPolicy_tof> tof_sync;
    std::shared_ptr<tof_sync> sync_tof_left;
    std::shared_ptr<tof_sync> sync_tof_right;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_tof_points_left;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_tof_points_right;
    message_filters::Subscriber<sensor_msgs::Image> sub_tof_gray_left;
    message_filters::Subscriber<sensor_msgs::Image> sub_tof_gray_right;
    ros::Subscriber cam_info_left,cam_info_right;
    bool cam_left_ready,cam_right_ready;
    CameraInfo tof_cam_left,tof_cam_right;
    // lidar
    ros::Subscriber sub_livox_left,sub_livox_right;
    //
    ros::Subscriber sub_tracker_state;
    // filter
    std::shared_ptr<points_filter> filter_machine=nullptr;
    // tf订阅
    tf::TransformListener tf_listener;
    // 发布
    ros::Publisher pub_result;
    ros::Publisher pub_reference;
    ros::Publisher pub_refer_region;
    ros::Publisher pub_naive_region;
    ros::Publisher pub_base_points;
    ros::Publisher pub_depth_detect;
    ros::Publisher pub_hypothesis_points;
    ros::Publisher pub_hypothesis_pose;
    ros::Publisher pub_evidence_points;
    ros::Publisher pub_evidence_pose;
    ros::Publisher pub_remember_pose;
    ros::Publisher pub_repub_points;
    ros::Publisher pub_repub_image;
    ros::Publisher pub_lidar_capture;
    // 参考短线拟合前的完整聚类点，用于分析点带厚度、密度和离群点形态。
    ros::Publisher pub_reference_cloud_cluster;
    ros::Publisher pub_event;
    // 本次任务的姿态记录
    geometry_msgs::PoseArray task_remember_pose;
    // 消息队列
    std::deque<MsgPacket> msg_packet;
    std::deque<MsgPacket> process_queue;
    // 线程队列
    std::vector<std::shared_ptr<boost::thread> > threads_pool;
    // 检测器
    std::shared_ptr<detector_base> detect_machine;
    // 记录器
    std::shared_ptr<analyzer> analyzer_machine=nullptr;
    // 相机内参订阅
    void cam_callback(const sensor_msgs::CameraInfo &msg,std::string &left_or_right);
    // 订阅tof数据
    void process_tof(const sensor_msgs::PointCloud2ConstPtr &points,
                     const sensor_msgs::ImageConstPtr &image,
                     std::string &which_side);
    // 订阅lidar点云
    void process_lidar(const sensor_msgs::PointCloud2ConstPtr &points,
                       std::string &which_side);
    // 服务接口
    bool perception_service(hecbot_srvs::JString::Request &req, hecbot_srvs::JString::Response &res);
};

#endif
