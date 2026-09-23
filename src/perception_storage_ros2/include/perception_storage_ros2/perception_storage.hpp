#ifndef PERCEPTION_STORAGE_HPP
#define PERCEPTION_STORAGE_HPP

#include <algorithm>
#include <atomic>
#include <thread>
#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h> 
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <pcl/filters/passthrough.h> 
#include <pcl/common/common.h>
#include <pcl/common/intersections.h>
#include <pcl/common/transforms.h>
#include <pcl/common/impl/intersections.hpp>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "perception_storage_ros2/common.hpp"
#include "perception_storage_ros2/detector.hpp"
#include "perception_storage_ros2/points_filter.hpp"

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

inline bool compere_func(compare_point a,compare_point b)
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

class PerceptionStorage : public rclcpp::Node
{
  public:
    PerceptionStorage();
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
                      std_msgs::msg::Header &header_info);
    // 中间线索的展示
    void provide_evidence(std::pair<cv::Mat,std::vector<intermediate>> &interm_info,
                         std::vector<mpose> &target_info,std::vector<mpose> &result_info,
                         std_msgs::msg::Header &header_info);
    // 后处理
    void post_process();
    // 发布检测结果
    void send_result(mpose & target_pose,std_msgs::msg::Header &header_info);

  private:
    std::mutex _mtx_;
    // 参数配置
    ConfigInfo config_info;
    std::string sensor_type_str;
    bool auto_start;
    std::string sensor_side;
    std::string control_name;
    // 激活控制量
    std::atomic<ActiveState> active_;
    // 一次激活任务后的工作次数
    int job_num;
    // 当前任务传感器数据header
    std_msgs::msg::Header job_header;
    // 计数标志位
    int valid_hys=0;
    int valid_re=0;
    // tof
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2,
                                                            sensor_msgs::msg::Image> SyncPolicy_tof;
    typedef message_filters::Synchronizer<SyncPolicy_tof> tof_sync;
    std::shared_ptr<tof_sync> sync_tof_left;
    std::shared_ptr<tof_sync> sync_tof_right;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_tof_points_left;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_tof_points_right;
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_tof_gray_left;
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_tof_gray_right;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_left,cam_info_right;
    bool cam_left_ready,cam_right_ready;
    CameraInfo tof_cam_left,tof_cam_right;
    // lidar
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_livox_left,sub_livox_right;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_control;
    // filter
    std::shared_ptr<points_filter> filter_machine=nullptr;
    // 发布
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_result;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_naive_region;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_base_points;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_detect;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_hypothesis_points;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_hypothesis_pose;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_evidence_points;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_evidence_pose;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_remember_pose;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_repub_points;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_repub_image;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_capture;
    // 本次任务的姿态记录
    geometry_msgs::msg::PoseArray task_remember_pose;
    // 消息队列
    std::deque<MsgPacket> msg_packet;
    std::deque<MsgPacket> process_queue;
    // 线程队列
    std::vector<std::shared_ptr<std::thread> > threads_pool;
    std::atomic<bool> worker_running{false};
    // 检测器
    std::shared_ptr<detector_base> detect_machine;
    // 相机内参订阅
    void cam_callback(const sensor_msgs::msg::CameraInfo &msg,const std::string &left_or_right);
    // 订阅tof数据
    void process_tof(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &points,
                     const sensor_msgs::msg::Image::ConstSharedPtr &image,
                     const std::string &which_side);
    // 订阅lidar点云
    void process_lidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &points,
                       const std::string &which_side);
    void control_callback(const std_msgs::msg::String::ConstSharedPtr msg);
};

#endif
