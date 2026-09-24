#ifndef DETECTOR_HPP
#define DETECTOR_HPP

#include "perception_storage/detector_base.hpp"

// 采用itof检测
class tof_detector: public detector_base
{
  public:
    tof_detector(ConfigInfo &config_);
    ~tof_detector();
    // 推理
    bool inference(depth_mapping &data,std_msgs::Header &header_info) override;
    std::pair<cv::Mat,std::vector<intermediate>> get_intermediate() override;
    std::vector<mpose> get_targets() override;
    std::vector<mpose> get_result() override;
   private:
    SensorInfo lidarinfo;
    TargetInfo targetinfo;
    SettingInfo settinginfo;
    // 深度图分辨率
    float depth_resolution;
    std::vector<Eigen::Vector3f> region_vertex;
    std::pair<cv::Mat,std::vector<intermediate>> interm_result;
    std::vector<mpose> detect_targets;
    std::vector<mpose> detect_result;

    std::shared_ptr<depth_segmentor> segmentor_machine=nullptr;
    std::shared_ptr<points_filter> filter_machine=nullptr;
    //
    void set_roi(std_msgs::Header &header_info);
    // 滤波
    bool base_filter(intermediate &target);
    bool support_filter(intermediate &target,intermediate &support_target);
    bool refine_filter(mpose &target,pcl::PointCloud<pcl::PointXYZI>::Ptr &cues,mpose &result);
};

// 采用lidar检测
class lidar_detector: public detector_base
{
  public:
    lidar_detector(ConfigInfo &config_);
    ~lidar_detector();
    // 推理
    bool inference(depth_mapping &data,std_msgs::Header &header_info) override;
    std::pair<cv::Mat,std::vector<intermediate>> get_intermediate() override;
    std::vector<mpose> get_targets() override;
    std::vector<mpose> get_result() override;
  private:
    SensorInfo lidarinfo;
    TargetInfo targetinfo;
    SettingInfo settinginfo;
    // 深度图分辨率
    float depth_resolution;
    float free_distance;
    //
    std::vector<Eigen::Vector3f> region_vertex;
    std::pair<cv::Mat,std::vector<intermediate>> interm_result;
    std::vector<mpose> detect_targets;
    std::vector<mpose> detect_result;

    // 稳定性诊断：分别发布精调前原始点和XOY投影点，intensity用于区分标签、长边和短边。
    ros::Publisher pub_refine_raw_points;
    ros::Publisher pub_refine_projected_points;
    // 调试点云沿用当前输入帧的时间戳和坐标系，便于与最终结果逐帧关联。
    std_msgs::Header refine_debug_header;

    std::shared_ptr<depth_segmentor> segmentor_machine=nullptr;
    std::shared_ptr<points_filter> filter_machine=nullptr;
    // 几何判据
    bool geomery_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr &patch);
    // 调整判断
    bool refine_filter(mpose &target,pcl::PointCloud<pcl::PointXYZI>::Ptr &cues,mpose &result);
    // 边界优化；tag_candidates保留完整标签候选，用于计算稳健的Q2/Q98端点中点。
    bool finetuning_filter(Eigen::VectorXf &short_coef_,pcl::PointCloud<pcl::PointXYZI>::Ptr &short_line,Eigen::Vector4f &short_direction,
                          Eigen::VectorXf &large_coef_,pcl::PointCloud<pcl::PointXYZI>::Ptr &large_line,Eigen::Vector4f &large_direction,
                          Eigen::VectorXf &tag_coef_,pcl::PointCloud<pcl::PointXYZI>::Ptr &tag_candidates,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr &tag_line,Eigen::Vector4f &tag_direction,
                          int &flag_index,mpose &base_pose,mpose &refine_pose);
    // roi可视化
    void set_roi(std_msgs::Header &header_info);
};

#endif
