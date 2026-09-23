#ifndef ANALYZER_HPP
#define ANALYZER_HPP
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
#include <boost/filesystem.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/plot.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <ros/package.h>
#include "perception_storage/common.hpp"
using namespace boost::gregorian;
using namespace boost::posix_time;
class analyzer
{
  public:
    analyzer(ConfigInfo &config_info,int &task,int &job);
    void writer_job(cv::Mat &depth,pcl::PointCloud<pcl::PointXYZI>::Ptr &points,int flag);
    TaskInfo writer_task(std::string &sensor,std::string &side);
    void update(std::tuple<Eigen::Vector2d,Eigen::Vector3i,Eigen::Vector3f> &data,int &flag);
    void update_time(Eigen::Vector2d &time_v);
    void update_index(Eigen::Vector3i &index_v);
    void update_pose(Eigen::Vector3f &pose_v);
    bool manager();

  private:
    bool enable_analyzer;
    int sliding_window;
    int valid_task;
    int valid_job;
    bool plotter(std::vector<double> &values);
    
    std::string shooter_dir;
    std::string current_exp;
    std::deque<Eigen::Vector3f> pose_que;
    Eigen::Vector3i valid_index;
    Eigen::Vector2d time_cost;
    int job_num;
};

#endif
