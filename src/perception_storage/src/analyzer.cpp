#include "perception_storage/analyzer.hpp"
#include <stdexcept>

analyzer::analyzer(ConfigInfo &config_info,int &task,int &job):valid_task(task),valid_job(job)
{
  enable_analyzer=config_info.m_settinginfo.trouble_shoting;
  sliding_window=config_info.m_settinginfo.batch_size;
  const char *home_dir=getenv("HOME");
  if(home_dir==nullptr || home_dir[0]=='\0')
  {
    throw std::runtime_error("HOME is not set; cannot create analyzer output directory");
  }
  shooter_dir=(boost::filesystem::path(home_dir)/".ros"/"analyzers").string();
  if(!boost::filesystem::is_directory(shooter_dir))
  {
    boost::filesystem::create_directories(shooter_dir);
  }
  pose_que.clear();
  valid_index=Eigen::Vector3i();
  time_cost=Eigen::Vector2d();
  job_num=0;
}

void analyzer::writer_job(cv::Mat &depth,pcl::PointCloud<pcl::PointXYZI>::Ptr &points,int flag)
{
  if(!enable_analyzer) return;
  if(!boost::filesystem::is_directory(current_exp)) return;
  std::string job_root=current_exp+"/"+"jobs";
  std::string job_exp=current_exp+"/"+"jobs"+"/"+std::to_string(job_num);
  if(!boost::filesystem::is_directory(job_root))
  {
    boost::filesystem::create_directory(job_root);
  }
  if(!boost::filesystem::is_directory(job_exp))
  {
    boost::filesystem::create_directory(job_exp);
  }
  if(flag==0)
  {
    std::string depth_name=job_exp+"/"+"source.jpg";
    cv::imwrite(depth_name,depth);
    std::string points_name=job_exp+"/"+"source.pcd";
    if(points->points.size()>0)
    {
      points->height=1;
      points->width=points->points.size();
      pcl::io::savePCDFileASCII(points_name,*points);
    }
  }
  else
  {
    std::string depth_name=job_exp+"/"+"result.jpg";
    cv::imwrite(depth_name,depth);
    std::string points_name=job_exp+"/"+"result.pcd";
    if(points->points.size()>0)
    {
      points->height=1;
      points->width=points->points.size();
      pcl::io::savePCDFileASCII(points_name,*points);
    }
  }
}

TaskInfo analyzer::writer_task(std::string &sensor,std::string &side)
{
  TaskInfo task_msg;
  if(!enable_analyzer) return task_msg;
  std::vector<double> values;
  plotter(values);
  std::string task_info=current_exp+"/"+"task_info.yaml";
  std::ofstream task_file(task_info);
  YAML::Node info_sensor;
  info_sensor["sensor"]=sensor;
  task_file<<info_sensor<<std::endl;
  YAML::Node info_side;
  info_side["side"]=side;
  task_file<<info_side<<std::endl;
  YAML::Node window;
  window["sliding_window"]=sliding_window;
  task_file<<window<<std::endl;
  YAML::Node info_infer;
  info_infer["infer_n"]=valid_index[0];
  task_file<<info_infer<<std::endl;
  YAML::Node info_hype;
  info_hype["hyp_n"]=valid_index[1];
  task_file<<info_hype<<std::endl;
  YAML::Node info_result;
  info_result["reslut_n"]=valid_index[2];
  task_file<<info_result<<std::endl;
  YAML::Node info_precost;
  info_precost["max_pre_cost"]=time_cost[0];
  task_file<<info_precost<<std::endl;
  YAML::Node info_infercost;
  info_infercost["max_infer_cost"]=time_cost[1];
  task_file<<info_infercost<<std::endl;
  if(values.size()==6)
  {
    YAML::Node yaw_value_mean;
    yaw_value_mean["yaw_mean"]=values[0];
    task_file<<yaw_value_mean<<std::endl;
    YAML::Node yaw_value_std;
    yaw_value_std["yaw_stdev"]=values[1];
    task_file<<yaw_value_std<<std::endl;
    YAML::Node x_value_mean;
    x_value_mean["x_mean"]=values[2];
    task_file<<x_value_mean<<std::endl;
    YAML::Node x_value_std;
    x_value_std["x_stdev"]=values[3];
    task_file<<x_value_std<<std::endl;
    YAML::Node y_value_mean;
    y_value_mean["y_mean"]=values[4];
    task_file<<y_value_mean<<std::endl;
    YAML::Node y_value_std;
    y_value_std["y_stdev"]=values[5];
    task_file<<y_value_std<<std::endl;
  }
  task_file.close();
  if(valid_index[2]>0)
  {
    boost::filesystem::remove_all(current_exp);
  }
  task_msg=TaskInfo(sensor,side,valid_index[0],valid_index[1],valid_index[2]);
  return task_msg;
}

void analyzer::update(std::tuple<Eigen::Vector2d,Eigen::Vector3i,Eigen::Vector3f> &data,int &flag)
{
  if(!enable_analyzer) return;
  switch(flag)
  {
    case 0:
    {
      pose_que.clear();
      valid_index=Eigen::Vector3i();
      time_cost=Eigen::Vector2d();
      job_num=0;
      break;
    }
    case 1:
    {
      auto time_v=std::get<0>(data);
      update_time(time_v);
      break;
    }
    case 2:
    {
      auto index_v=std::get<1>(data);
      update_index(index_v);
      break;
    }
    case 3:
    {
      auto pose_v=std::get<2>(data);
      update_pose(pose_v);
      break;
    }
    default:
      break;
  }
}

void analyzer::update_time(Eigen::Vector2d &time_v)
{
  time_cost[0]=std::max<double>(time_cost[0],time_v[0]);
  time_cost[1]=std::max<double>(time_cost[1],time_v[1]);
  ++job_num;
}

void analyzer::update_index(Eigen::Vector3i &index_v)
{
  valid_index=index_v;
}

void analyzer::update_pose(Eigen::Vector3f &pose_v)
{
  pose_que.emplace_back(pose_v);
  if(pose_que.size()>valid_job) pose_que.pop_front();
}

bool analyzer::manager()
{
  if(!enable_analyzer) return false;
  boost::filesystem::path src_path(shooter_dir);
  std::vector<int> dir_list;
  std::vector<std::string> dir_str_list;
  for(const auto &dir:boost::filesystem::directory_iterator(src_path))
  {
    if(boost::filesystem::is_directory(dir))
    {
      boost::filesystem::path path_(dir.path());
      auto name_str=path_.stem().string();
      dir_str_list.emplace_back(name_str);
      auto name_=atoi(path_.stem().string().c_str());
      dir_list.emplace_back(name_);
    }
  }
  auto now = std::chrono::system_clock::now();
  auto time=std::chrono::system_clock::to_time_t(now);
  char _time[19] = {0};
  struct tm local_time;
  localtime_r(&time, &local_time);
  strftime(_time, 22, "%m%d%H%M%S", &local_time);
  auto time_table=std::string(_time);
  current_exp=shooter_dir+"/"+time_table;
  boost::filesystem::create_directory(current_exp);
  if(dir_list.size()>=valid_task)
  {
    int oldest_name=*std::min_element(dir_list.begin(),dir_list.end());
    int oldest_index=std::min_element(dir_list.begin(),dir_list.end())-dir_list.begin();
    std::string delete_dir=shooter_dir+"/"+dir_str_list[oldest_index];
    boost::filesystem::remove_all(delete_dir);
  }
  pose_que.clear();
  valid_index=Eigen::Vector3i();
  time_cost=Eigen::Vector2d();
  job_num=0;
}

bool analyzer::plotter(std::vector<double> &values)
{
  if(pose_que.size()<=1) return false;
  std::vector<double> pose_yaw,pose_x,pose_y;
  for(auto & pose:pose_que)
  {
    pose_yaw.emplace_back(double(pose[0]));
    pose_x.emplace_back(double(pose[1]));
    pose_y.emplace_back(double(pose[2]));
  }
  // yaw
  double yaw_sum=std::accumulate(std::begin(pose_yaw),std::end(pose_yaw),0.0);
	double yaw_mean=yaw_sum/pose_yaw.size();
	double yaw_accum=0.0;
	std::for_each(std::begin(pose_yaw),std::end(pose_yaw),[&](const double d)
  {
		yaw_accum+=(d-yaw_mean)*(d-yaw_mean);
	});
	double yaw_stdev = sqrt(yaw_accum/(pose_yaw.size()-1));
  // x
  double x_sum=std::accumulate(std::begin(pose_x),std::end(pose_x),0.0);
	double x_mean=x_sum/pose_x.size();
	double x_accum=0.0;
	std::for_each(std::begin(pose_x),std::end(pose_x),[&](const double d)
  {
		x_accum+=(d-x_mean)*(d-x_mean);
	});
	double x_stdev = sqrt(x_accum/(pose_x.size()-1));
  // y
  double y_sum=std::accumulate(std::begin(pose_y),std::end(pose_y),0.0);
	double y_mean=y_sum/pose_y.size();
	double y_accum=0.0;
	std::for_each(std::begin(pose_y),std::end(pose_y),[&](const double d)
  {
		y_accum+=(d-y_mean)*(d-y_mean);
	});
	double y_stdev = sqrt(y_accum/(pose_y.size()-1));
  values={yaw_mean,yaw_stdev,x_mean,x_stdev,y_mean,y_stdev};
  int fontFace = cv::FONT_HERSHEY_SIMPLEX;
  double fontScale = 0.5;
  int thickness = 1;
  // yaw
  double yaw_min=*std::min_element(pose_yaw.begin(),pose_yaw.end());
  double yaw_max=*std::max_element(pose_yaw.begin(),pose_yaw.end());
  float yaw_res=0.03;
  int yaw_ceils=ceil((yaw_max-yaw_min)/yaw_res);
  if(yaw_ceils>200) yaw_res=(yaw_max-yaw_min)/200.0;
  double yaw_mid=(yaw_max+yaw_min)/2.0;
  cv::Mat yaw_figure=cv::Mat(250,500,CV_8UC3,cv::Scalar(125,125,125));
  for(int k=0;k<pose_yaw.size();k++)
  {
    cv::Point2f p1((k+1)*4.0,125);
    float p2_y=125-(pose_yaw[k]-yaw_mean)/yaw_res;
    cv::Point2f p2((k+1)*4.0,p2_y);
    cv::line(yaw_figure,p1,p2,cv::Scalar(0,0,255),1,cv::LINE_AA,0);
  }
  cv::line(yaw_figure,cv::Point2f(0,125),cv::Point2f(499,125),cv::Scalar(255,255,255),1,cv::LINE_AA,0);
  cv::line(yaw_figure,cv::Point2f(0,125-0.5/yaw_res),cv::Point2f(499,125-0.5/yaw_res),cv::Scalar(0,255,255),1,cv::LINE_AA,0);
  cv::line(yaw_figure,cv::Point2f(0,125+0.5/yaw_res),cv::Point2f(499,125+0.5/yaw_res),cv::Scalar(0,255,255),1,cv::LINE_AA,0);
  cv::line(yaw_figure,cv::Point2f(0,125-(yaw_mid-yaw_mean)/yaw_res),cv::Point2f(499,125-(yaw_mid-yaw_mean)/yaw_res),cv::Scalar(255,0,0),1,cv::LINE_AA,0);
  std::string text_raw = "Yaw Res(degree): "+std::to_string(yaw_res);
  cv::putText(yaw_figure,text_raw,cv::Point(20,20),fontFace,fontScale,cv::Scalar::all(255),thickness,1);
  // x
  double x_min=*std::min_element(pose_x.begin(),pose_x.end());
  double x_max=*std::max_element(pose_x.begin(),pose_x.end());
  float x_res=0.5;
  int x_ceils=ceil((x_max-x_min)/x_res);
  if(x_ceils>200 || (x_ceils<20 && x_ceils>0)) x_res=(x_max-x_min)/200.0;
  double x_mid=(x_max+x_min)/2.0;
  cv::Mat x_figure=cv::Mat(250,500,CV_8UC3,cv::Scalar(125,125,125));
  for(int k=0;k<pose_x.size();k++)
  {
    cv::Point2f p1((k+1)*4.0,125);
    float p2_y=125-(pose_x[k]-x_mean)/x_res;
    cv::Point2f p2((k+1)*4.0,p2_y);
    cv::line(x_figure,p1,p2, cv::Scalar(0,255,0), 1, cv::LINE_AA, 0);
  }
  cv::line(x_figure,cv::Point2f(0,125),cv::Point2f(499,125),cv::Scalar(255,255,255),1,cv::LINE_AA,0);
  cv::line(x_figure,cv::Point2f(0,std::max<float>(0,125-0.5/x_res)),cv::Point2f(499,std::max<float>(0,125-0.5/x_res)),cv::Scalar(0,255,255),1,cv::LINE_AA,0);
  cv::line(x_figure,cv::Point2f(0,std::max<float>(0,125+0.5/x_res)),cv::Point2f(499,std::max<float>(0,125+0.5/x_res)),cv::Scalar(0,255,255),1,cv::LINE_AA,0);
  cv::line(x_figure,cv::Point2f(0,std::max<float>(0,125-(x_mid-x_mean)/x_res)),
          cv::Point2f(499,std::max<float>(0,125-(x_mid-x_mean)/x_res)),cv::Scalar(255,0,0),1,cv::LINE_AA,0);
  std::string text_raw_ = "X Res(cm): "+std::to_string(x_res);
  cv::putText(x_figure,text_raw_,cv::Point(20,20),fontFace,fontScale,cv::Scalar::all(255),thickness,1);
  // concat
  cv::Mat figure=cv::Mat(500,500,CV_8UC3);
  yaw_figure.rowRange(0,250-1).copyTo(figure.rowRange(0,250-1));
  x_figure.rowRange(0,250-1).copyTo(figure.rowRange(250,250+250-1));
  cv::line(figure,cv::Point2f(0,250),cv::Point2f(499,250),cv::Scalar(0,0,0),3,cv::LINE_AA,0);
  
  std::string figure_name=current_exp+"/"+"figure.jpg";
  cv::imwrite(figure_name,figure);

  return true;
}
