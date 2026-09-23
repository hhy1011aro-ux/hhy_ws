#include "perception_storage/hecbot_storage_itof_monitor.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <stdlib.h>
StorageItof::StorageItof()
{
    ros::NodeHandle pnh("~");
    pnh.param("itof_frame", itof_frame_left, std::string("/itof_left_front_frame"));
    pnh.param("cloud_topic", cloud_topic_left, std::string("/itof/itof_left_front/tof_frame/pointcloud"));
    pnh.param("itof_frame_right", itof_frame_right, std::string("/itof_right_front_frame"));
    pnh.param("cloud_topic_right", cloud_topic_right, std::string("/itof/itof_right_front/tof_frame/pointcloud"));
    pnh.param("storage_monitor",storage_monitor,0);
    pnh.param("right_th_max",right_th_max,6.0);
    pnh.param("right_th_min",right_th_min,-4.0);
    pnh.param("left_th_max",left_th_max,5.0);
    pnh.param("left_th_min",left_th_min,-2.5);
    ros::NodeHandle nh;
    ori_pub_ = nh.advertise<sensor_msgs::PointCloud2>("ori_cloud",200);
    collision_pub_ = nh.advertise<std_msgs::String>("/hecbot_perception/storage_collision",200);
    horizon_pub = nh.advertise<sensor_msgs::PointCloud2>("/horizon_pub",200);
    vertical_pub = nh.advertise<sensor_msgs::PointCloud2>("/vertical_pub",200);
    itof_forward_pub = nh.advertise<sensor_msgs::PointCloud2>("/spark_charging/cloud_lines",200);
    //订阅itof点云
    cloud_left_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(cloud_topic_left, 1, &StorageItof::cloudleftCallback, this, ros::TransportHints().tcpNoDelay()); 
    cloud_right_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(cloud_topic_right, 1, &StorageItof::cloudrightCallback, this, ros::TransportHints().tcpNoDelay()); 
    pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/hecbot_perception/storage_goal_pose", 1, &StorageItof::poseCallback_, this, ros::TransportHints().tcpNoDelay());//20230825  
    storage_sub_ = nh.subscribe<std_msgs::String>("/app_sub", 1, &StorageItof::storageCallback, this, ros::TransportHints().tcpNoDelay()); 
    cloud_ori.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

StorageItof::~StorageItof()
{

}

//点云发布(指针)
void StorageItof::cloudptrPub(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, ros::Publisher &pub)
{
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud_in, msg);
    msg.header.frame_id = "/base_link";
    msg.header.stamp = ros::Time::now();
    pub.publish(msg);
    return;
}

//TF to Affin3f  从tf变换转换到了变换矩阵
Eigen::Matrix4f StorageItof::transformTf(const tf::StampedTransform& transform)
{
    //获得平移
    Eigen::Translation3f t(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
    //获得旋转
    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
    Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    transform_matrix = (t * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
    return transform_matrix;
}


void StorageItof::coordinateTransform(const std::string &target_frame,
                               const std::string &source_frame, 
                               const tf::TransformListener &listener, 
                               tf::StampedTransform &transform)
{
    try
    {   
        listener.waitForTransform(target_frame, source_frame, ros::Time(0),ros::Duration(3.0));
        listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    return ;
}

//统计滤波剔除离群点
PointCloudT::Ptr StorageItof::stasticFilter(const PointCloudT::Ptr &cloud_in, const PointCloudT::Ptr &cloud_out)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statis_filter;
    statis_filter.setInputCloud(cloud_in);
    statis_filter.setMeanK(50);
    statis_filter.setStddevMulThresh(1.0);
    statis_filter.filter(*cloud_out);
    return cloud_out;
}

void StorageItof::poseCallback_(const geometry_msgs::PoseStamped pose){
	ROS_INFO("poseCallback_:x: %f",pose.pose.position.x);
	ROS_INFO("poseCallback_:y: %f",pose.pose.position.y);
    pose_ = pose;
    if(pose_.pose.position.y<0){
        state_selection = "right";
    }
    if(pose_.pose.position.y>0){
        state_selection = "left";
    }  
}
void StorageItof::storageCallback(const std_msgs::String::ConstPtr &msg){
    Json::Reader reader;
    Json::Value root;
    Json::Value data;
    if(reader.parse(msg->data,root))
    {
        // std::cerr << "root" << root << std::endl;
        if(!root["data"].isNull())
        {
            data = root["data"];
            if(data["result"].asString() == "170")
            {
                if(storage_monitor > 0.001){//20230831
                    x_range_min=100;
                    x_range_max=-100;
                    th_range_min=100;
                    th_range_max=-100;
                    ROS_INFO("=========BEGIN contraposition");
                    first_time = true;
                    need_collision_detect_ = true;
                }
            }
            if(data["result"].asString() == "100" || data["result"].asString() == "300"|| data["result"].asString() == "174"|| data["result"].asString() == "173")
            {
                need_collision_detect_ = false;
                ROS_INFO("x_range_min %f,x_range_max %f,th_range_min%.2f,th_range_max%.2f",x_range_min,x_range_max,th_range_min,th_range_max);
                ROS_INFO("------------");   
            }
        }
        else
        {
            ROS_ERROR("no /app_sub");
        }
    }
    else
    {
        ROS_ERROR("storageCallback:Json Parse ERROR");
    }
}

void StorageItof::process()
{
    // ROS_INFO("process");
    geometry_msgs::PoseStamped pose_in_bl;
    geometry_msgs::PoseStamped pose_in_map;
    geometry_msgs::PoseStamped pose_itof;
    // extern std::string res_file_name;
    //base_link itof odom三者间的变换关系
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>());  //变换后点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_v_line_transformed(new pcl::PointCloud<pcl::PointXYZ>());  //变换后点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_h_line_transformed(new pcl::PointCloud<pcl::PointXYZ>());  //变换后点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr ROI_cloud(new pcl::PointCloud<pcl::PointXYZ>());  //感兴趣区域点云
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr horizon_line_cloud(new pcl::PointCloud<pcl::PointXYZ>());  //感兴趣区域点云       
    pcl::PointCloud<pcl::PointXYZ>::Ptr vertical_line_cloud(new pcl::PointCloud<pcl::PointXYZ>());  //感兴趣区域点云       

    if(need_collision_detect_){
	// ROS_INFO("NOT first_time");
        if(first_time){
        // ROS_INFO("first_time");
            try 
            {  
                coordinateTransform("/base_link", itof_frame_left, listener_itof_left2base_link, transform_itof_left2base_link); 
                coordinateTransform("/base_link", itof_frame_right, listener_itof_right2base_link, transform_itof_right2base_link); 
                coordinateTransform(itof_frame_right, "/base_link", listener_base_link2itof_right, transform_base_link2itof_right);
                coordinateTransform(itof_frame_left, "/base_link", listener_base_link2itof_left, transform_base_link2itof_left); 
            }
            catch (pcl::PCLException &e)
            {
                ROS_INFO_STREAM("abnormal process1...");
            }
            catch (std::exception &e)
            {
                ROS_INFO_STREAM("abnormal process2....");
            }
            if(state_selection == "right")
            {//transfer pose_ from baselink to itof
                tf::Stamped<tf::Pose> pose_itof_tf;
                tf::Stamped<tf::Pose> pose_BL_tf;
                poseStampedMsgToTF(pose_, pose_BL_tf);
                pose_itof_tf.setData(transform_base_link2itof_right * pose_BL_tf);
                // poseStampedTFToMsg(pose_itof_tf, pose_itof);
                poseStampedTFToMsg(pose_itof_tf, pose_);
            }
            if(state_selection == "left")
            {//transfer pose_ from baselink to itof
                tf::Stamped<tf::Pose> pose_itof_tf;
                tf::Stamped<tf::Pose> pose_BL_tf;
                poseStampedMsgToTF(pose_, pose_BL_tf);
                pose_itof_tf.setData(transform_base_link2itof_left * pose_BL_tf);
                // poseStampedTFToMsg(pose_itof_tf, pose_itof);
                poseStampedTFToMsg(pose_itof_tf, pose_);
            }
            // point_in_x = pose_itof.pose.position.x;
            // point_in_y = pose_itof.pose.position.y;
            // point_in_z = pose_itof.pose.position.z;
            point_in_x = pose_.pose.position.x; //in itof
            point_in_y = pose_.pose.position.y;
            point_in_z = pose_.pose.position.z;
            x_mid = point_in_x;
            y_mid = point_in_y;
            ROS_INFO("POSECALLBACK IN ITOF FRAME X%f,Y%f,Z%f,",point_in_x,point_in_y,point_in_z);
            // 向csv文档中写入数据                
            // std::ofstream dataFile(res_file_name, std::ios::app);
            // dataFile << "x" << "," << "y" << "," << "th" << std::endl;
            first_time = false;
        }
        //统计滤波剔除异常点
        stasticFilter(cloud_ori, cloud_ori);  
        ROS_INFO_STREAM("stastic cloud size is: " << cloud_ori->points.size()); 
        // ROS_INFO("extractROI X%f,y%f,z%f,",point_in_x,point_in_y,point_in_z);
        ROI_cloud = extractROI(cloud_ori); //20230731
        ROS_INFO_STREAM("roi_cloud size is: " << ROI_cloud->points.size());
        if(ROI_cloud->points.size() == 0){return;}
        horizon_line_cloud = extract_horizon_line(ROI_cloud);//20230811
        if(x_min == 100){ROS_INFO("not enough points in x");return;}
        ROS_INFO("x_min: %f,x_max:%f",x_min,x_max);
        x_mid = (x_min+x_max)/2;
        // z_mid = (z_min+z_max)/2;//method 1

        // ROS_INFO("CAL 10-------");
        // double z_avg_10 = four_point_z_cal(ROI_cloud,0.1);
        // ROS_INFO("z_avg_10: %f,  z_right - z_left: %f",z_avg_10,z_right-z_left);
        // if((z_right == -100 )||(z_left == 100)){
        //     ROS_INFO("CAL z_avg_10 z_right == -100 ||z_left == 100");
        //     return;
        // }
        // double th_10 = atan2(z_right-z_left,0.2)* 180 / 3.14159;
        // if(z_avg_10 > 10){
        //     ROS_INFO("z_avg_10 board is blocked");
        //     return;
        // }
        // ROS_INFO("CAL 5-------");
        // double z_avg_5 = four_point_z_cal(ROI_cloud,0.05);
        // ROS_INFO("z_avg_5: %f,  z_right - z_left: %f",z_avg_5,z_right-z_left);
        // if((z_right == -100 )||(z_left == 100)){
        //     ROS_INFO("CAL z_avg_5 z_right == -100 ||z_left == 100");
        //     return;
        // }
        // double th_5 = atan2(z_right-z_left,0.1)* 180 / 3.14159;
        // if(z_avg_5 > 10){
        //     ROS_INFO("z_avg_5 board is blocked");
        //     return;
        // }
        // double th;
        // ROS_INFO("th_10 :%f,th_5 :%f",th_10,th_5);
        // if((fabs(th_10)<50 )&&(fabs(th_5)<50) ){th = (th_10 + th_5)/2;}
        // else if (fabs(th_10)<50){th = th_10;}
        // else if (fabs(th_5)<50){th = th_5;}else{ROS_INFO("th calculate fail");return;}
        // z_mid = (z_avg_10+z_avg_5)/2;//method 3

        //method 4
        z_mid = projection_cal(ROI_cloud,0.1);
        double th = atan2(z_right-z_left,0.2)* 180 / 3.14159;
        if(fabs(th)>50){ROS_INFO("th calculate fail");return;}
        // ROS_INFO("calculate horizon done");
        // double dis = averageZ(ROI_cloud_center);//
        // ROS_INFO("dis",dis);
        // z_mid = z_avg;  //method 2:problem is that sometimes the num would be up to 6000
        // if(z_mid<0.2){return;}
        // double th = atan2(z_max-z_min,x_max-x_min)* 180 / 3.14159;
        vertical_line_cloud = extract_vertical_line(ROI_cloud);//20230811
        // y_mid = (y_min+y_max)/2;
        y_mid = y_min+board_height/2;
        pose_center = pose_;
        pose_center.pose.position.x = x_mid;
        pose_center.pose.position.y = y_mid;
        pose_center.pose.position.z = z_mid;
        ROS_INFO("x_mid:%f,y_mid:%f,z_mid:%f,th%f",x_mid,y_mid,z_mid,th);
        if(z_mid < 0.25){return;}
        tf::Stamped<tf::Pose> pose_center_itof_tf;
        tf::Stamped<tf::Pose> pose_center_BL_tf;

        poseStampedMsgToTF(pose_center, pose_center_itof_tf);
        if(state_selection == "left"){
            pose_center_BL_tf.setData(transform_itof_left2base_link * pose_center_itof_tf);    
        }
        if(state_selection == "right"){
            pose_center_BL_tf.setData(transform_itof_right2base_link * pose_center_itof_tf);    
        }
        poseStampedTFToMsg(pose_center_BL_tf, pose_center_in_BL);
        // if(pose_center_in_BL.pose.position.x < 0.00000001){return;}
        ROS_INFO("x:%f,y:%f,th:%f",pose_center_in_BL.pose.position.x,pose_center_in_BL.pose.position.y,th);
        
        // std::ofstream dataFile(res_file_name, std::ios::app);
        // dataFile << pose_center_in_BL.pose.position.x << "," << pose_center_in_BL.pose.position.y << "," << th << std::endl;
        // if(pose_center_in_BL.pose.position.x < 0.00000001 or th>50){return;}
        //pub collision
        std_msgs::String msg_collision;
        Json::Value msg_collision_;
        if(state_selection == "left"){
            if(pose_center_in_BL.pose.position.y < 0.6){
                if(pose_center_in_BL.pose.position.x>0.05 || pose_center_in_BL.pose.position.x<-0.02 ||th<left_th_min||th>left_th_max){
                    ROS_INFO("Storage collision*********************"); 
                    msg_collision_["result"] = true;
                    msg_collision_["x"] = pose_center_in_BL.pose.position.x;
                    msg_collision_["y"] = pose_center_in_BL.pose.position.y;
                    msg_collision_["th"] = th;
                    msg_collision_["x_limit"][0] = -0.02;
                    msg_collision_["x_limit"][1]  = 0.05;
                    msg_collision_["th_limit"][0] = left_th_min;
                    msg_collision_["th_limit"][1] = left_th_max;
                    Json::StyledWriter style;
                    msg_collision.data = style.write(msg_collision_);
                    collision_pub_.publish(msg_collision);
                }
            }
            else{
                msg_collision_["result"] = false;
                msg_collision_["x"] = pose_center_in_BL.pose.position.x;
                msg_collision_["y"] = pose_center_in_BL.pose.position.y;
                msg_collision_["th"] = th;
                msg_collision_["x_limit"][0] = -0.02;
                msg_collision_["x_limit"][1]  = 0.05;
                msg_collision_["th_limit"][0] = left_th_min;
                msg_collision_["th_limit"][1] = left_th_max;
                Json::StyledWriter style;
                msg_collision.data = style.write(msg_collision_);
                collision_pub_.publish(msg_collision);
            }
        }
        if(state_selection == "right"){
            if(pose_center_in_BL.pose.position.y > -0.6){
                if(pose_center_in_BL.pose.position.x>0.03 || pose_center_in_BL.pose.position.x<-0.1 ||th<right_th_min||th>right_th_max){
                    ROS_INFO("Storage collision*********************");
                    msg_collision_["result"] = true;
                    msg_collision_["x"] = pose_center_in_BL.pose.position.x;
                    msg_collision_["y"] = pose_center_in_BL.pose.position.y;
                    msg_collision_["th"] = th;
                    msg_collision_["x_limit"][0] = -0.1;
                    msg_collision_["x_limit"][1] = 0.03;
                    msg_collision_["th_limit"][0] = right_th_min;
                    msg_collision_["th_limit"][1] = right_th_max;
                    Json::StyledWriter style;
                    msg_collision.data = style.write(msg_collision_);
                    collision_pub_.publish(msg_collision);
                }
            }
            else{
                msg_collision_["result"] = false;
                msg_collision_["x"] = pose_center_in_BL.pose.position.x;
                msg_collision_["y"] = pose_center_in_BL.pose.position.y;
                msg_collision_["th"] = th;
                msg_collision_["x_limit"][0] = -0.1;
                msg_collision_["x_limit"][1] = 0.03;
                msg_collision_["th_limit"][0] = right_th_min;
                msg_collision_["th_limit"][1] = right_th_max;
                Json::StyledWriter style;
                msg_collision.data = style.write(msg_collision_);
                collision_pub_.publish(msg_collision);
            }
        }

        point_in_x = pose_center.pose.position.x;
        point_in_y = pose_center.pose.position.y;
        point_in_z = pose_center.pose.position.z;
        
        if(pose_center_in_BL.pose.position.x < x_range_min){x_range_min = pose_center_in_BL.pose.position.x;}
        if(pose_center_in_BL.pose.position.x > x_range_max){x_range_max = pose_center_in_BL.pose.position.x;}
        if(th < th_range_min){th_range_min = th;}
        if(th > th_range_max){th_range_max = th;}
        if(fabs(pose_center_in_BL.pose.position.y) < 0.3){need_collision_detect_ = false;}

        //点云数据转换到base_link  
        // Eigen::Affine3f itofTobase_link = Eigen::Affine3f::Identity();  //itof->base_link的变换矩阵 
        // itofTobase_link = transformTf(transform_itof_left2base_link);  
        // // pcl::transformPointCloud(*cloud_ori, *cloud_transformed, itofTobase_link);      //itof变换到base_link
        // pcl::transformPointCloud(*ROI_cloud, *cloud_transformed, itofTobase_link);      //itof变换到base_link  //20230731
        // cloudptrPub(cloud_transformed, ori_pub_);//

        // pcl::transformPointCloud(*horizon_line_cloud, *cloud_h_line_transformed, itofTobase_link);      //itof变换到base_link  //20230731
        // cloudptrPub(cloud_h_line_transformed, horizon_pub);//
        // pcl::transformPointCloud(*vertical_line_cloud, *cloud_v_line_transformed, itofTobase_link);      //itof变换到base_link  //20230731
        // cloudptrPub(cloud_v_line_transformed, vertical_pub);//
    }
}

void StorageItof::cloudrightCallback(const sensor_msgs::PointCloud2::ConstPtr &msgs){
    if(state_selection == "right"){
        // ROS_INFO("really right");
        if(need_collision_detect_){
            itof_forward_pub.publish(*msgs);
        }
        pcl::fromROSMsg(*msgs, *cloud_ori);
        process();
    }
}
void StorageItof::cloudleftCallback(const sensor_msgs::PointCloud2::ConstPtr &msgs)
{
    if(state_selection == "left"){
        // ROS_INFO("really left");
        if(need_collision_detect_){
            itof_forward_pub.publish(*msgs);
        }
        pcl::fromROSMsg(*msgs, *cloud_ori);
        process();
    }
}

double StorageItof::averageZ(const PointCloudT::Ptr &cloud_in)
{
    double sum = 0;
    int num = 0;
    for (const auto &p : cloud_in->points)
    {
        sum += p.z;
        num += 1;
    }
    if(num == 0){
        return 100;
    }
    else{return double(sum/num);}
}

double StorageItof::maxZ(const PointCloudT::Ptr &cloud_in)
{
    double z_max = -100;
    for (const auto &p : cloud_in->points)
    {
        if(p.z > z_max){z_max = p.z;}
    }
    return z_max;       
}
double StorageItof::maxZ(const PointCloudT::Ptr &cloud_in,const PointCloudT::Ptr &cloud_in1)
{
    double z_max = -100;
    for (const auto &p : cloud_in->points)
    {
        if(p.z > z_max){z_max = p.z;}
    }
    for (const auto &p : cloud_in1->points)
    {
        if(p.z > z_max){z_max = p.z;}
    }
    return z_max;       
}
double StorageItof::minZ(const PointCloudT::Ptr &cloud_in)
{
    double z_min = 100;
    for (const auto &p : cloud_in->points)
    {
        if(p.z < z_min){z_min = p.z;}
    }
    return z_min;       
}
double StorageItof::minZ(const PointCloudT::Ptr &cloud_in,const PointCloudT::Ptr &cloud_in1)
{
    double z_min = 100;
    for (const auto &p : cloud_in->points)
    {
        if(p.z < z_min){z_min = p.z;}
    }
    for (const auto &p : cloud_in1->points)
    {
        if(p.z < z_min){z_min = p.z;}
    }
    return z_min;       
}
//提取感兴趣区域
PointCloudT::Ptr StorageItof::extractROI(const PointCloudT::Ptr &cloud_in)
{
    double left_x=point_in_x - board_length/2 - 0.02;
    double right_x=point_in_x + board_length/2 + 0.02;
    double left_y=point_in_y - board_height/2-0.01;
    double right_y=point_in_y + board_height/2+0.01;
    PointCloudT::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto &p : cloud_in->points)
    {
        if(p.z>=point_in_z+0.05 || p.z<=point_in_z-0.2){
            continue;	
        }
        if(p.x < right_x && p.x > left_x && p.y <right_y && p.y > left_y)
        {
            cloud_out->push_back(p);
        }
    }
    return cloud_out;
}

PointCloudT::Ptr StorageItof::extract_part_ROI(const PointCloudT::Ptr &cloud_in,double x_in,double y_in)
{
    double left_x=x_in - 0.01;
    double right_x=x_in + 0.01;
    double left_y=y_in -0.01;
    double right_y=y_in +0.01;
    PointCloudT::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto &p : cloud_in->points)
    {
        if(p.x < right_x && p.x > left_x && p.y <right_y && p.y > left_y)
        {
            cloud_out->push_back(p);
        }
    }
    return cloud_out;
}

double StorageItof::four_point_z_cal(const PointCloudT::Ptr &cloud_in,double range)
{
    z_left = 100;
    z_right = -100;
    double left_up1_x = point_in_x + range;
    double left_up1_y = point_in_y + range;
    double right_up1_x = point_in_x - range;
    double right_up1_y = point_in_y + range;
    double left_down1_x = point_in_x + range;
    double left_down1_y = point_in_y - range;
    double right_down1_x = point_in_x - range;
    double right_down1_y = point_in_y - range; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left_up1(new pcl::PointCloud<pcl::PointXYZ>());  
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right_up1(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left_down1(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right_down1(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_left_up1 = extract_part_ROI(cloud_in,left_up1_x,left_up1_y);
    cloud_right_up1 = extract_part_ROI(cloud_in,right_up1_x,right_up1_y);
    cloud_left_down1 = extract_part_ROI(cloud_in,left_down1_x,left_down1_y);
    cloud_right_down1 = extract_part_ROI(cloud_in,right_down1_x,right_down1_y);
    double z_left_up1 = averageZ(cloud_left_up1);
    double z_right_up1 = averageZ(cloud_right_up1);
    double z_left_down1 = averageZ(cloud_left_down1);   
    double z_right_down1 = averageZ(cloud_right_down1);

    std::vector<double> vec_z;//创建一个向量存储容器 int
    vec_z.clear();
    
    // if(!std::isnan(z_left_up1) && z_left_up1<100){vec_z.push_back(z_left_up1);z_left = maxZ(cloud_left_up1);}
    // if(!std::isnan(z_right_up1)&& z_right_up1<100){vec_z.push_back(z_right_up1);z_right = minZ(cloud_right_down1);}
    // if(!std::isnan(z_left_down1)&& z_left_down1<100){vec_z.push_back(z_left_down1);z_left = maxZ(cloud_left_up1);}
    // if(!std::isnan(z_right_down1)&& z_left_down1<100){vec_z.push_back(z_right_down1);z_right = minZ(cloud_right_down1);}

    if(!std::isnan(z_left_up1) && z_left_up1<100 && !std::isnan(z_left_down1)&& z_left_down1<100){
        vec_z.push_back(z_left_up1);vec_z.push_back(z_left_down1);z_left = maxZ(cloud_left_up1,cloud_left_down1);
    }   
    else if(!std::isnan(z_left_up1) && z_left_up1<100){
        vec_z.push_back(z_left_up1);z_left = maxZ(cloud_left_up1);
    }
    else if(!std::isnan(z_left_down1) && z_left_down1<100){
        vec_z.push_back(z_left_down1);z_left = maxZ(cloud_left_down1);
    }
    else{
        ROS_INFO("left part blank");return 100;
    }
    if(!std::isnan(z_right_up1) && z_right_up1<100 && !std::isnan(z_right_down1)&& z_right_down1<100){
        vec_z.push_back(z_right_up1);vec_z.push_back(z_right_down1);z_right = minZ(cloud_right_up1,cloud_right_down1);
    }
    else if(!std::isnan(z_right_up1) && z_right_up1<100){
        vec_z.push_back(z_right_up1);z_right = minZ(cloud_right_up1);
    }
    else if(!std::isnan(z_right_down1) && z_right_down1<100){
        vec_z.push_back(z_right_down1);z_right = minZ(cloud_right_down1);
    }
    else{
        ROS_INFO("right part blank");return 100;
    }
    auto it = std::minmax_element(vec_z.begin(), vec_z.end());
    int z_outer_min = *it.first;
    int z_outer_max = *it.second;
    if(z_outer_max - z_outer_min > 0.3)
        {return 100;}
    // else if (vec_z.empty())  
    // {
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr ROI_cloud_center(new pcl::PointCloud<pcl::PointXYZ>());  //感兴趣区域点云
    //     ROI_cloud_center = extractROI_center(cloud_ori); //20230731
    //     ROS_INFO("four points all nan");//four points blank
    //     if(std::isnan(averageZ(ROI_cloud_center)) ||averageZ(ROI_cloud_center)>99 ){
    //         return 100;//ROI_cloud_center blank
    //     }
    //     else{return averageZ(ROI_cloud_center);}
    // }
    else
    {
        return std::accumulate(vec_z.begin(), vec_z.end(), 0.0) / vec_z.size();
    }
}

double StorageItof::projection_cal(const PointCloudT::Ptr &cloud_in,double range)
{
    z_left = 100;
    z_right = -100;
    double sum_z_left = 0,sum_z_right = 0,num_z_left = 0,num_z_right = 0;
    double left_range = point_in_x + range;
    double right_range = point_in_x - range;

    for(const auto &p : cloud_in->points)
    {
        if(fabs(p.x-left_range)<0.01){
            sum_z_left += p.z;
            num_z_left +=1;
        }
        if(fabs(p.x-right_range)<0.01){
            sum_z_right += p.z;
            num_z_right +=1;
        }
    }
    std::cerr << "num_z_left" << num_z_left << "num_z_right:" << num_z_right << std::endl;
    if(num_z_left == 0 || num_z_right == 0){
        ROS_INFO("BLANK");
        return 100;
    }
    else{
        z_left = sum_z_left/num_z_left;
        z_right = sum_z_right/num_z_right;
    }
    std::cerr << "z_left:" << z_left << "z_right:" << z_right << std::endl;
    ROS_INFO("z_left:%f,z_right%f",z_left,z_right);
    return((z_left+z_right)/2);
}



PointCloudT::Ptr StorageItof::extractROI_center(const PointCloudT::Ptr &cloud_in)
{
    double left_x=x_mid - 0.03;
    double right_x=x_mid + 0.03;
    double left_y=point_in_y -0.03;
    double right_y=point_in_y +0.03;
    PointCloudT::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto &p : cloud_in->points)
    {
        if(p.x < right_x && p.x > left_x && p.y <right_y && p.y > left_y)
        {
            cloud_out->push_back(p);
        }
        
    }
    return cloud_out;
}


PointCloudT::Ptr StorageItof::extract_horizon_line(const PointCloudT::Ptr &cloud_in)
{
    x_min = 100;
    x_max = -100;
    // z_min = 100;
    // z_max = -100;
    // double sum=0;
    // int num =0;
    PointCloudT::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto &p : cloud_in->points)
    {
        if(p.y < point_in_y+0.02 && p.y > point_in_y-0.02)
        {
            cloud_out->push_back(p);
            
            if(p.x < x_min){x_min = p.x;}
            if(p.x > x_max){x_max = p.x;}
            // if(p.z < z_min){z_min = p.z;}
            // if(p.z > z_max){z_max = p.z;}
            // sum += p.z;
            // num += 1;
        }
    }
    // ROS_INFO("sum%f,num%d",sum,num);
    // z_avg = double(sum/num);
    return cloud_out;
}

PointCloudT::Ptr StorageItof::extract_vertical_line(const PointCloudT::Ptr &cloud_in)
{
    y_min = 100;
    PointCloudT::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto &p : cloud_in->points)
    {
        if(p.x < x_mid+0.005 && p.x > x_mid-0.005)
        {
            cloud_out->push_back(p);
            if(p.y < y_min){y_min = p.y;}
        }
    }
    return cloud_out;
}


