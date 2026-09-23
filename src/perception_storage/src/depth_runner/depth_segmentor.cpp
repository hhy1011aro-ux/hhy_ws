#include "perception_storage/depth_runner/depth_segmentor.hpp"

depth_segmentor::depth_segmentor(float & length,float & angle,bool & loose):length_threshold(length),angle_threshold(angle),loosely(loose)
{

}

std::pair<cv::Mat,std::vector<hypothesis>> depth_segmentor::segmentation(depth_mapping &depth_input)
{
  cv::Mat depth_3ch;
  cv::cvtColor(depth_input.depth,depth_3ch,cv::COLOR_GRAY2RGB);
  int depth_rows=depth_input.depth.rows;
  int depth_cols=depth_input.depth.cols;
  depth_runner::Segments segment_array = edge_runner.detect(depth_input.depth);
  depth_runner::Segments vertical_edges,horizon_edges;
  cv::Mat display=depth_3ch.clone();
  for(auto & seg:segment_array)
  {
    auto norm_=cv::norm(cv::Point2f(seg[0],seg[1])-cv::Point2f(seg[2],seg[3]));
    if(fabs(norm_-length_threshold)<0.75*length_threshold)
    {
      double angle=atan2(fabs(seg[0]-seg[2]),fabs(seg[1]-seg[3]))/M_PI*180;
      if(angle<angle_threshold)
      {
        if(seg[1]>seg[3])
        {
          std::swap(seg[0],seg[2]);
          std::swap(seg[1],seg[3]);
        }
        vertical_edges.emplace_back(seg);
      }
      if(angle>60)
      {
        if(seg[0]>seg[2])
        {
          std::swap(seg[0],seg[2]);
          std::swap(seg[1],seg[3]);
        }
        horizon_edges.emplace_back(seg);
      }
    }
  }
  std::vector<hypothesis> hypothesis_list;
  if(vertical_edges.size()>=2)
  { 
    for(int i=0;i<(vertical_edges.size()-1);i++)
    {
      auto max_x=std::max(vertical_edges[i][0],vertical_edges[i][2]);
      for(int j=i+1;j<vertical_edges.size();j++)
      {
        auto min_x=std::min(vertical_edges[j][0],vertical_edges[j][2]);
        cv::Vec4f left_edge=vertical_edges[i];
        cv::Vec4f right_edge=vertical_edges[j];
        if(max_x>min_x)
        {
          right_edge=vertical_edges[i];
          left_edge=vertical_edges[j];
        }
        std::vector<cv::Point2f> points_four;
        tool_box::max_box(left_edge,right_edge,points_four);
        if(cv::norm(points_four[0]-points_four[3])<=1.414*length_threshold*1.25
          && cv::norm(points_four[1]-points_four[2])<=1.414*length_threshold*1.25
          && cv::norm(points_four[0]-points_four[2])>=0.5*length_threshold
          && cv::norm(points_four[0]-points_four[2])<=1.25*length_threshold
          && (cv::norm(points_four[0]-points_four[1]))/(cv::norm(points_four[0]-points_four[2]))>=0.5
          && (cv::norm(points_four[0]-points_four[1]))/(cv::norm(points_four[0]-points_four[2]))<=2.0
          && (cv::norm(points_four[0]-points_four[1]))/(cv::norm(points_four[1]-points_four[3]))>=0.5
          && (cv::norm(points_four[0]-points_four[1]))/(cv::norm(points_four[1]-points_four[3]))<=2.0)
        {
          cv::circle(depth_3ch,points_four[0],1,cv::Scalar(255,255,0),2);
          cv::circle(depth_3ch,points_four[1],1,cv::Scalar(255,255,0),2);
          cv::circle(depth_3ch,points_four[2],1,cv::Scalar(255,255,0),2);
          cv::circle(depth_3ch,points_four[3],1,cv::Scalar(255,255,0),2);
          cv::line(depth_3ch,points_four[0],points_four[1],cv::Scalar(0,0,255),1,cv::LINE_AA,0);
          cv::line(depth_3ch,points_four[2],points_four[3],cv::Scalar(0,0,255),1,cv::LINE_AA,0);
          hypothesis_list.emplace_back(hypothesis(points_four,depth_rows,depth_cols));
        }
      }
    }
  }
  if(hypothesis_list.size()==0 && vertical_edges.size()>=1 && horizon_edges.size()>=1 && loosely)
  {
    for(auto & he:horizon_edges)
    {
      double _angle_=atan2(fabs(he[0]-he[2]),fabs(he[1]-he[3]))/M_PI*180;
      auto _norm_=cv::norm(cv::Point2f(he[0],he[1])-cv::Point2f(he[2],he[3]));
      // 接近水平
      if(_angle_>80 && _norm_>=0.8*length_threshold && _norm_<=1.2*length_threshold)
      {
        // 尝试寻找一条左侧的垂直边界
        for(auto & ve:vertical_edges)
        {
          cv::Point2f phl(he[0],he[1]);
          cv::Point2f phr(he[2],he[3]);
          cv::Point2f pvt(ve[0],ve[1]);
          cv::Point2f pvb(ve[2],ve[3]);
          if(cv::norm(pvb-phl)<=3 && cv::norm(pvt-phr)<1.414*length_threshold*1.25 && cv::norm(pvt-phr)>1.414*length_threshold*0.75)
          {
            cv::line(depth_3ch,phl,phr,cv::Scalar(255,0,0),1,cv::LINE_AA,0);
            cv::line(depth_3ch,pvt,pvb,cv::Scalar(0,255,0),1,cv::LINE_AA,0);
            // 平移左边界
            cv::Point2f cpvb=phr;
            cv::Point2f cpvt=phr-pvb+pvt;
            cv::line(depth_3ch,cpvt,cpvb,cv::Scalar(0,255,0),1,cv::LINE_AA,0);
            std::vector<cv::Point2f> points_four={pvt,pvb,cpvt,cpvb};
            cv::circle(depth_3ch,points_four[0],1,cv::Scalar(255,255,0),2);
            cv::circle(depth_3ch,points_four[1],1,cv::Scalar(255,255,0),2);
            cv::circle(depth_3ch,points_four[2],1,cv::Scalar(255,255,0),2);
            cv::circle(depth_3ch,points_four[3],1,cv::Scalar(255,255,0),2);
            hypothesis_list.emplace_back(hypothesis(points_four,depth_rows,depth_cols));
          }
        }
      }
    }
  }
  int px=depth_input.mid_pixel[0];
  int py=depth_input.mid_pixel[1];
  cv::Point2f mid_point(px,py);
  if(hypothesis_list.size()==0 && vertical_edges.size()>=1 && px>0 && py>0 && px<depth_cols-1 && py<depth_rows-1)
  {
    for(auto & ve:vertical_edges)
    {
      cv::Point2f pvt(ve[0],ve[1]);
      cv::Point2f pvb(ve[2],ve[3]);
      cv::Point2f pvm=(pvt+pvb)/2.0;
      if(cv::norm(pvm-mid_point)>=length_threshold*0.5*0.5 
      && cv::norm(pvm-mid_point)<=1.414*length_threshold*0.5 
      && mid_point.y<pvb.y && mid_point.y>pvt.y)
      {
        cv::Point2f phl=pvb;
        cv::Point2f phr(mid_point.x-pvb.x+mid_point.x,pvb.y);
        cv::line(depth_3ch,phl,phr,cv::Scalar(255,0,0),1,cv::LINE_AA,0);
        cv::line(depth_3ch,pvt,pvb,cv::Scalar(0,255,0),1,cv::LINE_AA,0);
        cv::Point2f cpvb=phr;
        cv::Point2f cpvt=phr-pvb+pvt;
        cv::line(depth_3ch,cpvt,cpvb,cv::Scalar(0,255,0),1,cv::LINE_AA,0);
        std::vector<cv::Point2f> points_four={pvt,pvb,cpvt,cpvb};
        cv::circle(depth_3ch,points_four[0],1,cv::Scalar(255,255,0),2);
        cv::circle(depth_3ch,points_four[1],1,cv::Scalar(255,255,0),2);
        cv::circle(depth_3ch,points_four[2],1,cv::Scalar(255,255,0),2);
        cv::circle(depth_3ch,points_four[3],1,cv::Scalar(255,255,0),2);
        cv::circle(depth_3ch,mid_point,1,cv::Scalar(0,255,255),2);
        cv::line(depth_3ch,mid_point,cv::Point2f(0,0),cv::Scalar(255,0,255),1,cv::LINE_AA,0);
        hypothesis_list.emplace_back(hypothesis(points_four,depth_rows,depth_cols));
      }
    }
  }

  return std::make_pair(depth_3ch,hypothesis_list);
}
