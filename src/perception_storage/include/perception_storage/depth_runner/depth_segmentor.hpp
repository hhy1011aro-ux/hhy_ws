#ifndef DEPTH_SEGMENTOR_HPP
#define DEPTH_SEGMENTOR_HPP

#include "perception_storage/depth_runner/depth_elsed.h"
#include "perception_storage/tool_box.hpp"
#include "perception_storage/common.hpp"

class depth_segmentor
{
  public:
    depth_segmentor(float & length,float & angle,bool & loose);
    std::pair<cv::Mat,std::vector<hypothesis>> segmentation(depth_mapping &depth_input);
  private:
    float length_threshold;
    float angle_threshold;
    bool loosely;
    depth_runner::ELSED edge_runner;
};

#endif