#ifndef DEPTH_FULLSEGMENTINFO_H_
#define DEPTH_FULLSEGMENTINFO_H_

#include <ostream>
#include <memory>
#include "perception_storage/depth_runner/depth_utils.h"

namespace depth_runner 
{
  class FullSegmentInfo 
  {
  public:
    FullSegmentInfo(const std::vector<Pixel> &pts);
    FullSegmentInfo(const std::vector<Pixel> &pts, int startIdx);
    void init(const std::vector<Pixel> &pts, int startIdx);
    inline bool isInlier(int x, int y, double lineFitErrThreshold) 
    {
      const double pointToLineDis = equation[0] * x + equation[1] * y + equation[2];
      return UPM_ABS(pointToLineDis) < lineFitErrThreshold;
    }
    void addPixel(int x, int y, int pixelIndexInEdge, bool isPixelAtTheEnd = true);
    void finish();
    void skipPositions();
    inline double getFitError() const 
    {
      double dist, fitError = 0;
      for (int i = firstPxIndex; i <= lastPxIndex; i++) 
      {
        dist = equation[0] * (*pixels)[i].x + equation[1] * (*pixels)[i].y + equation[2];
        fitError += dist * dist;
      }
      return fitError / N;
    }
    void removeLastPx(bool removeFromTheEnd = true);
    inline void reset() 
    {
      sum_x_i = 0, sum_y_i = 0, sum_x_i_y_i = 0, sum_x_i_2 = 0, N = 0;
      firstPxIndex = -1;
      lastPxIndex = -1;
      arePixelsSorted = true;
      firstEndpointExtended = false;
      secondEndpointExtended = false;
    }
    inline bool horizontal() const 
    {
      return isHorizontal;
    }
    inline int getNumOfPixels() const 
    { 
      return N;
    }
    inline const Segment &getEndpoints() const 
    { 
      return endpoints;
    }

    ImageEdge getPixels() const;
    inline const Pixel &getFirstPixel() const 
    { 
      return firstPx;
    }
    inline const Pixel &getLastPixel() const 
    { 
      return lastPx;
    }
    inline const Pixel *begin() const 
    { 
      return &((*pixels)[firstPxIndex]);
    }
    inline const Pixel *end() const 
    { 
      return (&((*pixels)[lastPxIndex])) + 1;
    }
    inline const cv::Vec3d &getLineEquation() const 
    { 
      return equation;
    }
    inline bool hasSecondSideElements() const 
    { 
      return !arePixelsSorted;
    }
    bool firstEndpointExtended = false;
    bool secondEndpointExtended = false;

  private:
    void leastSquareLineFit(const std::vector<Pixel> &pts, int startIndex = 0);
    void leastSquaresLineFitNewPoint(int x, int y);
    void subtractPointFromModel(const Pixel &p);
    void calculateLineEq();
    void calcSegmentEndpoints();

    int64_t sum_x_i = 0, sum_y_i = 0, sum_x_i_y_i = 0, sum_x_i_2 = 0;
    uint32_t N = 0;
    int dx, dy;
    bool isHorizontal;
    Segment endpoints;
    cv::Vec3d equation;
    Pixel firstPx, prevFirstPx, lastPx, prevLastPx;
    int firstPxIndex = -1, lastPxIndex = -1;
    float salience = -1;
    const ImageEdge *pixels;
    bool arePixelsSorted = true;
  };

}

#endif
