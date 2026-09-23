#include <vector>
#include "perception_storage/depth_runner/depth_fullsegment.h"

namespace depth_runner 
{
  FullSegmentInfo::FullSegmentInfo(const std::vector<Pixel> &pts) : pixels(&pts)
  {}

  FullSegmentInfo::FullSegmentInfo(const std::vector<Pixel> &pts, int startIdx) : pixels(&pts)
  {
    init(pts, startIdx);
  }

  void FullSegmentInfo::init(const std::vector<Pixel> &pts, int startIdx)
  {
    assert(pts.size() > startIdx);
    assert(pts.size() >= 2);
    dx = pts.back().x - pts[startIdx].x;
    dy = pts.back().y - pts[startIdx].y;
    isHorizontal = UPM_ABS(dx) >= UPM_ABS(dy);
    leastSquareLineFit(pts, startIdx);
    firstPx = pts[startIdx];
    lastPx = pts.back();
    prevFirstPx = pts[startIdx + 1];
    prevLastPx = pts[pts.size() - 2];
    firstPxIndex = startIdx;
    lastPxIndex = pts.size() - 1;
    arePixelsSorted = true;
    firstEndpointExtended = false;
    secondEndpointExtended = false;
  }

  void FullSegmentInfo::skipPositions()
  {
    static_assert(UPM_SKIP_EDGE_PT == 2, "Error this code is optimized for UPM_SKIP_EDGE_PT = 2");
    assert(pixels->size() == lastPxIndex + 1 + UPM_SKIP_EDGE_PT);
    subtractPointFromModel(firstPx);
    subtractPointFromModel(prevFirstPx);
    firstPxIndex += UPM_SKIP_EDGE_PT;
    lastPxIndex += UPM_SKIP_EDGE_PT;
    firstPx = (*pixels)[firstPxIndex];
    lastPx = pixels->back();
    prevFirstPx = (*pixels)[firstPxIndex + 1];
    prevLastPx = (*pixels)[lastPxIndex - 1];
    dx = lastPx.x - firstPx.x;
    dy = lastPx.y - firstPx.y;
    bool tmp = UPM_ABS(dx) >= UPM_ABS(dy);
    if (tmp != isHorizontal) 
    {
      isHorizontal = tmp;
      leastSquareLineFit(*pixels, firstPxIndex);
    }
    else
    {
      leastSquaresLineFitNewPoint(prevLastPx.x, prevLastPx.y);
      leastSquaresLineFitNewPoint(lastPx.x, lastPx.y);
    }
  }

  void FullSegmentInfo::addPixel(int x, int y, int pixelIndexInEdge, bool isPixelAtTheEnd)
  {
    leastSquaresLineFitNewPoint(x, y);
    assert(!std::isnan(equation[0]) && !std::isnan(equation[1]));
    if (isPixelAtTheEnd)
    {
      prevLastPx = lastPx;
      lastPx.x = x;
      lastPx.y = y;
    }
    else
    {
      prevFirstPx = firstPx;
      firstPx.x = x;
      firstPx.y = y;
    }
    lastPxIndex = pixelIndexInEdge;
    arePixelsSorted = arePixelsSorted && isPixelAtTheEnd;
  }

  void FullSegmentInfo::finish()
  {
    calcSegmentEndpoints();
    assert(!std::isnan(endpoints[0]));
  }

  inline void FullSegmentInfo::calcSegmentEndpoints()
  {
    float a1 = equation[1] * equation[1];
    float a2 = equation[0] * equation[0];
    float a3 = equation[0] * equation[1];
    float a4 = equation[2] * equation[0];
    float a5 = equation[2] * equation[1];
    const Pixel &firstPx = getFirstPixel();
    int Px = firstPx.x;
    int Py = firstPx.y;
    endpoints[0] = a1 * Px - a3 * Py - a4;  // x
    endpoints[1] = a2 * Py - a3 * Px - a5;  // y
    const Pixel &secondPx = getLastPixel();
    Px = secondPx.x;
    Py = secondPx.y;
    endpoints[2] = a1 * Px - a3 * Py - a4;  // x
    endpoints[3] = a2 * Py - a3 * Px - a5;  // y
  }

  ImageEdge FullSegmentInfo::getPixels() const
  {
    assert(firstPxIndex < lastPxIndex);
    ImageEdge result;
    result.reserve(getNumOfPixels());
    for (const Pixel &px: *this)
      result.push_back(px);
    return result;
  }

  void FullSegmentInfo::removeLastPx(bool removeFromTheEnd)
  {
    subtractPointFromModel(removeFromTheEnd ? lastPx : firstPx);
    if (removeFromTheEnd) 
    {
      assert(prevLastPx.x != -1);
      lastPx = prevLastPx;
      prevLastPx.x = -1;
      prevLastPx.y = -1;
    }
    else
    {
      assert(prevFirstPx.x != -1);
      firstPx = prevFirstPx;
      prevFirstPx.x = -1;
      prevFirstPx.y = -1;
    }
    lastPxIndex--;
  }

  inline void FullSegmentInfo::leastSquareLineFit(const std::vector<Pixel> &pts, int startIdx)
  {
    int i, indpCoord, depCoord;
    sum_x_i = 0, sum_y_i = 0, sum_x_i_y_i = 0, sum_x_i_2 = 0;
    N = pts.size() - startIdx;
    if (isHorizontal) 
    {
      for (i = startIdx; i < pts.size(); i++) 
      {
        indpCoord = pts[i].x;
        depCoord = pts[i].y;
        sum_x_i += indpCoord;
        sum_y_i += depCoord;
        sum_x_i_2 += indpCoord * indpCoord;
        sum_x_i_y_i += indpCoord * depCoord;
      }
    }
    else
    {
      for (i = startIdx; i < pts.size(); i++)
      {
        indpCoord = pts[i].y;
        depCoord = pts[i].x;
        sum_x_i += indpCoord;
        sum_y_i += depCoord;
        sum_x_i_2 += indpCoord * indpCoord;
        sum_x_i_y_i += indpCoord * depCoord;
      }
    }
    calculateLineEq();
  }

  inline void FullSegmentInfo::leastSquaresLineFitNewPoint(int x, int y)
  {
    const int indpCoord = isHorizontal ? x : y;
    const int depCoord = isHorizontal ? y : x;
    N++;
    sum_x_i += indpCoord;
    sum_y_i += depCoord;
    sum_x_i_2 += indpCoord * indpCoord;
    sum_x_i_y_i += indpCoord * depCoord;
    calculateLineEq();
  }

  inline void FullSegmentInfo::subtractPointFromModel(const Pixel &p)
  {
    const int independentCord = isHorizontal ? p.x : p.y;
    const int dependentCord = isHorizontal ? p.y : p.x;
    N--;
    sum_x_i -= independentCord;
    sum_y_i -= dependentCord;
    sum_x_i_2 -= independentCord * independentCord;
    sum_x_i_y_i -= independentCord * dependentCord;
    calculateLineEq();
  }

  inline void FullSegmentInfo::calculateLineEq()
  {
    equation[2] = sum_y_i * sum_x_i_2 - sum_x_i * sum_x_i_y_i;
    if (isHorizontal) 
    {
      equation[0] = N * sum_x_i_y_i - sum_x_i * sum_y_i;
      equation[1] = sum_x_i * sum_x_i - N * sum_x_i_2;
    }
    else
    {
      equation[1] = N * sum_x_i_y_i - sum_x_i * sum_y_i;
      equation[0] = sum_x_i * sum_x_i - N * sum_x_i_2;
    }
    equation *= (1 / std::sqrt(equation[0] * equation[0] + equation[1] * equation[1]));
    if (dx * -equation[1] + dy * equation[0] < 0) 
    {
      equation[0] = -equation[0];
      equation[1] = -equation[1];
      equation[2] = -equation[2];
    }
  }
}