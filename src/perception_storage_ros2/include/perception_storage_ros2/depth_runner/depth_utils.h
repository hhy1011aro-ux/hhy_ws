#ifndef DEPTH_UTILS_H_
#define DEPTH_UTILS_H_

#include <ostream>
#include <opencv2/opencv.hpp>

#define UPM_ABS(x) ((x) >= 0 ? (x) : -(x))
#define UPM_EDGE_VERTICAL   0
#define UPM_EDGE_HORIZONTAL 255
#define UPM_ED_NO_EDGE_PIXEL 0
#define UPM_ED_EDGE_PIXEL    255
#define UPM_ED_ANCHOR_PIXEL  204
#define UPM_ED_SEGMENT_INLIER_PX 153
#define UPM_ED_SEGMENT_OUTLIER_PX 102
#define UPM_ED_JUNTION_PX 152
#define UPM_LEFT  1
#define UPM_RIGHT 2
#define UPM_UP    3
#define UPM_DOWN  4
#define UPM_SKIP_EDGE_PT 2
#define UPM_MAX_OUTLIERS_TH 3

namespace depth_runner 
{
  typedef cv::Vec4f Segment;
  typedef std::vector<Segment> Segments;

  struct LineDetectionExtraInfo 
  {
    cv::Mat dxImg;
    cv::Mat dyImg;
    cv::Mat gImgWO;
    cv::Mat gImg;
    cv::Mat dirImg;
    unsigned int imageWidth;
    unsigned int imageHeight;
  };

  typedef std::shared_ptr<LineDetectionExtraInfo> LineDetectionExtraInfoPtr;

  struct Pixel 
  {
    int x;
    int y;

    Pixel() = default;
    Pixel(int x, int y) : x(x), y(y) {}
    explicit Pixel(const cv::Point &p) : x(p.x), y(p.y) {}

    inline bool operator==(const Pixel &rhs) const 
    {
      return x == rhs.x && y == rhs.y;
    }
    inline bool operator!=(const Pixel &rhs) const 
    {
      return !(rhs == *this);
    }

    inline bool operator<(const Pixel &px2) const 
    {
      return (x + y) < (px2.x + px2.y);
    }
  };

  typedef std::vector<Pixel> ImageEdge;
  typedef std::vector<ImageEdge> ImageEdges;

  struct SalientSegment 
  {
    Segment segment;
    double salience;

    SalientSegment() = default;
    SalientSegment(const Segment &segment, double salience) : segment(segment), salience(salience) {}

    inline bool operator<(const SalientSegment &rhs) const 
    {
      if (salience == rhs.salience) {
        float dx1 = segment[0] - segment[2];
        float dx2 = rhs.segment[0] - rhs.segment[2];
        float dy1 = segment[1] - segment[3];
        float dy2 = rhs.segment[1] - rhs.segment[3];
        return std::sqrt(dx1 * dx1 + dy1 * dy1) > std::sqrt(dx2 * dx2 + dy2 * dy2);
      } 
      else 
      {
        return salience > rhs.salience;
      }
    }
  };

  typedef std::vector<SalientSegment> SalientSegments;

  inline double circularDist(double valueA, double valueB, double mod = 360) 
  {
    double a, b;
    if (valueA < valueB) 
    {
      a = valueA;
      b = valueB;
    } 
    else 
    {
      a = valueB;
      b = valueA;
    }

    double dist_clockwise, dist_no_clockwise;
    dist_clockwise = b - a;
    dist_no_clockwise = a + (mod - b);
    return std::min(dist_clockwise, dist_no_clockwise);
  }

  inline cv::Point2f getProjectionPtn(const cv::Vec3f &l, const cv::Point2f &p) 
  {
    const cv::Vec3f homoP(p.x, p.y, 1);
    if (l.dot(homoP) == 0) 
    {
      return p;
    }
    const cv::Vec2f v2(l[0], l[1]);
    const cv::Vec3f r2(v2[1], -v2[0], v2[0] * p.y - v2[1] * p.x);
    const cv::Vec3f p2 = l.cross(r2);
    return cv::Point2f(p2[0] / p2[2], p2[1] / p2[2]);
  }

  inline float segLength(const Segment &s) 
  {
    const float dx = s[0] - s[2];
    const float dy = s[1] - s[3];
    return std::sqrt(dx * dx + dy * dy);
  }

  inline float segAngle(const Segment &s) 
  {
    if (s[2] > s[0])
      return std::atan2(s[3] - s[1], s[2] - s[0]);
    else
      return std::atan2(s[1] - s[3], s[0] - s[2]);
  }

  static std::vector<Pixel> bresenham(int x0, int y0, int x1, int y1) 
  {
    int dx, dy, p, x, y, xIncrement, yIncrement;
    std::vector<Pixel> pixels;
    dx = x1 - x0;
    dy = y1 - y0;

    xIncrement = dx < 0 ? -1 : +1;
    yIncrement = dy < 0 ? -1 : +1;

    x = x0;
    y = y0;
    dx = UPM_ABS(dx);
    dy = UPM_ABS(dy);

    if (dx >= dy) 
    {
      p = 2 * dy - dx;
      while (x != x1) 
      {
        pixels.emplace_back(x, y);
        if (p >= 0) 
        {
          y += yIncrement;
          p += 2 * dy - 2 * dx;
        } 
        else 
        {
          p += 2 * dy;
        }
        x += xIncrement;
      }
    } 
    else 
    {
      p = 2 * dx - dy;
      while (y != y1) 
      {
        pixels.emplace_back(x, y);
        if (p >= 0) 
        {
          x += xIncrement;
          p += +2 * dx - 2 * dy;
        } 
        else 
        {
          p += 2 * dx;
        }
        y += yIncrement;
      }
    }
    pixels.emplace_back(x1, y1);
    return pixels;
  }
}

#endif
