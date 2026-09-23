#ifndef DEPTH_EDGEDRAWER_H_
#define DEPTH_EDGEDRAWER_H_

#include <stack>
#include <memory>
#include "perception_storage_ros2/depth_runner/depth_utils.h"
#include "perception_storage_ros2/depth_runner/depth_fullsegment.h"

namespace depth_runner 
{
  struct DrawingBranch 
  {
    uint8_t direction;
    Pixel px;
    bool addPixelsForTheFirstSide;
    ImageEdge pixels;
  };
  class EdgeDrawer 
  {
  public:
    EdgeDrawer() = default;
    EdgeDrawer(const LineDetectionExtraInfoPtr &gradientInfo,
              cv::Mat &edgeImage,
              float lineFitThreshold = 0.2,
              float pxToSegmentDistTh = 1.5,
              int minLineLength = 15,
              bool treatJunctions = true,
              std::vector<int> mListOfJunctionSizes = {5, 7, 9},
              double junctionEigenvalsTh = 10,
              double junctionAngleTh = 10 * (M_PI / 180.0));

    void init(const LineDetectionExtraInfoPtr &gradientInfo, cv::Mat &edgeImage);

    void drawEdgeInBothDirections(uint8_t direction, Pixel anchor);

    inline const std::vector<FullSegmentInfo> &getDetectedFullSegments() const 
    {
      return segments;
    }

    Segments getDetectedSegments() const;

    const ImageEdge &getPixels() const 
    { 
      return pixels;
    }

  private:
    inline void addJunctionPixelsToSegment(const ImageEdge &edgePixels,
                                          FullSegmentInfo &segment,
                                          bool addPixelsForTheFirstSide);

    inline static bool findNextPxWithProjection(const int16_t *gradImg,
                                                cv::Vec3f eq,
                                                bool invertLineDir,
                                                int imageWidth, int imageHeight,
                                                int stepSize,
                                                Pixel &px);

    inline static bool findNextPxWithGradient(uint8_t pxGradDirection,
                                              const int16_t *gradImg,
                                              int imageWidth, int imageHeight,
                                              Pixel &px,
                                              Pixel &lastPx);
    bool canSegmentBeExtended(FullSegmentInfo &segment,
                              bool extendByTheEnd,
                              ImageEdge &pixelsInTheExtension);

    static inline uint8_t inverseDirection(uint8_t dir);
    static inline uint8_t directionFromLineEq(const cv::Vec3d &vec);
    void drawEdgeTreeStack(Pixel anchor, ImageEdge &initialPixels, bool firstAnchorDirection);

    std::vector<FullSegmentInfo> segments;
    std::vector<DrawingBranch> branchesStack;
    ImageEdge pixels;
    const int16_t *gradImg;
    const int16_t *pDxImg;
    const int16_t *pDyImg;
    const uint8_t *pDirImg;
    uint8_t *edgeImg;
    int imageWidth, imageHeight;
    float lineFitThreshold = 0.2;
    float pxToSegmentDistTh = 1.5;
    int minLineLength = 15;
    bool treatJunctions = true;
    std::vector<int> listOfJunctionSizes = {5, 7, 9};
    double junctionEigenvalsTh = 10;
    double junctionAngleTh = 10 * (M_PI / 180.0);
  };
  typedef std::shared_ptr<EdgeDrawer> EdgeDrawerPtr;
}

#endif
