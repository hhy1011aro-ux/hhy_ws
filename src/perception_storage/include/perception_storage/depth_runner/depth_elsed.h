#ifndef DEPTH_ELSED_H_
#define DEPTH_ELSED_H_

#include <ostream>
#include "perception_storage/depth_runner/depth_fullsegment.h"
#include "perception_storage/depth_runner/depth_edgedrawer.h"

namespace depth_runner 
{
  struct ELSEDParams 
  {
    // Gaussian kernel size
    int ksize = 5;
    // Sigma of the gaussian kernel
    float sigma = 1;
    float gradientThreshold = 30;
    uint8_t anchorThreshold = 8;
    unsigned int scanIntervals = 2;
    int minLineLen = 15;
    double lineFitErrThreshold = 0.2;
    double pxToSegmentDistTh = 1.5;
    double junctionEigenvalsTh = 10;
    double junctionAngleTh = 10 * (M_PI / 180.0);
    double validationTh = 0.15;
    bool validate = true;
    bool treatJunctions = true;
    std::vector<int> listJunctionSizes = {5, 7, 9};
  };

  class ELSED 
  {
  public:
    explicit ELSED(const ELSEDParams &params = ELSEDParams());
    Segments detect(const cv::Mat &image);
    SalientSegments detectSalient(const cv::Mat &image);
    ImageEdges detectEdges(const cv::Mat &image);
    const LineDetectionExtraInfo &getImgInfo() const;
    const LineDetectionExtraInfoPtr &getImgInfoPtr() const;
    void processImage(const cv::Mat &image);
    void clear();
    static void computeAnchorPoints(const cv::Mat &dirImage,
                                    const cv::Mat &gradImageWO,
                                    const cv::Mat &gradImage,
                                    int scanInterval,
                                    int anchorThresh,
                                    std::vector<Pixel> &anchorPoints);
    static LineDetectionExtraInfoPtr
    computeGradients(const cv::Mat &srcImg, short gradientTh);
    ImageEdges getAllEdges() const;
    ImageEdges getSegmentEdges() const;
    const EdgeDrawerPtr &getDrawer() const { return drawer; }
  private:
    void drawAnchorPoints(const uint8_t *dirImg,
                          const std::vector<Pixel> &anchorPoints,
                          uint8_t *pEdgeImg);

    ELSEDParams params;
    LineDetectionExtraInfoPtr imgInfo;
    ImageEdges edges;
    Segments segments;
    SalientSegments salientSegments;
    std::vector<Pixel> anchors;
    EdgeDrawerPtr drawer;
    cv::Mat blurredImg;
    cv::Mat edgeImg;
  };

}

#endif 
