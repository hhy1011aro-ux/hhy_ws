#include <utility>
#include <memory>
#include "perception_storage/depth_runner/depth_edgedrawer.h"

namespace depth_runner
{
  template<class T>
  static inline std::vector<T> &
  operator+=(std::vector<T> &lhs, std::vector<T> l) 
  {
    lhs.insert(std::end(lhs), std::begin(l), std::end(l));
    return lhs;
  }

  EdgeDrawer::EdgeDrawer(const LineDetectionExtraInfoPtr &gradientInfo,
                        cv::Mat &edgeImage,
                        float lineFitThreshold,
                        float pxToSegmentDistTh,
                        int minLineLength,
                        bool treatJunctions,
                        std::vector<int> listOfJunctionSizes,
                        double junctionEigenvalsTh,
                        double junctionAngleTh) :
      lineFitThreshold(lineFitThreshold),
      pxToSegmentDistTh(pxToSegmentDistTh),
      minLineLength(minLineLength),
      treatJunctions(treatJunctions),
      listOfJunctionSizes(std::move(listOfJunctionSizes)),
      junctionEigenvalsTh(junctionEigenvalsTh),
      junctionAngleTh(junctionAngleTh) 
  {
    init(gradientInfo, edgeImage);
  }

  void EdgeDrawer::init(const LineDetectionExtraInfoPtr &gradientInfo, cv::Mat &edgeImage) 
  {
    assert(gradientInfo->imageWidth == gradientInfo->gImg.cols);
    assert(gradientInfo->imageHeight == gradientInfo->gImg.rows);
    assert(gradientInfo->imageWidth > 0 && gradientInfo->imageHeight > 0);
    imageWidth = gradientInfo->imageWidth;
    imageHeight = gradientInfo->imageHeight;

    assert(!gradientInfo->gImg.empty());
    assert(gradientInfo->gImg.type() == CV_16SC1);
    gradImg = gradientInfo->gImg.ptr<int16_t>();
    assert(!gradientInfo->dirImg.empty());
    assert(gradientInfo->dirImg.type() == CV_8UC1);
    assert(gradientInfo->dirImg.size() == gradientInfo->gImg.size());
    pDirImg = gradientInfo->dirImg.ptr<uint8_t>();
    assert(!gradientInfo->dxImg.empty());
    assert(gradientInfo->dxImg.type() == CV_16SC1);
    assert(gradientInfo->dxImg.size() == gradientInfo->gImg.size());
    pDxImg = gradientInfo->dxImg.ptr<int16_t>();
    assert(!gradientInfo->dyImg.empty());
    assert(gradientInfo->dyImg.type() == CV_16SC1);
    assert(gradientInfo->dyImg.size() == gradientInfo->gImg.size());
    pDyImg = gradientInfo->dyImg.ptr<int16_t>();

    assert(!edgeImage.empty());
    assert(edgeImage.type() == CV_8UC1);
    assert(edgeImage.size() == gradientInfo->gImg.size());
    edgeImg = edgeImage.ptr<uint8_t>();

    pixels.reserve(0.25 * imageWidth * imageHeight);
    segments.reserve(imageWidth / 2);
    branchesStack.reserve(100);
  }

  inline void EdgeDrawer::addJunctionPixelsToSegment(const ImageEdge &junctionPixels,
                                                    FullSegmentInfo &segment,
                                                    bool addPixelsForTheFirstSide)
  {
    if(addPixelsForTheFirstSide)
    {
      segment.firstEndpointExtended = true;
    }
    else
    {
      segment.secondEndpointExtended = true;
    }
    for(int i = 0; i < junctionPixels.size(); i++)
    {
      const Pixel &junctionPx = junctionPixels[i];
      if(segment.isInlier(junctionPx.x, junctionPx.y, pxToSegmentDistTh))
      {
        if(i < junctionPixels.size() / 2)
        {
          edgeImg[junctionPx.y * imageWidth + junctionPx.x] = UPM_ED_JUNTION_PX;
        }
        else 
        {
          edgeImg[junctionPx.y * imageWidth + junctionPx.x] = UPM_ED_SEGMENT_INLIER_PX;
        }
        pixels.push_back(junctionPx);
        segment.addPixel(junctionPx.x, junctionPx.y, pixels.size() - 1, addPixelsForTheFirstSide);
      }
      else
      {
        edgeImg[junctionPx.y * imageWidth + junctionPx.x] = UPM_ED_SEGMENT_OUTLIER_PX;
      }
    }
  }

  inline Pixel calcLastPixelWithDirection(const Pixel &px, uint8_t lastDir)
  {
    switch (lastDir)
    {
      case UPM_UP:return {px.x, px.y + 1};
      case UPM_DOWN:return {px.x, px.y - 1};
      case UPM_LEFT:return {px.x + 1, px.y};
      case UPM_RIGHT:return {px.x - 1, px.y};
      default: return {-1, -1};
    }
  }

  void EdgeDrawer::drawEdgeTreeStack(Pixel anchor, ImageEdge &initialPixels, bool firstAnchorDirection)
  {
    uint8_t direction, gradDir, lineDirection;
    int i, indexInArray, lastChekedPxIdx, initialPxIndex, indexInImage, nElements;
    bool addPixelsForTheFirstSide, inlierOverwritten, popStack, firstBranch, isAnchorFirstPx, wasExtended;
    bool localSegInitialized, segment;
    Pixel px, lastPx;
    double fitError;
    ImageEdge extensionPixels, outliersList;

    FullSegmentInfo *localSegment = nullptr;
    extensionPixels.reserve(minLineLength);
    segment = false;
    firstBranch = true;
    isAnchorFirstPx = pixels.back() == anchor && firstAnchorDirection;
    segments.emplace_back(pixels);
    localSegment = &segments.back();
    while (!branchesStack.empty())
    {
      popStack = true;
      DrawingBranch &branch = branchesStack.back();
      initialPxIndex = pixels.size();
      if (firstBranch && isAnchorFirstPx) initialPxIndex--;
      direction = branch.direction;
      px = branch.px;
      addPixelsForTheFirstSide = branch.addPixelsForTheFirstSide;
      pixels += branch.pixels;
      lastChekedPxIdx = initialPxIndex;
      outliersList.clear();
      inlierOverwritten = false;
      lastPx = calcLastPixelWithDirection(px, direction);
      gradDir = (direction == UPM_DOWN || direction == UPM_UP) ? UPM_EDGE_VERTICAL : UPM_EDGE_HORIZONTAL;
      localSegInitialized = false;
      while (outliersList.size() <= UPM_MAX_OUTLIERS_TH)
      {
        if(!findNextPxWithGradient(gradDir, gradImg, imageWidth, imageHeight, px, lastPx)) 
        {
          break;
        }
        indexInArray = px.y * imageWidth + px.x;
        inlierOverwritten = edgeImg[indexInArray] == UPM_ED_SEGMENT_INLIER_PX ||
            edgeImg[indexInArray] == UPM_ED_JUNTION_PX ||
            edgeImg[indexInArray] == UPM_ED_EDGE_PIXEL;
        if(inlierOverwritten) 
        {
          break;
        }
        if(segment)
        {
          if(localSegment->isInlier(px.x, px.y, pxToSegmentDistTh))
          {
            edgeImg[indexInArray] = UPM_ED_SEGMENT_INLIER_PX;
            pixels.push_back(px);
            if(!outliersList.empty()) 
            {
              outliersList.clear();
            }
            localSegment->addPixel(px.x, px.y, pixels.size() - 1, addPixelsForTheFirstSide);
            if(addPixelsForTheFirstSide) localSegment->firstEndpointExtended = false;
            else localSegment->secondEndpointExtended = false;
          }
          else
          {
            outliersList.push_back(px);
            edgeImg[indexInArray] = UPM_ED_SEGMENT_OUTLIER_PX;
          }
        }
        else
        {
          pixels.push_back(px);
          edgeImg[indexInArray] = UPM_ED_EDGE_PIXEL;
          if(pixels.size() - lastChekedPxIdx >= minLineLength)
          {
            if(localSegInitialized)
            {
              localSegment->skipPositions();
            }
            else
            {
              localSegInitialized = true;
              localSegment->init(pixels, lastChekedPxIdx);
            }
            fitError = localSegment->getFitError();
            if(fitError < lineFitThreshold) 
            {
              segment = true;
              for(i = lastChekedPxIdx; i < lastChekedPxIdx + minLineLength; i++) 
              {
                edgeImg[pixels[i].y * imageWidth + pixels[i].x] = UPM_ED_SEGMENT_INLIER_PX;
              }
            }
            else
            {
              lastChekedPxIdx += UPM_SKIP_EDGE_PT;
            }
          }
        }
        gradDir = pDirImg[indexInArray];
      }
      if(firstBranch && edgeImg[anchor.y * imageWidth + anchor.x] != UPM_ED_SEGMENT_INLIER_PX) 
      {
        initialPixels.clear();
        nElements = std::min(int(pixels.size() - initialPxIndex), (minLineLength - 1));
        for(i = initialPxIndex + nElements - 1; i >= initialPxIndex; i--)
        {
          indexInImage = pixels[i].y * imageWidth + pixels[i].x;
          if(edgeImg[indexInImage] == UPM_ED_SEGMENT_INLIER_PX || edgeImg[indexInImage] == UPM_ED_JUNTION_PX)
            break;
          initialPixels.push_back(pixels[i]);
        }
      }
      if(segment)
      {
        if(!outliersList.empty())
        {
          if(addPixelsForTheFirstSide || localSegment->hasSecondSideElements())
          {
            Pixel pxToDelete = addPixelsForTheFirstSide ? localSegment->getLastPixel() : localSegment->getFirstPixel();
            edgeImg[pxToDelete.y * imageWidth + pxToDelete.x] = UPM_ED_NO_EDGE_PIXEL;
            localSegment->removeLastPx(addPixelsForTheFirstSide);
            outliersList.insert(outliersList.begin(), pixels.back());
            pixels.pop_back();
            if(!addPixelsForTheFirstSide && localSegment->getFirstPixel() == anchor)
            {
              localSegment->secondEndpointExtended = true;
            }
          }
        }
        if(outliersList.size() > UPM_MAX_OUTLIERS_TH)
        {
          for(const Pixel &outlier: outliersList)
          {
            edgeImg[outlier.y * imageWidth + outlier.x] = UPM_ED_NO_EDGE_PIXEL;
          }
          uint8_t predictedLastDir;
          if(pDirImg[indexInArray] == UPM_EDGE_HORIZONTAL) predictedLastDir = lastPx.x < px.x ? UPM_RIGHT : UPM_LEFT;
          else predictedLastDir = lastPx.y < px.y ? UPM_DOWN : UPM_UP;

          popStack = false;
          branch.direction = predictedLastDir;
          branch.px.x = px.x;
          branch.px.y = px.y;
          branch.addPixelsForTheFirstSide = true;
          branch.pixels.clear();
          branch.pixels.push_back(pixels.back());
          for (Pixel &p: outliersList) branch.pixels.push_back(p);
        }
        wasExtended = localSegment->firstEndpointExtended;
        if((outliersList.size() > UPM_MAX_OUTLIERS_TH || inlierOverwritten) &&
            !wasExtended && treatJunctions && canSegmentBeExtended(*localSegment,
                                                                  true,
                                                                  extensionPixels))
        {
          lineDirection = directionFromLineEq(localSegment->getLineEquation());
          addJunctionPixelsToSegment(extensionPixels, *localSegment, addPixelsForTheFirstSide);
          if(popStack)
          {
            popStack = false;
            branch.direction = lineDirection;
            branch.px = extensionPixels.back();
            branch.addPixelsForTheFirstSide = true;
            branch.pixels.clear();
          }
          else
          {
            branchesStack.emplace_back(DrawingBranch{lineDirection, extensionPixels.back(), true, {}});
          }
        }
        else
        {
          localSegment->firstEndpointExtended = true;
          wasExtended = localSegment->secondEndpointExtended;
          if(!wasExtended && localSegment->getFirstPixel() == anchor)
          {
            uint8_t oppositeLineDirection = inverseDirection(directionFromLineEq(localSegment->getLineEquation()));
            localSegment->secondEndpointExtended = true;
            if(popStack) 
            {
              popStack = false;
              branch.direction = oppositeLineDirection;
              branch.px = anchor;
              branch.addPixelsForTheFirstSide = false;
              branch.pixels.clear();
            }
            else
            {
              branchesStack.emplace_back(DrawingBranch{oppositeLineDirection, anchor, false, {}});
            }
          } 
          else if (!wasExtended && treatJunctions && canSegmentBeExtended(*localSegment,
                                                                            false,
                                                                            extensionPixels))
          {
            addJunctionPixelsToSegment(extensionPixels, *localSegment, false);
            uint8_t opositeLineDirection = inverseDirection(directionFromLineEq(localSegment->getLineEquation()));
            if (popStack)
            {
              popStack = false;
              branch.direction = opositeLineDirection;
              branch.px = extensionPixels.back();
              branch.addPixelsForTheFirstSide = false;
              branch.pixels.clear();
            }
            else
            {
              branchesStack.emplace_back(DrawingBranch{opositeLineDirection, extensionPixels.back(), false, {}});
            }
          }
          else
          {
            localSegment->secondEndpointExtended = true;
            localSegment->finish();
            segments.emplace_back(pixels);
            localSegment = &segments.back();
            segment = false;
          }
        }
      }
      if (popStack)
      {
        branchesStack.resize(branchesStack.size() - 1);
      }
      firstBranch = false;
      extensionPixels.clear();
    }
    segments.pop_back();
  }

  void EdgeDrawer::drawEdgeInBothDirections(uint8_t direction, Pixel anchor)
  {
    assert(anchor.x >= 0 && anchor.x < imageWidth && anchor.y >= 0 && anchor.y < imageHeight);
    assert(gradImg && gradImg[anchor.y * imageWidth + anchor.x] > 0);

    ImageEdge anchorPixels, newBranchPixels;
    edgeImg[anchor.y * imageWidth + anchor.x] = UPM_ED_ANCHOR_PIXEL;
    branchesStack.emplace_back(DrawingBranch{direction, anchor, true, {}});
    pixels.push_back(anchor);
    drawEdgeTreeStack(anchor, anchorPixels, true);

    uint8_t oppositeDirection = inverseDirection(direction);
    branchesStack.emplace_back(DrawingBranch{oppositeDirection, anchor, true, anchorPixels});
    drawEdgeTreeStack(anchor, anchorPixels, false);
  }

  inline bool EdgeDrawer::findNextPxWithGradient(uint8_t pxGradDirection,
                                                const int16_t *gradImg,
                                                int imageWidth, int imageHeight,
                                                Pixel &px,
                                                Pixel &lastPx) 
  {

    int16_t gValue1, gValue2, gValue3;
    int lastX, lastY, indexInArray;

    indexInArray = px.y * imageWidth + px.x;
    lastX = lastPx.x;
    lastY = lastPx.y;
    lastPx = px;

    if (pxGradDirection == UPM_EDGE_HORIZONTAL)
    {
      // | Ok | X | Ok |
      // | Ok | X | Ok |
      // | Ok | X | Ok |
      if (lastX < px.x)
      {
        // | X | X | Ok |
        // | X | X | Ok |
        // | X | X | Ok |
        if (px.x == imageWidth - 1 || px.y == 0 || px.y == imageHeight - 1)
        {
          return false;
        }
        indexInArray++;
        px.x++;
        gValue1 = gradImg[indexInArray - imageWidth];
        gValue2 = gradImg[indexInArray];
        gValue3 = gradImg[indexInArray + imageWidth];
        if (gValue1 > gValue2 && gValue1 > gValue3)
        {
          px.y = px.y - 1;
        } 
        else if (gValue3 > gValue2 && gValue3 > gValue1)
        {
          px.y = px.y + 1;
        }
      } 
      else if (lastX > px.x)
      {
        // | Ok | X | X |
        // | Ok | X | X |
        // | Ok | X | X |
        if (px.x == 0 || px.y == 0 || px.y == imageHeight - 1)
        {
          return false;
        }
        indexInArray--;
        px.x--;
        gValue1 = gradImg[indexInArray - imageWidth];
        gValue2 = gradImg[indexInArray];
        gValue3 = gradImg[indexInArray + imageWidth];
        if (gValue1 > gValue2 && gValue1 > gValue3) 
        {
          px.y = px.y - 1;
        } 
        else if (gValue3 > gValue2 && gValue3 > gValue1)
        {
          px.y = px.y + 1;
        }
      } 
      else if (lastY < px.y)
      { 
        // | X  | X | X  |
        // | X  | X | X  |
        // | Ok | X | Ok |
        if(px.y == imageHeight - 1 || px.x == 0 || px.x == imageWidth - 1)
        {
          return false;
        }
        indexInArray += imageWidth;
        px.y++;
        if(gradImg[indexInArray - 1] > gradImg[indexInArray + 1])
        {
          px.x--;
        } 
        else 
        {
          px.x++;
        }
      }
      else 
      { 
        // | Ok | X | Ok |
        // | X  | X | X  |
        // | X  | X | X  |
        if(px.y == 0 || px.x == 0 || px.x == imageWidth - 1)
        {
          return false;
        }
        indexInArray -= imageWidth;
        px.y--;
        if(gradImg[indexInArray - 1] > gradImg[indexInArray + 1])
        {
          px.x--;
        }
        else
        {
          px.x++;
        }
      }
    }
    else
    {
      // | Ok | Ok | Ok |
      // |  X |  X |  X |
      // | Ok | Ok | Ok |
      if(lastY < px.y)
      {
        // |  X |  X |  X |
        // |  X |  X |  X |
        // | Ok | Ok | Ok |
        if(px.y == imageHeight - 1 || px.x == 0 || px.x == imageWidth - 1)
        {
          return false;
        }
        indexInArray += imageWidth;
        px.y++;
        gValue1 = gradImg[indexInArray + 1];
        gValue2 = gradImg[indexInArray];
        gValue3 = gradImg[indexInArray - 1];
        if(gValue1 > gValue2 && gValue1 > gValue3) 
        {
          px.x = px.x + 1;
        }
        else if(gValue3 > gValue2 && gValue3 > gValue1) 
        {
          px.x = px.x - 1;
        }
      }
      else if(lastY > px.y)
      {
        // | Ok | Ok | Ok |
        // |  X |  X |  X |
        // |  X |  X |  X |
        if(px.y == 0 || px.x == 0 || px.x == imageWidth - 1)
        {
          return false;
        }
        indexInArray -= imageWidth;
        px.y--;
        gValue1 = gradImg[indexInArray + 1];
        gValue2 = gradImg[indexInArray];
        gValue3 = gradImg[indexInArray - 1];
        if(gValue1 > gValue2 && gValue1 > gValue3) 
        {
          px.x = px.x + 1;
        } 
        else if(gValue3 > gValue2 && gValue3 > gValue1) 
        {
          px.x = px.x - 1;
        }
      }
      else if(lastX < px.x) 
      {
        // |  X |  X | Ok |
        // |  X |  X |  X |
        // |  X |  X | Ok |
        if(px.x == imageWidth - 1 || px.y == 0 || px.y == imageHeight - 1)
        {
          return false;
        }
        indexInArray++;
        px.x++;
        if(gradImg[indexInArray - imageWidth] > gradImg[indexInArray + imageWidth])
        {
          px.y--;
        }
        else
        {
          px.y++;
        }
      }
      else
      {
        // | Ok |  X |  X |
        // |  X |  X |  X |
        // | Ok |  X |  X |
        if(px.x == 0 || px.y == 0 || px.y == imageHeight - 1) 
        {
          return false;
        }
        indexInArray--;
        px.x--;
        if(gradImg[indexInArray - imageWidth] > gradImg[indexInArray + imageWidth])
        {
          px.y--;
        } 
        else
        {
          px.y++;
        }
      }
    }
    return gradImg[px.y * imageWidth + px.x];
  }

  bool EdgeDrawer::canSegmentBeExtended(FullSegmentInfo &segment,
                                        bool extendByTheEnd,
                                        ImageEdge &pixelsInTheExtension)
  {
    bool inlier, found, horizontalValidationDir, fitsEigenvaluesCond, fitsAngleCond;
    int i, validateDx, validateDy, xOffset, yOffset, indexInArray;
    double theta;
    float a, b, d, tmp, s1, s2, eigen_angle;
    Pixel firstPx, lastPx, px;
    ImageEdge pixelsToValidate;
    cv::Vec3d eq;
    firstPx = extendByTheEnd ? segment.getLastPixel() : segment.getFirstPixel();
    for (int GRAD_JUNCTION_SIZE: listOfJunctionSizes) 
    {
      if (segment.getNumOfPixels() <= GRAD_JUNCTION_SIZE) break;
      px = firstPx;
      found = findNextPxWithProjection(gradImg,
                                      segment.getLineEquation(),
                                      !extendByTheEnd,
                                      imageWidth,
                                      imageHeight,
                                      GRAD_JUNCTION_SIZE,
                                      px);
      if(!found)
      {
        continue;
      }
      pixelsInTheExtension = depth_runner::bresenham(firstPx.x, firstPx.y, px.x, px.y);
      lastPx = pixelsInTheExtension[pixelsInTheExtension.size() - 2];
      for(i = 0; i < GRAD_JUNCTION_SIZE; i++)
      {
        if(!findNextPxWithGradient(segment.horizontal() ? UPM_EDGE_HORIZONTAL : UPM_EDGE_VERTICAL,
                                    gradImg,
                                    imageWidth, imageHeight,
                                    px,
                                    lastPx))
        {
          pixelsInTheExtension.clear();
          break;
        }

        inlier = segment.isInlier(px.x, px.y, pxToSegmentDistTh);
        if (inlier) 
        {
          pixelsInTheExtension.push_back(px);
          pixelsToValidate.push_back(px);
        } 
        else 
        {
          pixelsInTheExtension.clear();
          break;
        }
      }
      if (pixelsInTheExtension.empty()) continue;

        eq = segment.getLineEquation();
        theta = std::atan2(eq[0], eq[1]) + M_PI_2;
        while (theta < 0) theta += M_PI;
        while (theta >= M_PI) theta -= M_PI;

        a = 0, b = 0, d = 0;
        validateDx = std::abs(pixelsToValidate.back().x - pixelsToValidate.front().x);
        validateDy = std::abs(pixelsToValidate.back().y - pixelsToValidate.front().y);
        horizontalValidationDir = validateDx >= validateDy;
        for (Pixel &extPixel: pixelsToValidate) 
        {
          for (int offset: {-1, 0, 1}) 
          {
            xOffset = horizontalValidationDir ? 0 : offset;
            yOffset = horizontalValidationDir ? offset : 0;
            indexInArray = std::min(imageHeight-1,
                                    std::max(0, extPixel.y + yOffset))*imageWidth+std::min(imageWidth-1,
                                    std::max(0, extPixel.x + xOffset));
            a += pDxImg[indexInArray] * pDxImg[indexInArray];
            b += pDxImg[indexInArray] * pDyImg[indexInArray];
            d += pDyImg[indexInArray] * pDyImg[indexInArray];
          }
        }
        tmp = a * a - d * d;
        s1 = a * a + 2 * b * b + d * d;
        s2 = std::sqrt(tmp * tmp + 4 * b * b * (a + d) * (a + d));
        eigen_angle = -0.5f * std::atan2(2 * a * b + 2 * b * d, tmp);
        while (eigen_angle < 0) eigen_angle += M_PI;
        while (eigen_angle >= M_PI) eigen_angle -= M_PI;
        fitsEigenvaluesCond = std::sqrt((s1 + s2) / (s1 - s2 + 0.00001)) > junctionEigenvalsTh;
        fitsAngleCond = circularDist(theta, eigen_angle, M_PI) < junctionAngleTh;
        if (!fitsEigenvaluesCond || !fitsAngleCond) 
        {
          continue;
        }
      return true;
    }
    return false;
  }

  inline uint8_t EdgeDrawer::inverseDirection(uint8_t dir)
  {
    switch (dir) 
    {
      case UPM_RIGHT:return UPM_LEFT;
      case UPM_LEFT: return UPM_RIGHT;
      case UPM_UP: return UPM_DOWN;
      default: return UPM_UP; 
    }
  }

  inline uint8_t EdgeDrawer::directionFromLineEq(const cv::Vec3d &eq) 
  {
    double lineX = -eq[1];
    double lineY = eq[0];
    if (UPM_ABS(lineX) > UPM_ABS(lineY)) 
    {
      if (lineX > 0) return UPM_RIGHT;
      else return UPM_LEFT;
    } 
    else 
    {
      if (lineY > 0) return UPM_DOWN;
      else return UPM_UP;
    }
  }

  Segments EdgeDrawer::getDetectedSegments() const
  {
    Segments result(segments.size());
    for (int i = 0; i < segments.size(); i++) result[i] = segments[i].getEndpoints();
    return result;
  }

  bool EdgeDrawer::findNextPxWithProjection(const int16_t *gradImg,
                                            cv::Vec3f eq,
                                            bool invertLineDir,
                                            int imageWidth, int imageHeight,
                                            int stepSize,
                                            Pixel &px) 
  {
    assert(px.x >= 0 && px.x < imageWidth && px.y >= 0 && px.y < imageHeight);
    int16_t gValue1, gValue2, gValue3, gValue4;
    cv::Point2f extendedPoint, lasPxReproj;
    lasPxReproj = getProjectionPtn(eq, cv::Point2f(px.x, px.y));
    if (!invertLineDir)
    {
      extendedPoint.x = lasPxReproj.x - stepSize * eq[1];
      extendedPoint.y = lasPxReproj.y + stepSize * eq[0];
    }
    else
    {
      extendedPoint.x = lasPxReproj.x + stepSize * eq[1];
      extendedPoint.y = lasPxReproj.y - stepSize * eq[0];
    }
    if(extendedPoint.x < 0 || extendedPoint.x >= imageWidth || extendedPoint.y < 0 || extendedPoint.y >= imageHeight)
    {
      return false;
    }
    int x = extendedPoint.x;
    int y = extendedPoint.y;
    gValue1 = gradImg[y * imageWidth + x];
    gValue2 = gradImg[y * imageWidth + std::min(x + 1, imageWidth - 1)];
    gValue3 = gradImg[std::min(y + 1, imageHeight - 1) * imageWidth + x];
    gValue4 = gradImg[std::min(y + 1, imageHeight - 1) * imageWidth + std::min(x + 1, imageWidth - 1)];

    if(gValue2 > gValue1)
    {
      if(gValue3 > gValue2)
      {
        if(gValue4 > gValue3)
        {
          px.x = std::min(x + 1, imageWidth);
          px.y = std::min(y + 1, imageHeight);
          return gValue4;
        }
        else
        {
          px.x = x;
          px.y = std::min(y + 1, imageHeight);
          return gValue3;
        }
      }
      else
      {
        if(gValue4 > gValue2) 
        {
          px.x = std::min(x + 1, imageWidth);
          px.y = std::min(y + 1, imageHeight);
          return gValue4;
        }
        else
        {
          px.x = std::min(x + 1, imageWidth);
          px.y = y;
          return gValue2;
        }
      }
    }
    else
    {
      if(gValue3 > gValue1)
      {
        if(gValue4 > gValue3)
        {
          px.x = std::min(x + 1, imageWidth);
          px.y = std::min(y + 1, imageHeight);
          return gValue4;
        }
        else
        {
          px.x = x;
          px.y = std::min(y + 1, imageHeight);
          return gValue3;
        }
      }
      else
      {
        if(gValue4 > gValue1) 
        {
          px.x = std::min(x + 1, imageWidth);
          px.y = std::min(y + 1, imageHeight);
          return gValue4;
        }
        else
        {
          px.x = x;
          px.y = y;
          return gValue1;
        }
      }
    }
  }
}