
#include "omnicamera/LookupStitcher.h"

#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <libpng/png.h>

using cv::Mat;
using cv::MatIterator_;
using cv::Scalar;
using cv::Size;

namespace omnicamera {

LookupStitcher::LookupStitcher() {
}
LookupStitcher::~LookupStitcher() {
}

void LookupStitcher::clearLookupTables() {
  lutCount = 0;
  lutX.clear();
  lutY.clear();
  lutA.clear();
}

void LookupStitcher::addLookupTables(Mat xMat, Mat yMat, Mat aMat) {
  lutCount++;
  lutX.push_back(xMat);
  lutY.push_back(yMat);
  lutA.push_back(aMat);
}

void LookupStitcher::addLookupTables(const std::string xPath, const std::string yPath, const std::string aPath) {
  Mat xMat = cv::imread(xPath, -1);
  Mat yMat = cv::imread(yPath, -1);
  Mat aMat = cv::imread(aPath, -1);
  // Mat xMat = cv::imread(xPath, 0); seems not to work properly for 16bit images.
  addLookupTables(xMat, yMat, aMat);
}

void LookupStitcher::stitchImage(const cv::Mat &src, cv::Mat &image) {
  // Try using at method which may not be the fastest...
  int rows = lutX[0].rows;
  int cols = lutX[0].cols;
  int channels = src.channels();
  // Assuming image is uint8 0-255 and alpha uint8 0-255, uint16 may be used for accumulators.
  Mat temp(Size(cols, rows), CV_16UC3);
  temp.setTo(Scalar(0, 0, 0));
  for (int i = 0; i < getLutCount(); i++) {
    for (int r = 0; r < rows; r++) {
      for (int c = 0; c < cols; c++) {
        ushort a = (ushort) lutA[i].at<uchar>(r, c);
        if (a == 0) {
          continue;
        }
        ushort x = lutX[i].at<ushort>(r, c);
        ushort y = lutY[i].at<ushort>(r, c);
        for (int ch = 0; ch < channels; ch++) {
          ushort addend = a * (ushort) src.at<cv::Vec3b>(y, x)[ch];
          temp.at<cv::Vec3w>(r, c)[ch] = temp.at<cv::Vec3w>(r, c)[ch] + addend;
        }
      }
    }
  }
  temp.convertTo(image, CV_8UC3, 255.0/65535.0);
}

bool LookupStitcher::fixBounds(const int width, const int height) {
  bool anyFix = false;
  if (getLutCount() < 1) {
    return anyFix;
  }
  int rows = lutX[0].rows;
  int cols = lutY[0].cols;
  for (int i = 0; i < getLutCount(); i++) {
    for (int r = 0; r < rows; r++) {
      for (int c = 0; c < cols; c++) {
        if (lutX[i].at<ushort>(r, c) < 0 || lutY[i].at<ushort>(r, c) < 0
            || lutX[i].at<ushort>(r, c) > width - 1 || lutY[i].at<ushort>(r, c) > height - 1) {
          anyFix = true;
          lutA[i].at<uchar>(r, c) = 0;
          lutX[i].at<ushort>(r, c) = 0;
          lutY[i].at<ushort>(r, c) = 0;
        }
      }
    }
  }
  return anyFix;
}


int LookupStitcher::getLutCount() const {
  return lutCount;
}

vector<Mat> LookupStitcher::getLutX() {
  return lutX;
}
vector<Mat> LookupStitcher::getLutY() {
  return lutY;
}
vector<Mat> LookupStitcher::getLutA() {
  return lutA;
}

} // namespace omnicamera
