
#include "omnicamera/LadySaw.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using std::find;
using std::vector;
using cv::Mat;
using cv::Size;

namespace omnicamera {

LadySaw::LadySaw() : camsIn(vector<int>(6)), camsOut(vector<int>(6)), rotate(true), width(1232), height(1616) {
  camsIn.resize(6);
  camsOut.resize(6);
  for (int i = 0; i < 6; i++) {
    camsIn[i] = i;
    camsOut[i] = i;
  }
}
LadySaw::~LadySaw() {
}

void LadySaw::saw(const Mat frame, vector<Mat> &cams) {

  // Avoid resizing matrix buffers if not necessary.
  if (cams.size() != camsOut.size()) {
    cams.resize(camsOut.size());
  }

  int camRows = frame.rows / camsIn.size();
  for (int i = 0; i < (int) camsOut.size(); i++) {
    vector<int>::iterator it = find(camsIn.begin(), camsIn.end(), camsOut[i]);
    if (it == camsIn.end()) {
      // Element not found.
      cams[i].create(0, 0, frame.type());
      continue;
    }
    int iCamIn = it - camsIn.begin();
    Mat camRoi = frame(cv::Rect(0, iCamIn * camRows, frame.cols, camRows));
    // Rotate 90 deg. clock-wise if requested.
    if (rotate) {
      camRoi = camRoi.t();
      cv::flip(camRoi, camRoi, 1); // Flip horizontally.
    }

    // Resize if necessary and copy to output.
    if (width != 0 && height != 0 && (camRoi.rows != height || camRoi.cols != width)) {
      cv::resize(camRoi, cams[i], Size(width, height), 0, 0, CV_INTER_LINEAR);
    } else {
      camRoi.copyTo(cams[i]);
    }
  }
}

vector<int> LadySaw::getCamsIn() {
  return camsIn;
}
void LadySaw::setCamsIn(const vector<int> camsIn) {
  this->camsIn = camsIn;
}
vector<int> LadySaw::getCamsOut() {
  return camsOut;
}
void LadySaw::setCamsOut(const vector<int> camsOut) {
  this->camsOut = camsOut;
}

bool LadySaw::isRotate() {
  return rotate;
}
void LadySaw::setRotate(bool rotate) {
  this->rotate = rotate;
}
int LadySaw::getWidth() {
  return width;
}
void LadySaw::setWidth(int width) {
  this->width = width;
}
int LadySaw::getHeight() {
  return height;
}
void LadySaw::setHeight(int height) {
  this->height = height;
}

} // namespace omnicamera
