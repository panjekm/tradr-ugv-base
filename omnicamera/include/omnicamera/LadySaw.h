
#ifndef OMNICAMERA_LADYSAW_H_
#define OMNICAMERA_LADYSAW_H_

#include <opencv2/core/core.hpp>

using std::vector;
using cv::Mat;

namespace omnicamera {

/**
 * Cuts ladybug frame into separate images, fixing aspect ratio to 4x3 (for images with binning).
 */
class LadySaw {

public:
  LadySaw();
  virtual ~LadySaw();
  void saw(const Mat frame, vector<Mat> &cams);
  vector<int> getCamsIn();
  void setCamsIn(const vector<int> camsIn);
  vector<int> getCamsOut();
  void setCamsOut(const vector<int> camsOut);
  bool isRotate();
  void setRotate(bool rotate);
  int getWidth();
  void setWidth(int width);
  int getHeight();
  void setHeight(int height);

private:
  vector<int> camsIn;
  vector<int> camsOut;
  bool rotate;
  int width;
  int height;

};

} // namespace omnicamera

#endif /* OMNICAMERA_LADYSAW_H_ */
