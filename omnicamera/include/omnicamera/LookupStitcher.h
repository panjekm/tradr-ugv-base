
#ifndef OMNICAMERA_LOOKUPSTITCHER_H_
#define OMNICAMERA_LOOKUPSTITCHER_H_

#include <opencv2/core/core.hpp>

using std::string;
using std::vector;
using cv::Mat;

namespace omnicamera {

/**
 * Image stitcher based on multiple look-up tables.
 */
class LookupStitcher {

public:
  LookupStitcher();
  virtual ~LookupStitcher();
  void clearLookupTables();
  void addLookupTables(Mat x, Mat y, Mat a);
  void addLookupTables(const string x, const string y, const string a);
  void stitchImage(const Mat &src, Mat &image);

  /**
   * Fixes values in look-up tables to be within bounds of the input image,
   * x in <0, width - 1>, y in <0, height - 1>.
   * For values out of bounds, zero alpha is used. Total alpha then does not sum up to one.
   *
   * @param width
   * @param height
   */
  bool fixBounds(const int width, const int height);

  int getLutCount() const;
  vector<Mat> getLutX();
  vector<Mat> getLutY();
  vector<Mat> getLutA();

  vector<Mat> lutX; // lutX and lutY must have same length
  vector<Mat> lutY;
  vector<Mat> lutA; // lutA must either have same length as lutX and lutY, or be empty

private:
  int lutCount;

};

} // namespace omnicamera

#endif /* OMNICAMERA_LOOKUPSTITCHER_H_ */
