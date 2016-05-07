
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>

#include "omnicamera/LookupStitcher.h"
#include "omnicamera/LadySaw.h"

using omnicamera::LookupStitcher;
using omnicamera::LadySaw;

namespace omnicamera {

/**
 * Nodelet for panorama stitching and image splitting etc.
 *
 * Separate nodelets (img2pano_nodelet, ladysaw_nodelet) subscribing
 * same topic did not work therefore this single nodelet.
 */
class LadybugProcNodelet : public nodelet::Nodelet {
public:
  LadybugProcNodelet();
  void onInit();
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

private:
//  boost::shared_ptr<image_transport::ImageTransport> it;
  // Panorama stitching
  int inWidth;
  int inHeight;
  int outWidth;
  int outHeight;
  bool alpha;
  int radius;
  string lutPath;
  omnicamera::LookupStitcher stitcher;
  image_transport::Publisher outPub;
  ros::Publisher camPub;
  bool lookupTablesLoaded;
  Mat pano;
  // Image splitting
  LadySaw ladySaw;
  vector<Mat> cams;
  image_transport::Subscriber imageSub;
  image_transport::Publisher outPubs[6];
  ros::Publisher camPubs[6];
};

LadybugProcNodelet::LadybugProcNodelet() {
}

} // namespace omnicamera
