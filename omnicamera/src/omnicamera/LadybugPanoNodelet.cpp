
#include <boost/make_shared.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "omnicamera/LookupStitcher.h"

//#define USE_MT_NODE_HANDLE

namespace omnicamera {

class LadybugPanoNodelet : public nodelet::Nodelet {
public:
  LadybugPanoNodelet();
  void onInit();
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
private:
  int inWidth;
  int inHeight;
  int outWidth;
  int outHeight;
  double pan;
  string omnicamFrameId;
  string frameId;
  bool alpha;
  int radius;
  string lutPath;
  omnicamera::LookupStitcher stitcher;
  /**
   * Image transport, seems to be needed to avoid image transport subscribers/publishers going out of scope.
   */
  boost::shared_ptr<image_transport::ImageTransport> it;
  image_transport::Subscriber imageSub;
  image_transport::Publisher outPub;
  ros::Publisher camPub;
  bool lookupTablesLoaded;
  tf::TransformBroadcaster tfBroadcaster;
};

LadybugPanoNodelet::LadybugPanoNodelet() {
}

void LadybugPanoNodelet::onInit() {
  NODELET_INFO("Initializing LadybugPanoNodelet...");
#ifdef USE_MT_NODE_HANDLE
  ros::NodeHandle &nh = getMTNodeHandle();
  ros::NodeHandle &pnh = getMTPrivateNodeHandle();
#else
  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &pnh = getPrivateNodeHandle();
#endif

  pnh.param("in_width", inWidth, 1616);
  pnh.param("in_height", inHeight, 1232);
  NODELET_INFO("Input image size: %dx%d.", inWidth, inHeight);

  pnh.param("out_width", outWidth, 960);
  pnh.param("out_height", outHeight, 480);
  pnh.param("pan", pan, 0.0);
  pnh.param("frame_id", frameId, string("pano"));
  pnh.param("omnicam_frame_id", omnicamFrameId, string("omnicam"));
  NODELET_INFO("Output image size: %dx%d, frame id: %s, omnicam frame id: %s.", outWidth, outHeight, frameId.data(), omnicamFrameId.data());

  pnh.param("alpha", alpha, true);
  NODELET_INFO("Alpha blending: %s.", alpha ? "enabled" : "disabled");

  pnh.param("radius", radius, 20);
  NODELET_INFO("Sphere radius: %d.", radius);

  std::string omnicameraPath = ros::package::getPath("omnicamera");
  pnh.param("lut_format", lutPath, string("/res/lut/LUT_img_%dx%d_pano_%dx%dr%d_%s%d.png"));
  lutPath = omnicameraPath + lutPath;
  NODELET_INFO("LUT path format: %s.", lutPath.data());
  // Loading look-up tables postponed to first callback to return fast.
  lookupTablesLoaded = false;

//  image_transport::ImageTransport it(nh);
  it.reset(new image_transport::ImageTransport(nh));
  imageSub = it->subscribe("in", 1, &LadybugPanoNodelet::imageCallback, this);
  outPub = it->advertise("out", 1);
  camPub = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
}

void LadybugPanoNodelet::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
//   NODELET_INFO("LadybugPanoNodelet::imageCallback: entering.");

  if (!lookupTablesLoaded) {
    NODELET_INFO("Loading look-up tables...");
    stitcher.clearLookupTables();
    for (int iCam = 1; iCam <= 6; iCam++) {
      int len = 1024;
      char xPath[len], yPath[len], aPath[len];
      snprintf(xPath, len, lutPath.data(), inWidth, inHeight, outWidth, outHeight, radius, "x", iCam);
      snprintf(yPath, len, lutPath.data(), inWidth, inHeight, outWidth, outHeight, radius, "y", iCam);
      snprintf(aPath, len, lutPath.data(), inWidth, inHeight, outWidth, outHeight, radius, "a", iCam);
      NODELET_INFO("Camera %d look-up tables: %s, %s, %s.", iCam, xPath, yPath, aPath);
      stitcher.addLookupTables(string(xPath), string(yPath), string(aPath));
    }
    stitcher.fixBounds(inWidth, inHeight);

    // Apply pan if needed by shifting look-up tables.
    int shiftRight = round(outWidth * pan / 360.0);
    if (shiftRight != 0) {
      NODELET_INFO("Using pan %f. Shifting look-up tables right by %d pixels.", pan, shiftRight);
      cv::Mat tempLut, tempColRange;
      int cols = stitcher.getLutCount() > 0 ? stitcher.lutX[0].cols : 0;
      for (int i = 0; shiftRight != 0 && i < stitcher.getLutCount(); i++) {
        if (shiftRight > 0) {
          // Shift look-up tables right.
          // Ensure appropriate dimensions and type.
          stitcher.lutA[i].copyTo(tempLut);
          // First fill temporary matrix.
          NODELET_DEBUG("Valid range <%d, %d>.", 0, cols - 1);
          NODELET_DEBUG("Copy <%d, %d> to <%d, %d>", 0, cols - shiftRight - 1, shiftRight, cols - 1);
          NODELET_DEBUG("Copy <%d, %d> to <%d, %d>", cols - shiftRight, cols - 1, 0, shiftRight - 1);
          tempColRange = tempLut.colRange(shiftRight, cols - 1);
          stitcher.lutA[i].colRange(0, cols - shiftRight - 1).copyTo(tempColRange);
          tempColRange = tempLut.colRange(0, shiftRight - 1);
          stitcher.lutA[i].colRange(cols - shiftRight, cols - 1).copyTo(tempColRange);
          // Then copy back to the working look-up table.
          tempLut.copyTo(stitcher.lutA[i]);

          stitcher.lutX[i].copyTo(tempLut);
          tempColRange = tempLut.colRange(shiftRight, cols - 1);
          stitcher.lutX[i].colRange(0, cols - shiftRight - 1).copyTo(tempColRange);
          tempColRange = tempLut.colRange(0, shiftRight - 1);
          stitcher.lutX[i].colRange(cols - shiftRight, cols - 1).copyTo(tempColRange);
          tempLut.copyTo(stitcher.lutX[i]);

          stitcher.lutY[i].copyTo(tempLut);
          tempColRange = tempLut.colRange(shiftRight, cols - 1);
          stitcher.lutY[i].colRange(0, cols - shiftRight - 1).copyTo(tempColRange);
          tempColRange = tempLut.colRange(0, shiftRight - 1);
          stitcher.lutY[i].colRange(cols - shiftRight, cols - 1).copyTo(tempColRange);
          tempLut.copyTo(stitcher.lutY[i]);

        } else {
          // Shift look-up tables left.
          int shiftLeft = -shiftRight;
          stitcher.lutA[i].copyTo(tempLut);
          NODELET_DEBUG("Valid range <%d, %d>.", 0, cols - 1);
          NODELET_DEBUG("Copy <%d, %d> to <%d, %d>", shiftLeft, cols - 1, 0, cols - shiftLeft - 1);
          NODELET_DEBUG("Copy <%d, %d> to <%d, %d>", 0, shiftLeft - 1, cols - shiftLeft, cols - 1);
          tempColRange = tempLut.colRange(0, cols - shiftLeft - 1);
          stitcher.lutA[i].colRange(shiftLeft, cols - 1).copyTo(tempColRange);
          tempColRange = tempLut.colRange(cols - shiftLeft, cols - 1);
          stitcher.lutA[i].colRange(0, shiftLeft - 1).copyTo(tempColRange);
          tempLut.copyTo(stitcher.lutA[i]);

          stitcher.lutX[i].copyTo(tempLut);
          tempColRange = tempLut.colRange(0, cols - shiftLeft - 1);
          stitcher.lutX[i].colRange(shiftLeft, cols - 1).copyTo(tempColRange);
          tempColRange = tempLut.colRange(cols - shiftLeft, cols - 1);
          stitcher.lutX[i].colRange(0, shiftLeft - 1).copyTo(tempColRange);
          tempLut.copyTo(stitcher.lutX[i]);

          stitcher.lutY[i].copyTo(tempLut);
          tempColRange = tempLut.colRange(0, cols - shiftLeft - 1);
          stitcher.lutY[i].colRange(shiftLeft, cols - 1).copyTo(tempColRange);
          tempColRange = tempLut.colRange(cols - shiftLeft, cols - 1);
          stitcher.lutY[i].colRange(0, shiftLeft - 1).copyTo(tempColRange);
          tempLut.copyTo(stitcher.lutY[i]);
        }
      }
    }

    lookupTablesLoaded = true;
  }

  if (outPub.getNumSubscribers() == 0 && camPub.getNumSubscribers() == 0) {
    return;
  }

  cv_bridge::CvImageConstPtr cvPtr;
  try {
    cvPtr = cv_bridge::toCvShare(msg);
    if (cvPtr->encoding.compare(sensor_msgs::image_encodings::BGR8)) {
      NODELET_ERROR("Encoding not supported: %s.", cvPtr->encoding.data());
      return;
    }
  } catch (cv_bridge::Exception& e) {
    NODELET_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (cvPtr->image.rows != inHeight || cvPtr->image.cols != inWidth) {
    NODELET_ERROR("Unexpected image dimensions: %dx%d.", cvPtr->image.cols, cvPtr->image.rows);
  }

  Mat image;
  NODELET_DEBUG("Stitching started.");
  stitcher.stitchImage(cvPtr->image, image);
  NODELET_DEBUG("Stitching finished.");

  tf::Transform transform;
  double realPan = 2 * M_PI * round(outWidth * pan / 360.0) / outWidth;
  transform.setRotation(tf::createQuaternionFromRPY(0, 0, realPan));
  tfBroadcaster.sendTransform(tf::StampedTransform(transform, msg->header.stamp, omnicamFrameId, frameId));

  cv_bridge::CvImage out;
  out.encoding = sensor_msgs::image_encodings::BGR8;
  out.header = msg->header;
  out.header.frame_id = frameId;
  out.image = image;
  outPub.publish(out.toImageMsg());

  sensor_msgs::CameraInfo camMsg;
  camMsg.header = out.header;
  camMsg.height = out.image.rows;
  camMsg.width = out.image.cols;
  camPub.publish(camMsg);
}

} // namespace omnicamera

PLUGINLIB_DECLARE_CLASS(omnicamera, ladybug_pano, omnicamera::LadybugPanoNodelet, nodelet::Nodelet)
