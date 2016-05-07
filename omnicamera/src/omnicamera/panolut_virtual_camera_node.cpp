
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>

#include "omnicamera_msgs/GetVirtualCameraConfig.h"
#include "omnicamera/panolut_virtual_camera_common.h"

//#define USE_MT_NODE_HANDLE

using cv::Mat_;
using omnicamera_msgs::GetVirtualCameraConfig;

using omnicamera::rotationMatrix;
using omnicamera::panTiltToCart;
using omnicamera::cartToPanTilt;
using omnicamera::cameraInverse;
using omnicamera::updateVcamStitcher;

namespace {
const int N_CAMS = 6;
int inWidth, inHeight;
int panoWidth, panoHeight;
bool alpha;
int radius;
string lutPath;

omnicamera::LookupStitcher pano;
omnicamera::LookupStitcher vcam;
image_transport::Publisher outPub;
ros::Publisher camPub;

/**
 * Current virtual camera configuration, updated from a subscribed topic.
 */
VirtualCameraConfig vcamConfig;

bool getVirtualCameraConfigService(GetVirtualCameraConfig::Request& request, GetVirtualCameraConfig::Response& response) {
  response.config = vcamConfig;
  return true;
}

void virtualCameraConfigCallback(const VirtualCameraConfig::ConstPtr& msg) {
  updateVcamStitcher(pano, panoWidth, panoHeight, *msg, vcam);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  ROS_INFO("imageCallback: Entering...");

  cv_bridge::CvImageConstPtr cvPtr;
  try {
    cvPtr = cv_bridge::toCvShare(msg);
    if (cvPtr->encoding.compare(sensor_msgs::image_encodings::BGR8)) {
      ROS_ERROR("Encoding not supported: %s.", cvPtr->encoding.data());
      return;
    }
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (cvPtr->image.rows != inHeight || cvPtr->image.cols != inWidth) {
    ROS_ERROR("Unexpected image dimensions: %dx%d.", cvPtr->image.cols, cvPtr->image.rows);
  }

  Mat image;
  vcam.stitchImage(cvPtr->image, image);

  cv_bridge::CvImage out;
  out.encoding = sensor_msgs::image_encodings::BGR8;
  out.header = msg->header;
  out.image = image;
  outPub.publish(out.toImageMsg());

  sensor_msgs::CameraInfo camMsg;
  camMsg.header = out.header;
  camMsg.height = out.image.rows;
  camMsg.width = out.image.cols;
  camPub.publish(camMsg);

  ROS_INFO("imageCallback: Returning...");
}

} // namespace

int main(int argc, char ** argv) {

  ros::init(argc, argv, "img2pano");
  ros::NodeHandle nh;

  // Construct a node handle with private namespace to get private parameters...
  ros::NodeHandle pnh("~");
  pnh.param("in_width", inWidth, 1616);
  pnh.param("in_height", inHeight, 1232);
  ROS_INFO("Input image size: %dx%d.", inWidth, inHeight);

  pnh.param("pano_width", panoWidth, 960);
  pnh.param("pano_height", panoHeight, 480);
  ROS_INFO("Panorama size: %dx%d.", panoWidth, panoHeight);

  pnh.param("alpha", alpha, true);
  ROS_INFO("Alpha blending: %s.", alpha ? "enabled" : "disabled");

  pnh.param("radius", radius, 20);
  ROS_INFO("Sphere radius: %d.", radius);

  pnh.param("lut_path", lutPath, string("LUT_img_%dx%d_pano_%dx%dr%d_%s%d.png"));
  ROS_INFO("LUT path template: %s.", lutPath.data());

  pano.clearLookupTables();
  vcam.clearLookupTables();
  for (int iCam = 1; iCam <= N_CAMS; iCam++) {
    int len = 1024;
    char xPath[len], yPath[len], aPath[len];
    snprintf(xPath, len, lutPath.data(), inWidth, inHeight, panoWidth, panoHeight, radius, "x", iCam);
    snprintf(yPath, len, lutPath.data(), inWidth, inHeight, panoWidth, panoHeight, radius, "y", iCam);
    snprintf(aPath, len, lutPath.data(), inWidth, inHeight, panoWidth, panoHeight, radius, "a", iCam);
    ROS_INFO("Camera %d look-up tables: %s, %s, %s.", iCam, xPath, yPath, aPath);
    pano.addLookupTables(string(xPath), string(yPath), string(aPath));
    vcam.addLookupTables(Mat(0, 0, CV_16UC1), Mat(0, 0, CV_16UC1), Mat(0, 0, CV_8UC1));
  }

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("in", 1, imageCallback);
  outPub = it.advertise("out", 1);
  camPub = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1);

  ros::Subscriber virtualCameraConfigSubscriber = nh.subscribe("config", 1, virtualCameraConfigCallback);
  ros::ServiceServer getVirtualCameraConfigServer = nh.advertiseService("get_config", getVirtualCameraConfigService);

  vcamConfig.viewportWidth = 640;
  vcamConfig.viewportHeight = 480;
  vcamConfig.pan = 0.0;
  vcamConfig.tilt = 0.0;
  vcamConfig.horizontalFov = 90.0;
  vcamConfig.verticalFov = 90.0 * 3 / 4;
//  setVirtualCameraConfig(vcamConfig);
  updateVcamStitcher(pano, panoWidth, panoHeight, vcamConfig, vcam);

  ros::spin();

  return 0;
}
