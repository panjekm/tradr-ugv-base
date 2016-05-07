
#include "omnicamera/PanolutVirtualCameraNodelet.h"

#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/image_encodings.h>

#include "omnicamera/panolut_virtual_camera_common.h"

//#define USE_MT_NODE_HANDLE

using ros::NodeHandle;
using omnicamera::rotationMatrix;
using omnicamera::panTiltToCart;
using omnicamera::cartToPanTilt;
using omnicamera::cameraInverse;
using omnicamera::updateVcamStitcher;

namespace omnicamera {

const int PanolutVirtualCameraNodelet::N_CAMS = 6;

PanolutVirtualCameraNodelet::PanolutVirtualCameraNodelet() {
//  NODELET_INFO("PanolutVirtualCameraNodelet::PanolutVirtualCameraNodelet");
}

void PanolutVirtualCameraNodelet::setup(NodeHandle &nh, NodeHandle & pnh) {
//  NODELET_INFO("PanolutVirtualCameraNodelet::setup");

  pnh.param("in_width", inWidth, 1616);
  pnh.param("in_height", inHeight, 7392);
  NODELET_INFO("Input image size: %dx%d.", inWidth, inHeight);

  pnh.param("pano_width", panoWidth, 960);
  pnh.param("pano_height", panoHeight, 480);
  NODELET_INFO("Panorama size: %dx%d.", panoWidth, panoHeight);
  pnh.param("frame_id", frameId, string("ptz"));
  pnh.param("parent_frame_id", parentFrameId, string("omnicam"));

  pnh.param("alpha", alpha, true);
  NODELET_INFO("Alpha blending: %s.", alpha ? "enabled" : "disabled");

  pnh.param("equalize_hist", equalizeHist, false);
  NODELET_INFO("Equalize histogram: %s.", equalizeHist ? "enabled" : "disabled");

  pnh.param("radius", radius, 20);
  NODELET_INFO("Sphere radius: %d.", radius);

  std::string omnicameraPath = ros::package::getPath("omnicamera");
  pnh.param("lut_format", lutPath, string("/res/lut/LUT_img_%dx%d_pano_%dx%dr%d_%s%d.png"));
  lutPath = omnicameraPath + lutPath;
  NODELET_INFO("LUT path template: %s.", lutPath.data());

  pano.clearLookupTables();
  vcam.clearLookupTables();
  for (int iCam = 1; iCam <= N_CAMS; iCam++) {
    int len = 1024;
    char xPath[len], yPath[len], aPath[len];
    snprintf(xPath, len, lutPath.data(), inWidth, inHeight, panoWidth, panoHeight, radius, "x", iCam);
    snprintf(yPath, len, lutPath.data(), inWidth, inHeight, panoWidth, panoHeight, radius, "y", iCam);
    snprintf(aPath, len, lutPath.data(), inWidth, inHeight, panoWidth, panoHeight, radius, "a", iCam);
    NODELET_INFO("Camera %d look-up tables: %s, %s, %s.", iCam, xPath, yPath, aPath);
    pano.addLookupTables(string(xPath), string(yPath), string(aPath));
    vcam.addLookupTables(Mat(0, 0, CV_16UC1), Mat(0, 0, CV_16UC1), Mat(0, 0, CV_8UC1));
  }
  pano.fixBounds(inWidth, inHeight);

  pnh.param("viewport_width",   vcamConfig.viewportWidth,   512);
  pnh.param("viewport_height",  vcamConfig.viewportHeight,  512);
  pnh.param("pan",              vcamConfig.pan,             0.);
  pnh.param("tilt",             vcamConfig.tilt,            0.);
  pnh.param("horizontal_fov",   vcamConfig.horizontalFov,   90.);
  pnh.param("vertical_fov",     vcamConfig.verticalFov,     90.);
  NODELET_INFO("Virtual camera settings: Viewport width: %i, height: %i, pan: %f, tilt: %f, horizontal fov: %f, vertical fov: %f.",
      vcamConfig.viewportWidth, vcamConfig.viewportHeight, vcamConfig.pan, vcamConfig.tilt, vcamConfig.horizontalFov, vcamConfig.verticalFov);

  updateTransform();

  updateVcamStitcher(pano, panoWidth, panoHeight, vcamConfig, vcam);
  vcam.fixBounds(inWidth, inHeight);
  // Update camera info "template".
  camMsg.width = vcamConfig.viewportWidth;
  camMsg.height = vcamConfig.viewportHeight;
  Mat K;
  cameraMatrix(vcamConfig.horizontalFov / 180 * M_PI, vcamConfig.verticalFov / 180 * M_PI, vcamConfig.viewportWidth, vcamConfig.viewportHeight, K);
  for (int r = 0; r < K.rows; r++) {
    for (int c = 0; c < K.cols; c++) {
      camMsg.K[r * 3 + c] = K.at<double>(r, c);
    }
  }
  for (int r = 0; r < K.rows; r++) {
    for (int c = 0; c < K.cols; c++) {
      camMsg.P[r * 4 + c] = K.at<double>(r, c);
    }
    camMsg.P[r * 4 + 3] = 0; // No translation.
  }
  for (int i = 0; i < 9; i++) {
    camMsg.R[i] = 0;
  }
  camMsg.R[0] = 1;
  camMsg.R[4] = 1;
  camMsg.R[8] = 1;

//  image_transport::ImageTransport it(nh);
  it.reset(new image_transport::ImageTransport(nh));
  imageSubcriber = it->subscribe("in", 1, &PanolutVirtualCameraNodelet::imageCallback, this);
  outPub = it->advertise("out", 1);
  camPub = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1);

  tfPublisherTimer = nh.createTimer(ros::Duration(0.01), &PanolutVirtualCameraNodelet::tfPublisherCallback, this);
  virtualCameraConfigSubscriber = nh.subscribe("config", 1, &PanolutVirtualCameraNodelet::virtualCameraConfigCallback, this);
  getVirtualCameraConfigServer = nh.advertiseService("get_config", &PanolutVirtualCameraNodelet::getVirtualCameraConfigService, this);
}

void PanolutVirtualCameraNodelet::onInit() {
  NODELET_INFO("Initializing PanolutVirtualCameraNodelet...");

#ifdef USE_MT_NODE_HANDLE
    ros::NodeHandle &nh = getMTNodeHandle();
    ros::NodeHandle &pnh = getMTPrivateNodeHandle();
#else
  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &pnh = getPrivateNodeHandle();
#endif

  setup(nh, pnh);
}

void PanolutVirtualCameraNodelet::tfPublisherCallback(const ros::TimerEvent& event) {
  static tf::TransformBroadcaster br;
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parentFrameId, frameId));
}

void PanolutVirtualCameraNodelet::updateTransform() {
  transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf::Quaternion parent2cam = tf::createQuaternionFromRPY(0.0, -vcamConfig.tilt / 180 * M_PI, vcamConfig.pan / 180 * M_PI);
  tf::Quaternion cam2image(-0.5, 0.5, -0.5, 0.5);
  transform.setRotation(parent2cam * cam2image);
}

void PanolutVirtualCameraNodelet::virtualCameraConfigCallback(const VirtualCameraConfig::ConstPtr& msg) {
  NODELET_DEBUG("PanolutVirtualCameraNodelet::virtualCameraConfigCallback: Entering...");

  vcamConfig = *msg;
  // Fix vcam config.
  if (vcamConfig.horizontalFov < 1) vcamConfig.horizontalFov = 1;
  if (vcamConfig.horizontalFov > 179) vcamConfig.horizontalFov = 179;
  if (vcamConfig.verticalFov < 1) vcamConfig.verticalFov = 1;
  if (vcamConfig.verticalFov > 179) vcamConfig.verticalFov = 179;
  if (vcamConfig.viewportWidth < 1) vcamConfig.viewportWidth = 1;
  if (vcamConfig.viewportHeight < 1) vcamConfig.viewportHeight = 1;
  if (vcamConfig.imageUpdateInterval < 0) vcamConfig.viewportHeight = 0;

  updateTransform();

  updateVcamStitcher(pano, panoWidth, panoHeight, vcamConfig, vcam);
  vcam.fixBounds(inWidth, inHeight);

  camMsg.width = vcamConfig.viewportWidth;
  camMsg.height = vcamConfig.viewportHeight;
  Mat K;
  cameraMatrix(vcamConfig.horizontalFov / 180 * M_PI, vcamConfig.verticalFov / 180 * M_PI, vcamConfig.viewportWidth, vcamConfig.viewportHeight, K);
  for (int r = 0; r < K.rows; r++) {
    for (int c = 0; c < K.cols; c++) {
      camMsg.K[r * 3 + c] = K.at<double>(r, c);
    }
  }
  for (int r = 0; r < K.rows; r++) {
    for (int c = 0; c < K.cols; c++) {
      camMsg.P[r * 4 + c] = K.at<double>(r, c);
    }
    camMsg.P[r * 4 + 3] = 0; // No translation.
  }

  NODELET_DEBUG("PanolutVirtualCameraNodelet::virtualCameraConfigCallback: Leaving...");
}

bool PanolutVirtualCameraNodelet::getVirtualCameraConfigService(GetVirtualCameraConfig::Request& request, GetVirtualCameraConfig::Response& response) {
  response.config = vcamConfig;
  return true;
}

void PanolutVirtualCameraNodelet::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
//  NODELET_INFO("imageCallback: Entering...");

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
  vcam.stitchImage(cvPtr->image, image);

  cv_bridge::CvImage out;
  out.encoding = sensor_msgs::image_encodings::BGR8;
  out.header = msg->header;
  out.header.frame_id = frameId;
  if (equalizeHist) {
    // Equalize histogram before publishing.
    ros::Time t1 = ros::Time::now();
    Mat hsv;
    cv::cvtColor(image, hsv, CV_BGR2HSV);
    std::vector<Mat> hsvChannels;
    cv::split(hsv, hsvChannels);
    Mat vOut;
    cv::equalizeHist(hsvChannels[2], vOut);
    hsvChannels[2] = vOut;
    cv::merge(hsvChannels, hsv);
    cv::cvtColor(hsv, image, CV_HSV2BGR);
    NODELET_DEBUG("Histogram equalization: %.3f s.", (ros::Time::now() - t1).toSec());
  }
  out.image = image;
  outPub.publish(out.toImageMsg());

  camMsg.header = out.header;
  camMsg.height = out.image.rows;
  camMsg.width = out.image.cols;
  camPub.publish(camMsg);

//  NODELET_INFO("imageCallback: Returning...");
}

} // namespace omnicamera

PLUGINLIB_DECLARE_CLASS(omnicamera, panolut_virtual_camera, omnicamera::PanolutVirtualCameraNodelet, nodelet::Nodelet)
