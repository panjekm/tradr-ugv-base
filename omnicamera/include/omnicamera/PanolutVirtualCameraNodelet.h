
#ifndef OMNICAMERA_PANOLUTVIRTUALCAMERANODELET_H_
#define OMNICAMERA_PANOLUTVIRTUALCAMERANODELET_H_

#include <boost/make_shared.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <tf/transform_datatypes.h>

#include <omnicamera_msgs/GetVirtualCameraConfig.h>
#include <omnicamera_msgs/VirtualCameraConfig.h>

#include "omnicamera/LookupStitcher.h"


using omnicamera_msgs::VirtualCameraConfig;
using omnicamera_msgs::GetVirtualCameraConfig;

namespace omnicamera {

class PanolutVirtualCameraNodelet : public nodelet::Nodelet {
public:
  PanolutVirtualCameraNodelet();
  /**
   * Nodelet initialization callback.
   */
  void onInit();
  /**
   * Parses parameters using node handles, setup publishers and subscribers etc.
   */
  void setup(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  /**
   * Callback called by a timer to publish tf messages.
   */
  void tfPublisherCallback(const ros::TimerEvent& event);
  /**
   * Callback for updating virtual camera config.
   */
  void virtualCameraConfigCallback(const VirtualCameraConfig::ConstPtr& msg);
  /**
   * Service for getting current virtual camera config.
   */
  bool getVirtualCameraConfigService(GetVirtualCameraConfig::Request& request, GetVirtualCameraConfig::Response& response);
  /**
   * Callback for camera images, yields rendering and publishing new virtual camera view.
   */
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
private:
  static const int N_CAMS;
  /**
   * Input image width.
   */
  int inWidth;
  /**
   * Input image height.
   */
  int inHeight;
  /**
   * Panorama width.
   */
  int panoWidth;
  /**
   * Panorama height.
   */
  int panoHeight;
  /**
   * Flag indicating whether alpha masks are used.
   */
  bool alpha;
  /**
   * Sphere radius for panorama stitching.
   */
  int radius;
  /** Path for look-up table files. */
  std::string lutPath;
  /** Virtual camera frame ID. */
  std::string frameId;
  /** Parent frame id, tf messages are sent for parentFrameId -> frameId link. */
  std::string parentFrameId;
  /** Timer for publishing tf messages. */
  ros::Timer tfPublisherTimer;
  /** Transform to be published. Updated only when camera config changes. */
  tf::Transform transform;

  /** Indicates whether histogram equalization should be performed prior to publishing the image. */
  bool equalizeHist;

  /**
   * Panorama look-up table stitcher.
   */
  omnicamera::LookupStitcher pano;
  /**
   * Virtual camera look-up table stitcher.
   */
  omnicamera::LookupStitcher vcam;
  /**
   * Image transport, seems to be needed to avoid image transport subscribers/publishers going out of scope.
   */
  boost::shared_ptr<image_transport::ImageTransport> it;
  /**
   * Image subscriber.
   */
  image_transport::Subscriber imageSubcriber;
  /**
   * Virtual camera config subscriber.
   */
  ros::Subscriber virtualCameraConfigSubscriber;
  /**
   * Virtual camera config service server.
   */
  ros::ServiceServer getVirtualCameraConfigServer;
  /**
   * Virtual camera view publisher.
   */
  image_transport::Publisher outPub;
  /**
   * Camera info publisher.
   */
  ros::Publisher camPub;
  /**
   * Current virtual camera configuration, updated from a subscribed topic.
   */
  VirtualCameraConfig vcamConfig;
  /**
   * Camera info message template. Updated when camera configuration (field of view, width, height) changes, and sent for each image sent.
   */
  sensor_msgs::CameraInfo camMsg;
  /**
   * Flag indicating whether look-up tables are loaded.
   */
  bool lookupTablesLoaded;

  void updateTransform();
};

} // namespace omnicamera

#endif /* OMNICAMERA_PANOLUTVIRTUALCAMERANODELET_H_ */
