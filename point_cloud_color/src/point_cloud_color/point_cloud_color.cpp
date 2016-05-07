/**
 * Point cloud coloring from calibrated cameras.
 *
 * TODO: Use OpenCV project points, along with a 'undistort' param.
 * TODO: Mask out robot model.
 */

//#define USE_MT_NODE_HANDLE

#include <limits>
#include <boost/format.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <nifti_pcl_common/point_types.h>
#include <nodelet/nodelet.h>
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_listener.h>

typedef nifti_pcl_common::LaserPoint PointIn;
typedef nifti_pcl_common::AssemblerPoint PointOut;

using namespace sensor_msgs;
using namespace std;

namespace point_cloud_color {

class PointCloudColor: public nodelet::Nodelet {
public:
  PointCloudColor();
  virtual ~PointCloudColor();
  void onInit();
  void cameraCallback(const sensor_msgs::ImageConstPtr &imageMsg, const sensor_msgs::CameraInfoConstPtr &cameraInfoMsg);
  void pointCloudCallback(const pcl::PointCloud<PointIn>::ConstPtr &pointCloudMsg);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  tf::TransformListener transformListener;
  vector<image_transport::CameraSubscriber> cameraSubscribers;
  ros::Subscriber pointCloudSub;
  ros::Publisher pointCloudPub;
  ros::Subscriber pointCloud2Sub;
  ros::Publisher pointCloud2Pub;
  map<string, cv_bridge::CvImageConstPtr> images;
  map<string, sensor_msgs::CameraInfoConstPtr> cameraInfos;
  string targetFrame;
  string fixedFrame;
  int numCameras;
  double maxImageAge;
  bool useFirstValid;
  bool unrectify;
  int imageQueueSize;
  int pointCloudQueueSize;
  double waitForTransform;
};

PointCloudColor::PointCloudColor() :
    transformListener(ros::Duration(15.0)),
    targetFrame("/laser"),
    fixedFrame("/odom"),
    numCameras(1),
    maxImageAge(5.0),
    useFirstValid(true),
    unrectify(false),
    imageQueueSize(1),
    pointCloudQueueSize(1),
    waitForTransform(1.0) {
}

PointCloudColor::~PointCloudColor() {
}

void PointCloudColor::onInit() {
  NODELET_INFO("PointCloudColor::onInit: Initializing...");

#ifdef USE_MT_NODE_HANDLE
  ros::NodeHandle &nh = getMTNodeHandle();
  ros::NodeHandle &pnh = getMTPrivateNodeHandle();
#else
  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &pnh = getPrivateNodeHandle();
#endif

  // Get and process parameters.
  pnh.param("target_frame", targetFrame, targetFrame);
  NODELET_INFO("PointCloudColor::onInit: Target frame: %s.", targetFrame.c_str());
  pnh.param("fixed_frame", fixedFrame, fixedFrame);
  NODELET_INFO("PointCloudColor::onInit: Fixed frame: %s.", fixedFrame.c_str());
  pnh.param("num_cameras", numCameras, numCameras);
  numCameras = numCameras >= 0 ? numCameras : 0;
  NODELET_INFO("PointCloudColor::onInit: Number of cameras: %i.", numCameras);
  pnh.param("max_image_age", maxImageAge, maxImageAge);
  NODELET_INFO("PointCloudColor::onInit: Maximum image age: %.1f s.", maxImageAge);
  pnh.param("use_first_valid", useFirstValid, useFirstValid);
  NODELET_INFO("PointCloudColor::onInit: Use first valid projection: %s.", useFirstValid ? "yes" : "no");
  pnh.param("unrectify", unrectify, unrectify);
  NODELET_INFO("PointCloudColor::onInit: Unrectify points: %s.", unrectify ? "yes" : "no");
  pnh.param("image_queue_size", imageQueueSize, imageQueueSize);
  imageQueueSize = imageQueueSize >= 1 ? imageQueueSize : 1;
  NODELET_INFO("PointCloudColor::onInit: Image queue size: %i.", imageQueueSize);
  pnh.param("point_cloud_queue_size", pointCloudQueueSize, pointCloudQueueSize);
  pointCloudQueueSize = pointCloudQueueSize >= 1 ? pointCloudQueueSize : 1;
  NODELET_INFO("PointCloudColor::onInit: Point cloud queue size: %i.", pointCloudQueueSize);
  pnh.param("wait_for_transform", waitForTransform, waitForTransform);
  waitForTransform = waitForTransform >= 0.0 ? waitForTransform : 0.0;
  NODELET_INFO("PointCloudColor::onInit: Wait for transform timeout: %.2f s.", waitForTransform);

  // Subscribe list of camera topics.
  image_transport::ImageTransport it(nh);
  cameraSubscribers.resize(numCameras);
  for (int iCam = 0; iCam < numCameras; iCam++) {
    string cameraTopic = nh.resolveName((boost::format("camera_%i/image") % iCam).str(), true);
    NODELET_INFO("PointCloudColor::onInit: Subscribing camera %s.", cameraTopic.c_str());
    cameraSubscribers[iCam] = it.subscribeCamera(cameraTopic, imageQueueSize, &PointCloudColor::cameraCallback, this);
  }

  // Subscribe point cloud topic.
  string pclInTopic = nh.resolveName("pcl_in", true);
  NODELET_INFO("PointCloudColor::onInit: Subscribing point cloud %s.", pclInTopic.c_str());
  pointCloudSub = nh.subscribe<pcl::PointCloud<PointIn> >(pclInTopic, pointCloudQueueSize, &PointCloudColor::pointCloudCallback, this);

  // Advertise colored point cloud topic.
  string pclOutTopic = nh.resolveName("pcl_out", true);
  NODELET_INFO("PointCloudColor::onInit: Advertising colored point cloud %s.", pclOutTopic.c_str());
  pointCloudPub = nh.advertise<pcl::PointCloud<PointOut> >(pclOutTopic, pointCloudQueueSize, false);
}

void PointCloudColor::cameraCallback(const sensor_msgs::ImageConstPtr &imageMsg, const sensor_msgs::CameraInfoConstPtr &cameraInfoMsg) {
  NODELET_DEBUG("PointCloudColor::cameraCallback: Image frame %s, camera info frame %s.", imageMsg->header.frame_id.c_str(), cameraInfoMsg->header.frame_id.c_str());
  // Use frame id from image message for both image and camera info to ensure consistency.
  images[imageMsg->header.frame_id] = cv_bridge::toCvShare(imageMsg, sensor_msgs::image_encodings::BGR8);
  cameraInfos[imageMsg->header.frame_id] = cameraInfoMsg;
}

void PointCloudColor::pointCloudCallback(const pcl::PointCloud<PointIn>::ConstPtr &pclPtr) {
  ros::Time t1 = ros::Time::now();
  size_t numPoints = pclPtr->points.size();
  NODELET_DEBUG("PointCloudColor::pointCloudCallback: Point cloud received (%lu points).", numPoints);

  // Prepare output point cloud.
  pcl::PointCloud<PointOut> pclOut;
  pcl::copyPointCloud<PointIn, PointOut>(*pclPtr, pclOut);

  // For point projections get rid off the fields we don't need.
  pcl::PointCloud<pcl::PointXYZ> pclXyz;
  pcl::copyPointCloud<PointIn, pcl::PointXYZ>(*pclPtr, pclXyz);

  // Initialize vector with projection distances from image center, used as a quality measure.
  vector<double> dist(numPoints, DBL_MAX);
  map<string, cv_bridge::CvImageConstPtr>::iterator itImages;
  for (itImages = images.begin(); itImages != images.end(); itImages++) {
    string cameraFrame = (*itImages).first;
    cv_bridge::CvImageConstPtr imagePtr  = images[cameraFrame];
    sensor_msgs::CameraInfoConstPtr cameraInfoPtr = cameraInfos[cameraFrame];
    NODELET_DEBUG("Map key %s, image frame %s, camera info frame %s.", cameraFrame.c_str(), imagePtr->header.frame_id.c_str(), cameraInfoPtr->header.frame_id.c_str());

    image_geometry::PinholeCameraModel cameraModel;
    if (!cameraModel.fromCameraInfo(cameraInfoPtr)) {
      NODELET_WARN("Camera model could not be constructed from camera info. Skipping the image...");
      continue;
    }

    // Check relative age of the point cloud and the image. Skip the image if the time span is to large.
    double imageAge = (pclPtr->header.stamp - imagePtr->header.stamp).toSec();
    if (imageAge > maxImageAge) {
      NODELET_WARN("Image %.1f s > %.1f s older than point cloud. Skipping the image...", imageAge, maxImageAge);
      continue;
    }
    NODELET_DEBUG("PointCloudColor::pointCloudCallback: Using camera %s, image age %.1f s.", cameraFrame.c_str(), imageAge);

    // Transform point cloud into camera frame, skip the image if the transform is not available.
    pcl::PointCloud<pcl::PointXYZ> pclCam;
    if (!transformListener.waitForTransform(cameraFrame, cameraInfoPtr->header.stamp, // target frame and time
                           pclPtr->header.frame_id, pclPtr->header.stamp, // source frame and time
                           fixedFrame, ros::Duration(waitForTransform))
        || !pcl_ros::transformPointCloud(cameraFrame, imagePtr->header.stamp, pclXyz, fixedFrame, pclCam, transformListener)) {
      NODELET_WARN("PointCloudColor::pointCloudCallback: Cannot transform point cloud to camera frame %s. Skipping the image...", cameraFrame.c_str());
      continue;
    }

    for (size_t iPoint = 0; iPoint < numPoints; iPoint++) {
      // Skip points behind the camera.
      if (pclCam.points[iPoint].z < 0 || (useFirstValid && dist[iPoint] < DBL_MAX)) {
        continue;
      }
      cv::Point3d p3 = cv::Point3d(pclCam.points[iPoint].x, pclCam.points[iPoint].y, pclCam.points[iPoint].z);
      // Get rectified image coordinates.
      cv::Point2d p2 = cameraModel.project3dToPixel(p3);
      if (unrectify) {
        p2 = cameraModel.unrectifyPoint(p2);
      }
      double d = hypot(cameraInfoPtr->width / 2.0 - p2.x, cameraInfoPtr->height / 2.0 - p2.y);
      if (d >= dist[iPoint]) {
        // Keep color from the previous projection which we assume is better.
        continue;
      }
      // TODO: Linear interpolation.
      int x = round(p2.x);
      int y = round(p2.y);
      // Skip points outside the image.
      if (x < 0 || y < 0 || x >= cameraInfoPtr->width || y >= cameraInfoPtr->height) {
        continue;
      }
      dist[iPoint] = d;
      cv::Vec3b px = imagePtr->image.at<cv::Vec3b>(y, x);
      // Assuming RGB.
      pclOut.points[iPoint].r = px.val[0];
      pclOut.points[iPoint].g = px.val[1];
      pclOut.points[iPoint].b = px.val[2];
    }
  }
  NODELET_DEBUG("PointCloudColor::pointCloudCallback: Coloring point cloud: %.3f s.", (ros::Time::now() - t1).toSec());
  pointCloudPub.publish(pclOut);
}

} /* namespace point_cloud_color */

PLUGINLIB_DECLARE_CLASS(point_cloud_color, point_cloud_color, point_cloud_color::PointCloudColor, nodelet::Nodelet)
