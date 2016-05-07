
//#define USE_MT_NODE_HANDLE

#include <laser_geometry/laser_geometry.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/LaserScan.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace point_cloud_color {

/**
 * @brief A nodelet converting laser scans to point clouds.
 *
 * This nodelet converts scans (i.e., sensor_msgs::LaserScan messages) to point clouds (i.e., to
 * sensor_msgs::PointCloud2 messages).
 *
 */
class ScanToPointCloud: public nodelet::Nodelet {
public:
  ScanToPointCloud();
  virtual ~ScanToPointCloud();
  void onInit();
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scanMsg);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  std::string targetFrame;
  double waitForTransform;
  /**
   * @brief channelOptions Channels to extract.
   *
   * Channels to extract, see laser_geometry.h for details.
   * 0x00 - no channels enabled,
   * 0x01 - enable intensity (default),
   * 0x02 - enable index (default),
   * 0x04 - enable distance,
   * 0x08 - enable stamps,
   * 0x10 - enable viewpoint.
   */
  int channelOptions;
  /** @brief Scan queue size. */
  int scanQueue;
  /** @brief Point cloud queue size. */
  int pointCloudQueue;
  tf::TransformListener transformListener;
  ros::Subscriber scanSubscriber;
  laser_geometry::LaserProjection projector;
  ros::Publisher pointCloudPublisher;
};

ScanToPointCloud::ScanToPointCloud():
  targetFrame(""),
  waitForTransform(0.0),
//  channelOptions(laser_geometry::channel_option::Default),
  channelOptions(laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Index | laser_geometry::channel_option::Timestamp),
  scanQueue(10),
  pointCloudQueue(10),
  transformListener() {
}

ScanToPointCloud::~ScanToPointCloud() {
}

void ScanToPointCloud::onInit() {
  NODELET_INFO("ScanToPointCloud::onInit: Initializing...");

#ifdef USE_MT_NODE_HANDLE
  ros::NodeHandle &nh = getMTNodeHandle();
  ros::NodeHandle &pnh = getMTPrivateNodeHandle();
#else
  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &pnh = getPrivateNodeHandle();
#endif

  // Process parameters.
  pnh.param("target_frame", targetFrame, targetFrame);
  NODELET_INFO("ScanToPointCloud::onInit: Using target frame: %s.", targetFrame.data());
  pnh.param("wait_for_transform", waitForTransform, waitForTransform);
  NODELET_INFO("ScanToPointCloud::onInit: Maximum time to wait for transform: %.2f s.", waitForTransform);
  pnh.param("channel_options", channelOptions, channelOptions);
  NODELET_INFO("ScanToPointCloud::onInit: Channel options: %#x.", channelOptions);
  pnh.param("scan_queue", scanQueue, scanQueue);
  NODELET_INFO("ScanToPointCloud::onInit: Scan queue size: %i.", scanQueue);
  pnh.param("point_cloud_queue", pointCloudQueue, pointCloudQueue);
  NODELET_INFO("ScanToPointCloud::onInit: Point cloud queue size: %i.", pointCloudQueue);

  // Subscribe scan topic.
  std::string scanTopic = nh.resolveName("scan", true);
  NODELET_INFO("ScanToPointCloud::onInit: Subscribing scan %s.", scanTopic.data());
  scanSubscriber = nh.subscribe<sensor_msgs::LaserScan>(scanTopic, scanQueue, &ScanToPointCloud::scanCallback, this);

  // Advertise scan point cloud.
  std::string cloudTopic = nh.resolveName("cloud", true);
  NODELET_INFO("ScanToPointCloud::onInit: Advertising point cloud %s.", cloudTopic.data());
  pointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>(cloudTopic, pointCloudQueue, false);
}

void ScanToPointCloud::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scanPtr) {
  ros::Time t1 = ros::Time::now();
  // NODELET_DEBUG("ScanToPointCloud::scanCallback: Point cloud received (%lu points).", scanPtr->ranges.size());
  // Wait for the transform if the target frame differs from that of the scan.
  std::string frame = targetFrame.size() ? targetFrame : scanPtr->header.frame_id;
  if (frame != scanPtr->header.frame_id
      && !transformListener.waitForTransform(frame, scanPtr->header.frame_id,
                                             scanPtr->header.stamp + ros::Duration(scanPtr->scan_time),
                                             ros::Duration(waitForTransform))) {
    NODELET_WARN("ScanToPointCloud::scanCallback: Cannot transform from %s to %s at %.2f s.",
                 scanPtr->header.frame_id.c_str(), frame.c_str(), scanPtr->header.stamp.toSec());
    return;
  }
  sensor_msgs::PointCloud2::Ptr cloud2(new sensor_msgs::PointCloud2);
  try {
    projector.transformLaserScanToPointCloud(frame, *scanPtr, *cloud2, transformListener, -1.0, channelOptions);
    pointCloudPublisher.publish(cloud2);
    // NODELET_DEBUG("ScanToPointCloud::scanCallback: Converting scan to point cloud: %.3f s.",
    // (ros::Time::now() - t1).toSec());
  } catch(tf::TransformException& ex) {
    ROS_ERROR("ScanToPointCloud::scanCallback: Transform exception: %s.", ex.what());
    return;
  }
}

} /* namespace point_cloud_color */

PLUGINLIB_DECLARE_CLASS(point_cloud_color, scan_to_point_cloud, point_cloud_color::ScanToPointCloud, nodelet::Nodelet)
