
//#define USE_MT_NODE_HANDLE

#include <laser_geometry/laser_geometry.h>
#include <nifti_pcl_common/point_types.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/common/io.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace point_cloud_color {

typedef nifti_pcl_common::LaserPoint PointOut;

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
  int scanQueue;
  int pointCloudQueue;
  tf::TransformListener transformListener;
  ros::Subscriber scanSubscriber;
  laser_geometry::LaserProjection projector;
  ros::Publisher pointCloudPublisher;
};

ScanToPointCloud::ScanToPointCloud() : targetFrame(""), waitForTransform(0.5), scanQueue(10), pointCloudQueue(10), transformListener() {
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
  NODELET_INFO("ScanToPointCloud::onInit: Using target frame %s.", targetFrame.data());
  pnh.param("wait_for_transform", waitForTransform, waitForTransform);
  NODELET_INFO("ScanToPointCloud::onInit: Maximum time to wait for transform %.2f s.", waitForTransform);
  pnh.param("scan_queue", scanQueue, scanQueue);
  NODELET_INFO("ScanToPointCloud::onInit: Scan queue size %i.", scanQueue);
  pnh.param("point_cloud_queue", pointCloudQueue, pointCloudQueue);
  NODELET_INFO("ScanToPointCloud::onInit: Point cloud queue size %i.", pointCloudQueue);

  // Subscribe scan topic.
  std::string scanTopic = nh.resolveName("scan", true);
  NODELET_INFO("ScanToPointCloud::onInit: Subscribing scan %s.", scanTopic.data());
  scanSubscriber = nh.subscribe<sensor_msgs::LaserScan>(scanTopic, scanQueue, &ScanToPointCloud::scanCallback, this);

  // Advertise scan point cloud.
  std::string pclTopic = nh.resolveName("scan_point_cloud", true);
  NODELET_INFO("ScanToPointCloud::onInit: Advertising point cloud %s.", pclTopic.data());
  pointCloudPublisher = nh.advertise<pcl::PointCloud<nifti_pcl_common::LaserPoint> >(pclTopic, pointCloudQueue, false);
}

void ScanToPointCloud::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scanPtr) {
  ros::Time t1 = ros::Time::now();
  NODELET_DEBUG("ScanToPointCloud::scanCallback: Point cloud received (%lu points).", scanPtr->ranges.size());
  std::string frame = targetFrame.size() > 0 ? targetFrame : scanPtr->header.frame_id;
  if (!transformListener.waitForTransform(frame, scanPtr->header.frame_id, scanPtr->header.stamp + ros::Duration(scanPtr->scan_time), ros::Duration(waitForTransform))) {
    NODELET_WARN("ScanToPointCloud::scanCallback: Cannot transform from %s to %s at %.2f s.", scanPtr->header.frame_id.data(), frame.data(), scanPtr->header.stamp.toSec());
    return;
  }
  sensor_msgs::PointCloud2 pclMsg;
  try {
//    projector.transformLaserScanToPointCloud(scanPtr->header.frame_id, *scanPtr, pclMsg, transformListener);
    int channelOptions = laser_geometry::channel_option::Intensity
        | laser_geometry::channel_option::Index
        | laser_geometry::channel_option::Timestamp
        | laser_geometry::channel_option::Viewpoint;
    projector.transformLaserScanToPointCloud(frame, *scanPtr, pclMsg, transformListener, -1.0, channelOptions);
    pcl::PointCloud<nifti_pcl_common::LaserProjectionPoint> pclProj;
    pcl::fromROSMsg(pclMsg, pclProj);
    // Convert relative time offsets to timestamps.
    pcl::PointCloud<nifti_pcl_common::LaserPoint> pclOut;
    pcl::copyPointCloud(pclProj, pclOut);
    for (size_t i = 0; i < pclProj.points.size(); ++i) {
      ros::Time pointStamp = pclProj.header.stamp + ros::Duration(pclProj.points[i].stamps);
      pclOut.points[i].sec = pointStamp.sec;
      pclOut.points[i].nsec = pointStamp.nsec;
    }
    pointCloudPublisher.publish(pclOut);
    NODELET_DEBUG("ScanToPointCloud::scanCallback: Converting scan to point cloud: %.3f s.", (ros::Time::now() - t1).toSec());

  } catch(tf::TransformException& ex) {
    ROS_ERROR("ScanToPointCloud::scanCallback: Transform exception: %s.", ex.what());
    return;
  }
}

} /* namespace point_cloud_color */

PLUGINLIB_DECLARE_CLASS(point_cloud_color, scan_to_point_cloud, point_cloud_color::ScanToPointCloud, nodelet::Nodelet)
