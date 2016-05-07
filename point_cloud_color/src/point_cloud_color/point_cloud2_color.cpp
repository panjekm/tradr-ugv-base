/**
 * Point cloud coloring from calibrated cameras.
 * Static image masks can be used to denote ROI for coloring.
 *
 * TODO: Mask out robot model dynamically.
 */

//#define USE_MT_NODE_HANDLE

#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <limits>
#include <nodelet/nodelet.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

namespace point_cloud_color {

int getFieldIndex(const sensor_msgs::PointCloud2 &cloud, const std::string &fieldName) {
  for (size_t i = 0; i < cloud.fields.size(); i++) {
    if (cloud.fields[i].name == fieldName) {
      return i;
    }
  }
  return -1;
}

size_t typeSize(const sensor_msgs::PointField::_datatype_type dt) {
  switch (dt) {
  case sensor_msgs::PointField::INT8:    return 1;
  case sensor_msgs::PointField::UINT8:   return 1;
  case sensor_msgs::PointField::INT16:   return 2;
  case sensor_msgs::PointField::UINT16:  return 2;
  case sensor_msgs::PointField::INT32:   return 4;
  case sensor_msgs::PointField::UINT32:  return 4;
  case sensor_msgs::PointField::FLOAT32: return 4;
  case sensor_msgs::PointField::FLOAT64: return 8;
  default:
    assert(false);
    return 0;
  }
}

cv::Mat rotationFromTransform(const tf::Transform &t) {
  return (cv::Mat_<double>(3, 3) << t.getBasis()[0][0], t.getBasis()[0][1], t.getBasis()[0][2],
                                    t.getBasis()[1][0], t.getBasis()[1][1], t.getBasis()[1][2],
                                    t.getBasis()[2][0], t.getBasis()[2][1], t.getBasis()[2][2]);
}

cv::Mat translationFromTransform(const tf::Transform &t) {
  return (cv::Mat_<double>(3, 1) << t.getOrigin()[0], t.getOrigin()[1], t.getOrigin()[2]);
}

cv::Mat matFromCloud(sensor_msgs::PointCloud2::Ptr cloud, const std::string fieldName, const int numElements) {
  const int numPoints = cloud->width * cloud->height;
  int fieldIndex = getFieldIndex(*cloud, fieldName);
  void *data = static_cast<void *>(&cloud->data[cloud->fields[fieldIndex].offset]);
  int matType;
  switch (cloud->fields[fieldIndex].datatype) {
  case sensor_msgs::PointField::FLOAT32: matType = CV_32FC1; break;
  case sensor_msgs::PointField::FLOAT64: matType = CV_64FC1; break;
  case sensor_msgs::PointField::UINT8:   matType = CV_8UC1;  break;
  default:
    assert(0);
    break;
  }
  return cv::Mat(numPoints, numElements, matType, data, cloud->point_step);
}

sensor_msgs::PointCloud2::Ptr createCloudWithColorFrom(const sensor_msgs::PointCloud2::ConstPtr inCloud) {
  // Create a copy to fix fields - use correct count 1 instead of 0.
//  sensor_msgs::PointCloud2 inCloudFixed = *inCloud;
  pcl::PCLPointCloud2 inCloudFixed;
  pcl_conversions::toPCL(*inCloud, inCloudFixed);
//  BOOST_FOREACH(sensor_msgs::PointField &f, inCloudFixed.fields) {
  BOOST_FOREACH(pcl::PCLPointField &f, inCloudFixed.fields) {
    if (f.count == 0) {
      f.count = 1;
    }
  }

//  sensor_msgs::PointField rgbField;
  pcl::PCLPointField rgbField;
//  rgbField.name = "argb";
//  rgbField.datatype = sensor_msgs::PointField::UINT32;
  rgbField.name = "rgb";
  rgbField.datatype = sensor_msgs::PointField::FLOAT32;
  rgbField.count = 1;
  rgbField.offset = 0;
//  sensor_msgs::PointCloud2 rgbCloud;
  pcl::PCLPointCloud2 rgbCloud;
  rgbCloud.header = inCloudFixed.header;
  rgbCloud.width = inCloudFixed.width;
  rgbCloud.height = inCloudFixed.height;
  rgbCloud.fields.push_back(rgbField);
  rgbCloud.is_bigendian = inCloudFixed.is_bigendian;
  rgbCloud.is_dense = inCloudFixed.is_dense;
  rgbCloud.point_step = 4;
  rgbCloud.row_step = rgbCloud.width * rgbCloud.point_step;
  rgbCloud.data.resize(rgbCloud.row_step * rgbCloud.height, 0);

//  pcl::concatenateFields(inCloudFixed, rgbCloud, *outCloud);
  pcl::PCLPointCloud2 outPcl;
  pcl::concatenateFields(inCloudFixed, rgbCloud, outPcl);

  sensor_msgs::PointCloud2::Ptr outCloud(new sensor_msgs::PointCloud2);
  pcl_conversions::moveFromPCL(outPcl, *outCloud);
  return outCloud;
}

/**http://answers.ros.org/question/97631/qt-creator-can-i-work-on-one-package-without-building-other-packages-in-same-catkin-workspace/
 * @brief A nodelet for coloring point clouds from calibrated cameras.
 */
class PointCloudColor : public nodelet::Nodelet {
public:
  PointCloudColor();
  virtual ~PointCloudColor();
  void onInit();
private:
  tf::TransformListener transformListener;
  std::vector<image_transport::CameraSubscriber> cameraSubscribers;
  ros::Subscriber pointCloudSub;
  ros::Publisher pointCloudPub;
  std::vector<cv_bridge::CvImage::ConstPtr> images;
  std::vector<sensor_msgs::CameraInfo::ConstPtr> cameraInfos;
  std::vector<cv::Mat> cameraMasks;
  float defaultColor;
  std::string fixedFrame;
  int numCameras;
  double maxImageAge;
  bool useFirstValid;
  int imageQueueSize;
  int pointCloudQueueSize;
  double waitForTransform;
  void cameraCallback(const sensor_msgs::Image::ConstPtr &imageMsg, const sensor_msgs::CameraInfo::ConstPtr &cameraInfoMsg, const int iCam);
  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloudMsg);
};

PointCloudColor::PointCloudColor() :
    transformListener(ros::Duration(10.0)),
    fixedFrame("/odom"),
    numCameras(1),
    maxImageAge(10.0),
    useFirstValid(true),
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
  pnh.param("fixed_frame", fixedFrame, fixedFrame);
  NODELET_INFO("PointCloudColor::onInit: Fixed frame: %s.", fixedFrame.c_str());
  std::string defaultColorStr("0x00000000");
  pnh.param("default_color", defaultColorStr, defaultColorStr);
  uint32_t defaultColorUl = 0xfffffffful & strtoul(defaultColorStr.c_str(), NULL, 0);
  defaultColor = *reinterpret_cast<float *>(&defaultColorUl);
  NODELET_INFO("PointCloudColor::onInit: Default color: %#x.", defaultColorUl);
  pnh.param("num_cameras", numCameras, numCameras);
  numCameras = numCameras >= 0 ? numCameras : 0;
  NODELET_INFO("PointCloudColor::onInit: Number of cameras: %i.", numCameras);
  pnh.param("max_image_age", maxImageAge, maxImageAge);
  NODELET_INFO("PointCloudColor::onInit: Maximum image age: %.1f s.", maxImageAge);
  pnh.param("use_first_valid", useFirstValid, useFirstValid);
  NODELET_INFO("PointCloudColor::onInit: Use first valid projection: %s.", useFirstValid ? "yes" : "no");
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
  cameraMasks.resize(numCameras);
  images.resize(numCameras);
  cameraInfos.resize(numCameras);
  for (int iCam = 0; iCam < numCameras; iCam++) {
    std::string cameraTopic = nh.resolveName((boost::format("camera_%i/image") % iCam).str(), true);
    std::string cameraMaskParam = (boost::format("camera_%i/mask") % iCam).str();
    std::string cameraMaskPath("");
    pnh.param(cameraMaskParam, cameraMaskPath, cameraMaskPath);
    if (!cameraMaskPath.empty()) {
      cameraMasks[iCam] = cv::imread(cameraMaskPath, CV_LOAD_IMAGE_GRAYSCALE);
      NODELET_INFO("PointCloudColor::onInit: Camera %i: Using camera mask from %s.", iCam, cameraMaskPath.c_str());
    }
    NODELET_INFO("PointCloudColor::onInit: Camera %i: Subscribing camera topic %s.", iCam, cameraTopic.c_str());
    cameraSubscribers[iCam] = it.subscribeCamera(cameraTopic, imageQueueSize, (boost::bind(&PointCloudColor::cameraCallback, this, _1, _2, iCam)));
  }

  // Subscribe point cloud topic.
  std::string pclInTopic = nh.resolveName("cloud_in", true);
  NODELET_INFO("PointCloudColor::onInit: Subscribing point cloud %s.", pclInTopic.c_str());
  pointCloudSub = nh.subscribe<sensor_msgs::PointCloud2>(pclInTopic, pointCloudQueueSize, &PointCloudColor::pointCloudCallback, this);

  // Advertise colored point cloud topic.
  std::string pclOutTopic = nh.resolveName("cloud_out", true);
  NODELET_INFO("PointCloudColor::onInit: Advertising colored point cloud %s.", pclOutTopic.c_str());
  pointCloudPub = nh.advertise<sensor_msgs::PointCloud2>(pclOutTopic, pointCloudQueueSize, false);
}

void PointCloudColor::cameraCallback(const sensor_msgs::Image::ConstPtr &imageMsg, const sensor_msgs::CameraInfo::ConstPtr &cameraInfoMsg, const int iCam) {
  NODELET_DEBUG("PointCloudColor::cameraCallback: Camera %i: Image frame: %s, camera info frame: %s.", iCam, imageMsg->header.frame_id.c_str(), cameraInfoMsg->header.frame_id.c_str());
  // Use frame id from image message for both image and camera info to ensure consistency.
  images[iCam] = cv_bridge::toCvShare(imageMsg, sensor_msgs::image_encodings::BGR8);
  cameraInfos[iCam] = cameraInfoMsg;
}

void PointCloudColor::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloudMsg) {
  const int numPoints = cloudMsg->width * cloudMsg->height;
  const std::string cloudFrame = cloudMsg->header.frame_id;
  if (numPoints == 0 || cloudMsg->data.size() == 0) {
    NODELET_WARN("PointCloudColor::pointCloudCallback: Skipping empty point cloud in frame %s.", cloudFrame.c_str());
    return;
  }
  sensor_msgs::PointCloud2::Ptr outCloud = createCloudWithColorFrom(cloudMsg);
  cv::Mat rgbMat = matFromCloud(outCloud, "rgb", 1);
  rgbMat.setTo(defaultColor);
  // Initialize vector with projection distances from image center, used as a quality measure.
  std::vector<float> dist(numPoints, std::numeric_limits<float>::infinity());
  for (int iCam = 0; iCam < numCameras; iCam++) {
    if (!images[iCam] || !cameraInfos[iCam]) {
      NODELET_WARN("PointCloudColor::pointCloudCallback: Camera image %i has not been received yet...", iCam);
      continue;
    }
    // Check relative age of the point cloud and the image. Skip the image if the time span is to large.
    const double imageAge = (outCloud->header.stamp - images[iCam]->header.stamp).toSec();
    if (imageAge > maxImageAge) {
      NODELET_WARN("PointCloudColor::pointCloudCallback: Skipping image %.1f (> %.1f) s older than point cloud...", imageAge, maxImageAge);
      continue;
    }
    // Wait for transform from cloud to image.
    if (!transformListener.waitForTransform(images[iCam]->header.frame_id, images[iCam]->header.stamp, // target frame and time
                                            cloudFrame, cloudMsg->header.stamp, // source frame and time
                                            fixedFrame, ros::Duration(waitForTransform))) {
      NODELET_WARN("PointCloudColor::pointCloudCallback: Could not transform point cloud from frame %s to camera frame %s. Skipping the image...",
                   cloudFrame.c_str(), images[iCam]->header.frame_id.c_str());
      continue;
    }
    tf::StampedTransform cloudToCamStamped;
    transformListener.lookupTransform(images[iCam]->header.frame_id, images[iCam]->header.stamp, // target frame and time
                                      cloudFrame, cloudMsg->header.stamp, // source frame and time
                                      fixedFrame, cloudToCamStamped);
    cv::Mat camRotation = rotationFromTransform(cloudToCamStamped);
    cv::Mat camTranslation = translationFromTransform(cloudToCamStamped);
    cv::Mat objectPoints;
    matFromCloud(outCloud, "x", 3).convertTo(objectPoints, CV_64FC1);
    // Rigid transform with points in rows: (X_C)^T = (R * X_L + t)^T.
    objectPoints = objectPoints * camRotation.t() + cv::repeat(camTranslation.t(), objectPoints.rows, 1);
    cv::Mat cameraMatrix(3, 3, CV_64FC1, const_cast<void *>(reinterpret_cast<const void *>(&cameraInfos[iCam]->K[0])));
    cv::Mat distCoeffs(1, static_cast<int>(cameraInfos[iCam]->D.size()), CV_64FC1, const_cast<void *>(reinterpret_cast<const void *>(&cameraInfos[iCam]->D[0])));
    cv::Mat imagePoints;
    cv::projectPoints(objectPoints.reshape(3), cv::Mat::zeros(3, 1, CV_64FC1), cv::Mat::zeros(3, 1, CV_64FC1), cameraMatrix, distCoeffs, imagePoints);

    for (int iPoint = 0; iPoint < numPoints; iPoint++) {
      // Continue if we already have got a color.
      if (useFirstValid && dist[iPoint] < DBL_MAX) {
        continue;
      }
      const double z = objectPoints.at<double>(iPoint, 2);
      // Skip points behind the camera.
      if (z <= 0.0) {
        continue;
      }
      const cv::Vec2d pt = imagePoints.at<cv::Vec2d>(iPoint, 0);
      const double x = round(pt.val[0]);
      const double y = round(pt.val[1]);
      // Skip points outside the image.
      if (x < 0.0 || y < 0.0 || x >= static_cast<double>(cameraInfos[iCam]->width) || y >= static_cast<double>(cameraInfos[iCam]->height)) {
        continue;
      }
      // Apply static mask with image ROI to be used for coloring.
      const int yi = static_cast<int>(y);
      const int xi = static_cast<int>(x);
      if (!cameraMasks[iCam].empty() && !cameraMasks[iCam].at<uint8_t>(yi, xi)) {
        // Pixel masked out.
        continue;
      }
      const float r = hypot(static_cast<float>(cameraInfos[iCam]->width / 2 - xi), static_cast<float>(cameraInfos[iCam]->height / 2 - yi));
      if (r >= dist[iPoint]) {
        // Keep color from the previous projection closer to image center.
        continue;
      }
      dist[iPoint] = r;
      const cv::Vec3b px = images[iCam]->image.at<cv::Vec3b>(static_cast<int>(y), static_cast<int>(x));
      uint32_t rgb = static_cast<uint32_t>(px.val[2]) << 16 | static_cast<uint32_t>(px.val[1]) << 8 | static_cast<uint32_t>(px.val[0]);
      rgbMat.at<float>(iPoint, 0) = *reinterpret_cast<float*>(&rgb);
    }
  }
  pointCloudPub.publish(outCloud);
}

} /* namespace point_cloud_color */

PLUGINLIB_DECLARE_CLASS(point_cloud_color, point_cloud_color, point_cloud_color::PointCloudColor, nodelet::Nodelet)
