
#include <boost/format.hpp>
#include <boost/make_shared.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <camera_info_manager/camera_info_manager.h>

#include "omnicamera/LadySaw.h"

//#define USE_MT_NODE_HANDLE

using std::string;

namespace omnicamera {

const int N_CAMS = 6;

class LadySawNodelet : public nodelet::Nodelet {
public:
  LadySawNodelet();
  virtual void onInit();
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
private:
  omnicamera::LadySaw ladySaw;
  vector<Mat> cams;

  // Camera names parametrized by whatever affects calibration, e.g. GUID, capture mode.
  vector<camera_info_manager::CameraInfoManager*> cameraInfoManagers;
  string frameIdFormat;
  /**
   * Image transport, seems to be needed to avoid image transport subscribers/publishers going out of scope.
   */
  boost::shared_ptr<image_transport::ImageTransport> it;
  image_transport::Subscriber imageSub;
  image_transport::Publisher outPubs[N_CAMS];
  ros::Publisher camPubs[N_CAMS];
};

LadySawNodelet::LadySawNodelet() {
}

void LadySawNodelet::onInit() {
  NODELET_INFO("Initializing LadySawNodelet...");
#ifdef USE_MT_NODE_HANDLE
  ros::NodeHandle &nh = getMTNodeHandle();
  ros::NodeHandle &pnh = getMTPrivateNodeHandle();
#else
  ros::NodeHandle &nh = getNodeHandle();
  ros::NodeHandle &pnh = getPrivateNodeHandle();
#endif

  bool rotate;
  pnh.param("rotate", rotate, true);
  ladySaw.setRotate(rotate);
  int width, height;
  pnh.param("width", width, 1232);
  pnh.param("height", height, 1616);
  ladySaw.setWidth(width);
  ladySaw.setHeight(height);

//  image_transport::ImageTransport it(nh);
  it.reset(new image_transport::ImageTransport(nh));

  // Get camera name and frame ID formats.
  string cameraNameFmtStr;
  pnh.param(string("camera_name_format"), cameraNameFmtStr, string("camera_%d"));
  boost::format cameraNameFmt(cameraNameFmtStr);
  pnh.param(string("frame_id_format"), frameIdFormat, string("camera_%d"));

  // Get camera info URL format, parameterized by camera name.
  string cameraInfoUrlFmtStr;
  pnh.param(string("camera_info_url_format"), cameraInfoUrlFmtStr, string("package://omnicamera/res/camera_info/%s.ini"));
  boost::format cameraInfoUrlFmt(cameraInfoUrlFmtStr);

  boost::format cameraNsFmt("camera_%i");

  for (int i = 0; i < N_CAMS; i++) {
    ros::NodeHandle cnh = ros::NodeHandle(nh, (cameraNsFmt % i).str());
    string cameraName = (cameraNameFmt % i).str();
    string cameraInfoUrl = (cameraInfoUrlFmt % cameraName).str();
    NODELET_INFO("Setting up camera info manager for %s, %s...", cameraName.data(), cameraInfoUrl.data());
    cameraInfoManagers.push_back(new camera_info_manager::CameraInfoManager(cnh, cameraName.data(), cameraInfoUrl.data()));
    string imageOutTopic = cnh.resolveName("image", true);
    string camInfoTopic = cnh.resolveName("camera_info", true);
    NODELET_INFO("Advertising topics %s and %s...", imageOutTopic.data(), camInfoTopic.data());
    outPubs[i] = it->advertise(imageOutTopic, 1);
    camPubs[i] = nh.advertise<sensor_msgs::CameraInfo>(camInfoTopic, 1);
  }

  string imageTopic = nh.resolveName("camera/image", true);
  NODELET_INFO("Subscribing to topic %s...", imageTopic.data());
  imageSub = it->subscribe(imageTopic, 1, &LadySawNodelet::imageCallback, this);
}

void LadySawNodelet::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
//  NODELET_INFO("Entering image callback...");

  vector<int> camsOut;
  for (int i = 0; i < N_CAMS; i++) {
    if (outPubs[i].getNumSubscribers() > 0 || camPubs[i].getNumSubscribers() > 0) {
      camsOut.push_back(i);
    }
  }
  if (camsOut.size() == 0) {
    return;
  }
  ladySaw.setCamsOut(camsOut);

  cv_bridge::CvImageConstPtr cvPtr;
  try {
    cvPtr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception& e) {
    NODELET_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  NODELET_DEBUG("Sawing started.");
  ladySaw.saw(cvPtr->image, cams);
  NODELET_DEBUG("Sawing finished.");

  boost::format frameIdFmt(frameIdFormat);
  camsOut = ladySaw.getCamsOut();
  for (size_t i = 0; i < camsOut.size(); i++) {
    int iCam = camsOut[i];
    string frameId = (frameIdFmt % iCam).str();

    cv_bridge::CvImage out;
    out.encoding = cvPtr->encoding;
    out.header = msg->header;
    out.header.frame_id = frameId;
    out.image = cams[i];
    outPubs[iCam].publish(out.toImageMsg());

    sensor_msgs::CameraInfo camMsg = cameraInfoManagers[iCam]->getCameraInfo();
    camMsg.header = out.header;
    camMsg.height = out.image.rows;
    camMsg.width = out.image.cols;
    camPubs[iCam].publish(camMsg);
  }
}

} // namespace omnicamera

PLUGINLIB_DECLARE_CLASS(omnicamera, lady_saw, omnicamera::LadySawNodelet, nodelet::Nodelet)
