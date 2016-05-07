/**
 * ROS node.
 * Converts other messages to omnicamera/VirtualCameraConfig.
 * Supported input messages sensor_msgs/JointState. Other may be added when needed.
 *
 * Author: Tomas Petricek
 */

#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
//#include <ViewControllers/PTZCamViewController.h>
#include <omnicamera_msgs/VirtualCameraConfig.h>

using std::string;

using ros::Publisher;
using ros::Subscriber;
//using rviz::PTZCamViewController;
using sensor_msgs::JointState;

using omnicamera_msgs::VirtualCameraConfig;


namespace {
// Conversion mode from sensor_msgs/JointState to omnicamera/VirtualCameraConfig
const int MODE_JOINT_STATE_IN = 0;
int mode = MODE_JOINT_STATE_IN;

string inTopic = "in";
string outTopic = "out";

Subscriber jointStateSub;

VirtualCameraConfig out;
Publisher pub;

// Callback for MODE_JOINT_STATE_IN
void jointStateCallback(const JointState::ConstPtr& msg) {
  // PTZCamViewController::PAN_SLOT, PTZCamViewController::TILT_SLOT protected...
//  out.pan = msg->position[PTZCamViewController::PAN_SLOT];
//  out.tilt = msg->position[PTZCamViewController::TILT_SLOT];
  out.pan = msg->position[0];
  out.tilt = msg->position[1];

  pub.publish(out);
}
} // anonymous namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "virtual_camera_conv");
  ros::NodeHandle nh;

  // Construct a node handle with private ns to get private parameters...
  ros::NodeHandle pnh("~");

  // Initialize default values for virtual camera config.
  pnh.param(string("viewport_width"), out.viewportWidth, 512);
  pnh.param(string("viewport_height"), out.viewportHeight, 512);
  pnh.param(string("image_update_interval"), out.imageUpdateInterval, 3.0);
  pnh.param(string("pan"), out.pan, 0.0);
  pnh.param(string("tilt"), out.tilt, 0.0);
  pnh.param(string("horizontal_fov"), out.horizontalFov, M_PI / 180 * 90);
  pnh.param(string("vertical_fov"), out.verticalFov, M_PI / 180 * 90);

  // Conversion mode.
  pnh.param(string("mode"), mode, MODE_JOINT_STATE_IN);
  ROS_INFO("Conversion mode set to %d.", mode);
  switch (mode) {
  case MODE_JOINT_STATE_IN:
  default:
    jointStateSub = nh.subscribe(inTopic, 1, jointStateCallback);
  }

  pub = nh.advertise<VirtualCameraConfig>(outTopic, 1);

  ros::spin();

  return 0;
}
