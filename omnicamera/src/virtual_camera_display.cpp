/*
 * virtual_camera_display.cpp
 *
 *  Created on: Aug 25, 2010
 *      Author: petrito1@cmp.felk.cvut.cz
 *
 * ROS node for displaying images from virtual camera (e.g. used with omnicamera node).
 * Publishes virtual camera configuration on topic "virtual_camera/config",
 * subscribes for images from "virtual_camera/image".
 */

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <omnicamera_msgs/VirtualCameraConfig.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

using std::string;
using omnicamera_msgs::VirtualCameraConfig;

#define KEY_UP    0xff52
#define KEY_DOWN  0xff54
#define KEY_LEFT  0xff51
#define KEY_RIGHT 0xff53
#define KEY_Q     0x71
#define KEY_A     0x61
#define KEY_W     0x77
#define KEY_S     0x73
#define KEY_E     0x65
#define KEY_D     0x64

// Zoom sensitivity = 2^(1/4)
#define ZOOM_SENSITIVITY        1.189207115
#define VIEWPORT_CHANGE_FACTOR  1.189207115
// Rotation sensitivity in terms of FOV
#define ROTATION_SENSITIVITY    0.25

namespace {
const string nodeName = "virtual_camera_display";
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  cv_bridge::CvImageConstPtr cvPtr;
  try {
    cvPtr = cv_bridge::toCvShare(msg);
    cv::imshow(nodeName, cvPtr->image);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, nodeName);

  printf("Controls:\n");
  printf("left and right arrows: pan\n");
  printf("up and down arrows:    tilt\n");
  printf("'q' and 'a':           zoom in and zoom out\n");
  printf("'w' and 's':           raise or lower image update interval\n");
  printf("'e' and 'd':           scale viewport up or down\n");

  ros::NodeHandle nh;
  ros::Publisher p = nh.advertise<VirtualCameraConfig> ("virtual_camera/config", 1);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("virtual_camera/image", 1, imageCallback);

  cv::namedWindow(nodeName);
  cv::startWindowThread();

  // Init config to default values.
  ros::NodeHandle pnh("~");
  VirtualCameraConfig config;
  pnh.param("viewport_width", config.viewportWidth, 512);
  pnh.param("viewport_height", config.viewportHeight, 512);
  pnh.param("pan", config.pan, 0.0);
  pnh.param("tilt", config.tilt, 0.0);
  pnh.param("horizontal_fov", config.horizontalFov, 90.0);
  pnh.param("vertical_fov", config.verticalFov, 90.0);
  pnh.param("image_update_interval", config.imageUpdateInterval, 0.0);

  while (nh.ok()) {
    ros::spinOnce();
    int keyPressed = cv::waitKey(100);
    if (keyPressed >= 0) {
      printf("Key pressed: %#x %d %d %d %d.\n", keyPressed, (keyPressed >> 24) & 0xff, (keyPressed >> 16) & 0xff,
          (keyPressed >> 8) & 0xff, keyPressed & 0xff);
      switch (keyPressed & 0xffff) {
      case KEY_LEFT:
        config.pan += config.horizontalFov * ROTATION_SENSITIVITY;
        p.publish(config);
        break;
      case KEY_RIGHT:
        config.pan -= config.horizontalFov * ROTATION_SENSITIVITY;
        p.publish(config);
        break;
      case KEY_UP:
        config.tilt += config.verticalFov * ROTATION_SENSITIVITY;
        p.publish(config);
        break;
      case KEY_DOWN:
        config.tilt -= config.verticalFov * ROTATION_SENSITIVITY;
        p.publish(config);
        break;
      case KEY_Q:
        config.horizontalFov /= ZOOM_SENSITIVITY;
        config.verticalFov /= ZOOM_SENSITIVITY;
        p.publish(config);
        break;
      case KEY_A:
        config.horizontalFov *= ZOOM_SENSITIVITY;
        config.verticalFov *= ZOOM_SENSITIVITY;
        p.publish(config);
        break;
      case KEY_W:
        config.imageUpdateInterval += 1;
        printf("Image update interval changed to %f sec.\n", config.imageUpdateInterval);
        p.publish(config);
        break;
      case KEY_S:
        config.imageUpdateInterval -= 1;
        if (config.imageUpdateInterval < 0) {
          config.imageUpdateInterval = 0;
        }
        printf("Image update interval changed to %f sec.\n", config.imageUpdateInterval);
        p.publish(config);
        break;
      case KEY_E:
        config.viewportWidth = static_cast<int32_t> (config.viewportWidth * VIEWPORT_CHANGE_FACTOR);
        config.viewportHeight = static_cast<int32_t> (config.viewportHeight * VIEWPORT_CHANGE_FACTOR);
        p.publish(config);
        break;
      case KEY_D:
        config.viewportWidth = static_cast<int32_t> (config.viewportWidth / VIEWPORT_CHANGE_FACTOR);
        config.viewportHeight = static_cast<int32_t> (config.viewportHeight / VIEWPORT_CHANGE_FACTOR);
        p.publish(config);
        break;
      }
    }
  }

  cv::destroyWindow(nodeName.data());
}

