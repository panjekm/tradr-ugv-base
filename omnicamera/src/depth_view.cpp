/*
  A viewer for Kinect Depth images
  Copyright (c) 2011 Michael Ferguson.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
 * Neither the name of the copyright holders nor the names of its
        contributors may be used to endorse or promote products derived
        from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * Modified by Tomas Petricek:
 * Private namespace, min and max depth parametrized etc.
 *
 * http://www.ros.org/doc/api/depth_viewer/html/depth__viewer_8cpp_source.html
 * http://answers.ros.org/question/869/kinect-opencv
 * http://answers.ros.org/question/478/disparity-image-publishing
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <sensor_msgs/Image.h>

namespace {
const std::string WINDOW_NAME = "Depth View";
double min_range_;
double max_range_;

void depthCb(const sensor_msgs::ImageConstPtr& image) {
  // convert to cv image
  cv_bridge::CvImagePtr bridge;
  try
  {
    bridge = cv_bridge::toCvCopy(image, "32FC1");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Failed to transform depth image.");
    return;
  }

  // convert to something visible
  cv::Mat img(bridge->image.rows, bridge->image.cols, CV_8UC1);
  for(int i = 0; i < bridge->image.rows; i++)
  {
    float* Di = bridge->image.ptr<float>(i);
    char* Ii = img.ptr<char>(i);
    for(int j = 0; j < bridge->image.cols; j++)
    {
      Ii[j] = (char) (255*((Di[j]-min_range_)/(max_range_-min_range_)));
    }
  }

  // display
  cv::imshow(WINDOW_NAME, img);
}
}

int main(int argc, char* argv[]) {
  ros::init( argc, argv, "depth_viewer" );
  ros::NodeHandle n;

  ros::NodeHandle nh("~");
  nh.param("min_range", min_range_, 0.5);
  nh.param("max_range", max_range_, 10.0);

  cv::namedWindow(WINDOW_NAME);
  cv::startWindowThread();

  ros::Subscriber sub = n.subscribe("/camera/depth/image", 3, &depthCb);
  ros::spin();
}
