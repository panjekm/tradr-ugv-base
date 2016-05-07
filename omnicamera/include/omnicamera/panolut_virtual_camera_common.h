
#include <opencv2/core/core.hpp>

#include "omnicamera/LookupStitcher.h"
#include "omnicamera_msgs/VirtualCameraConfig.h"

using cv::Mat;
using omnicamera_msgs::VirtualCameraConfig;

namespace omnicamera {

void rotationMatrix(const double x, const double y, const double z, const double angle, Mat &R);
void panTiltToCart(const double pan, const double tilt, double &x, double &y, double &z);
/**
 * @param x point x coordinate
 * @param y point y coordinate
 * @param z point z coordinate
 * @param pan in range <-PI, PI>, output
 * @param tilt in range <-PI, PI>, output
 */
void cartToPanTilt(const double x, const double y, const double z, double &pan, double &tilt);
/**
 * @param width image width
 * @param height image height
 * @param fovx field of view along x axis, in radians
 * @param fovy field of view along y axis, in radians
 */
void cameraMatrix(const double fovx, const double fovy, const double width, const double height, Mat &camMat);
void cameraInverse(const double pan, const double tilt, const double fovx, const double fovy, const double width, const double height, Mat &camInv);
void updateVcamStitcher(const omnicamera::LookupStitcher &pano, const int panoWidth, const int panoHeight, const VirtualCameraConfig &vcamConfig, omnicamera::LookupStitcher &vcam);

} // namespace omnicamera
