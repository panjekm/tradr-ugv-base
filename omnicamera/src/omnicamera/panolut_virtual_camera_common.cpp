
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "omnicamera/panolut_virtual_camera_common.h"

using cv::Mat_;

namespace omnicamera {

void rotationMatrix(const double x, const double y, const double z, const double angle, Mat &R) {

  double norm = sqrt(x*x + y*y + z*z);
  double ux = x / norm;
  double uy = y / norm;
  double uz = z / norm;
  double c = cos(angle);
  double s = sin(angle);

  R = (Mat_<double>(3, 3) <<
         c+ux*ux*(1-c), ux*uy*(1-c)+uz*s, uz*ux*(1-c)-uy*s,
      ux*uy*(1-c)-uz*s,    c+uy*uy*(1-c), uz*uy*(1-c)+ux*s,
      ux*uz*(1-c)+uy*s, uy*uz*(1-c)-ux*s,    c+uz*uz*(1-c));
}

void panTiltToCart(const double pan, const double tilt, double &x, double &y, double &z) {
  x = cos(tilt) * cos(pan);
  y = cos(tilt) * sin(pan);
  z = sin(tilt);
}

void cartToPanTilt(const double x, const double y, const double z, double &pan, double &tilt) {
  tilt = atan2(z, hypot(x, y));
  pan = atan2(y, x);
}

void cameraMatrix(const double fovx, const double fovy, const double width, const double height, Mat &camMat) {
  double fx = 1 / tan(fovx / 2);
  double fy = 1 / tan(fovy / 2);
  double mx = width / 2;
  double my = height / 2;
  camMat = (Mat_<double> (3, 3) <<
      fx * mx,       0, mx,
            0, fy * my, my,
            0,       0,  1);
}

void cameraInverse(const double pan, const double tilt, const double fovx, const double fovy, const double width, const double height, Mat &camInv) {
  double x, y, z;
  panTiltToCart(pan, tilt, x, y, z);
//  if (pan == 0.0 && tilt == 0.0) {
//    printf("cameraInverse: pan %f, tilt %f corresponds to (%f, %f, %f)\n", pan, tilt, x, y, z);
//  }

  Mat panR, tiltR, camR1, camR2;
  rotationMatrix(0, 0, 1, pan, panR);
  rotationMatrix(0, 1, 0, -tilt, tiltR);
  Mat camInWorld = (Mat_<double>(3, 3) << 0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0);
  Mat camR = camInWorld * tiltR * panR;

  Mat K;
  cameraMatrix(fovx, fovy, width, height, K);

  camInv = (K * camR).inv(cv::DECOMP_SVD);
}

void updateVcamStitcher(const omnicamera::LookupStitcher &pano, const int panoWidth, const int panoHeight, const VirtualCameraConfig &vcamConfig, omnicamera::LookupStitcher &vcam) {
  // Handle viewport change, resize all virtual camera lookup tables.
  if (vcam.lutX[0].cols != vcamConfig.viewportWidth || vcam.lutY[0].rows != vcamConfig.viewportHeight) {
    for (int i = 0; i < pano.getLutCount(); i++) {
      vcam.lutX[i].create(vcamConfig.viewportHeight, vcamConfig.viewportWidth, CV_16UC1);
      vcam.lutY[i].create(vcamConfig.viewportHeight, vcamConfig.viewportWidth, CV_16UC1);
      vcam.lutA[i].create(vcamConfig.viewportHeight, vcamConfig.viewportWidth, CV_8UC1);
    }
  }

  Mat camInv;
  double panRad = vcamConfig.pan / 180.0 * M_PI;
  double tiltRad = vcamConfig.tilt / 180.0 * M_PI;
  double fovxRad = vcamConfig.horizontalFov / 180.0 * M_PI;
  double fovyRad = vcamConfig.verticalFov / 180.0 * M_PI;
  cameraInverse(panRad, tiltRad, fovxRad, fovyRad, vcamConfig.viewportWidth, vcamConfig.viewportHeight, camInv);

  Mat p = Mat_<double>(3, 1);
  Mat d = Mat_<double>(3, 1);
  for (int r = 0; r < vcamConfig.viewportHeight; r++) {
    for (int c = 0; c < vcamConfig.viewportWidth; c++) {

      // Create a point in image coordinates.
      p.at<double>(0, 0) = c;
      p.at<double>(1, 0) = r;
      p.at<double>(2, 0) = 1;

      // Get the world-coordinates direction for that image pixel.
      d = camInv * p;
      d = d / cv::norm(d);

      // Transform to pan and tilt, and then to panorama coordinates.
      double x, y, z, pan, tilt;
      x = d.at<double>(0, 0);
      y = d.at<double>(1, 0);
      z = d.at<double>(2, 0);
      cartToPanTilt(x, y, z, pan, tilt);
//      if (r == vcamConfig.viewportHeight / 2 && c == vcamConfig.viewportWidth / 2) {
//        printf("Image coordinates (%d, %d) corresponds to world point (%f, %f, %f), pan %f, tilt %f.\n", c, r, x, y, z, pan / M_PI * 180.0, tilt / M_PI * 180.0);
//      }
      int panox = (int) round((0.5 + -pan / (2 * M_PI)) * (panoWidth - 1));
      int panoy = (int) round((0.5 + -tilt / M_PI) * (panoHeight - 1));
      panox = std::max(panox, 0);
      panox = std::min(panoWidth - 1, panox);
      panoy = std::max(panoy, 0);
      panoy = std::min(panoHeight - 1, panoy);

      for (int i = 0; i < pano.getLutCount(); i++) {
        vcam.lutX[i].at<ushort>(r, c) = pano.lutX[i].at<ushort>(panoy, panox);
        vcam.lutY[i].at<ushort>(r, c) = pano.lutY[i].at<ushort>(panoy, panox);
        vcam.lutA[i].at<uchar>(r, c) = pano.lutA[i].at<uchar>(panoy, panox);
      }
    }
  }
}

} // namespace omnicamera
