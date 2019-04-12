#ifndef __SENSOR_PARAMS_H
#define __SENSOR_PARAMS_H

#include <Eigen/Dense>

struct SensorParams {
  // Pixel size, in meters
  const double pixelSize = 2e-6;
  // Distance of the image plane along camera z axis, in meters
  const double f = 0.004;
  // Camera intrinsic matrix
  const Eigen::Matrix3d K = Eigen::Vector3d(f,f,1).asDiagonal();
  // 2x2 error covariance matrix for the Gaussian image noise, in pixels^2
  const Eigen::Matrix2d Rc = Eigen::Vector2d(20*20,20*20).asDiagonal();
};

#endif
