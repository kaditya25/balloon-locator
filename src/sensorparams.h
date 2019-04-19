#ifndef __SENSOR_PARAMS_H
#define __SENSOR_PARAMS_H

#include <Eigen/Dense>

struct SensorParams {
  // Pixel size, in meters
  const double pixelSize = 2e-6;
  // Distance of the image plane along camera z axis, in meters
  const double f = pixelSize*1657.72;
  // Camera principal point for 3840x2160 images, in meters
  const double cx = pixelSize*1920, cy = pixelSize*1080;
  // Camera intrinsic matrix. Note that all quantities in K have units of
  // meters. Divide f and c values by pixelSize to express in pixels.
  const Eigen::Matrix3d K = (Eigen::Matrix3d() <<
                             f, 0, cx,
                             0, f, cy,
                             0, 0,  1).finished();
  // Camera distortion coefficients in this order: k1, k2, p1, p2, k3
  const Eigen::Matrix<double,5,1> distortionCoeffs =
    (Eigen::Matrix<double,5,1>() << -0.0217226, 0, 0, 0, 0).finished();
  // 2x2 error covariance matrix for the Gaussian image noise, in pixels^2
  const Eigen::Matrix2d Rc = Eigen::Vector2d(20*20,20*20).asDiagonal();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
