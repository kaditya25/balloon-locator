#include "structurecomputer.h"

#include <Eigen/LU>
#include <Eigen/Core>

Eigen::MatrixXd blk(const Eigen::MatrixXd& a, int count)
{Eigen::MatrixXd dia_mat = Eigen::MatrixXd::Zero(a.rows() * count, a.cols() * count);
    for (int i = 0; i < count; ++i){
      dia_mat.block(i * a.rows(), i * a.cols(), a.rows(), a.cols()) = a;
    }
    return dia_mat;
}
void pr(Eigen::MatrixXd m) { std::cout << m << std::endl; }
void pr(Eigen::VectorXd m) { std::cout << m << std::endl; }
void pr(Eigen::Matrix3d m) { std::cout << m << std::endl; }
void pr(Eigen::Vector3d m) { std::cout << m << std::endl; }
void pr(Eigen::Vector2d m) { std::cout << m << std::endl; }

Eigen::Vector2d backProject(const Eigen::Matrix3d& RCI,
                            const Eigen::Vector3d& rc_I,
                            const Eigen::Vector3d& X3d) {
  using namespace Eigen;
  Vector3d t = -RCI * rc_I;
  MatrixXd Pextrinsic(3, 4);
  Pextrinsic << RCI, t;
  SensorParams sp;
  MatrixXd Pc = sp.K() * Pextrinsic;
  VectorXd X(4, 1);
  X.head(3) = X3d;
  X(3) = 1;
  Vector3d x = Pc * X;
  Vector2d xc_pixels = (x.head(2) / x(2)) / sp.pixelSize();
  return xc_pixels;
}

Eigen::Vector3d pixelsToUnitVector_C(const Eigen::Vector2d& rPixels) {
  using namespace Eigen;
  SensorParams sp;
  // Convert input vector to meters
  Vector2d rMeters = rPixels * sp.pixelSize();
  // Write as a homogeneous vector, with a 1 in 3rd element
  Vector3d rHomogeneous;
  rHomogeneous.head(2) = rMeters;
  rHomogeneous(2) = 1;
  // Invert the projection operation through the camera intrinsic matrix K to
  // yield a vector rC in the camera coordinate frame that has a Z value of 1
  Vector3d rC = sp.K().lu().solve(rHomogeneous);
  // Normalize rC so that output is a unit vector
  return rC.normalized();
}

void StructureComputer::clear() {
  // Zero out contents of point_
  point_.rXIHat.fill(0);
  point_.Px.fill(0);
  // Clear bundleVec_
  bundleVec_.clear();
}

void StructureComputer::push(std::shared_ptr<const CameraBundle> bundle) {
  bundleVec_.push_back(bundle);
}

// This function is where the computation is performed to estimate the
// contents of point_.  The function returns a copy of point_.
//
Point StructureComputer::computeStructure() {
  // Throw an error if there are fewer than 2 CameraBundles in bundleVec_,
  // since in this case structure computation is not possible.
  if (bundleVec_.size() < 2) {
    throw std::runtime_error(
        "At least 2 CameraBundle objects are "
        "needed for structure computation.");
  }

  // *********************************************************************
  // Fill in here the required steps to calculate the 3D position of the
  // feature point and its covariance.  Put these respectively in
  // point_.rXIHat and point_.Px
  // *********************************************************************

  // float p_s = SensorParams.pixelSize();
  // Eigen::MatrixXd H(bundleVec_.size()*2, 4);
  // for (int ii=0; ii<bundleVec_.size(); ii++){
  //   Eigen::MatrixXd t_I = bundleVec_.rc_I(ii)
  //   Eigen::MatrixXd R_CI = bundleVec_.RCI(ii)
  //   float x = p_s*bundleVec_.rxArray(ii)(1); 
  //   float y = p_s*bundleVec_.rxArray(ii)(2);
  //   Eigen::MatrixXd P_mat = SensorParams.K*[R_CI, -R_CI*t_I];
  //   H(ii) = [x*P_mat(3,:) - P_mat(1,:); y*P_mat(3,:) - P_mat(2,:);];

  // }
  // H_r = H.block(0,0,bundleVec_.()size*2,3)
  // z = - H.block(0,2,bundleVec_.()size*2,1)

  // R  = pow(sensorParams_.pixelSize(),2) * blkdiag( sensorParams_.Rc(), nf); 

  // Eigen::MatrixXd Rinv = R.inverse();
  // point_.Px     = (Hr.transpose() * Rinv * Hr).inverse();
  // point_.rXIHat = (Hr.transpose() * Rinv * Hr).ldlt().solve(Hr.transpose() * Rinv * z);

  // return point_;
  Eigen::MatrixXd H(bundleVec_.size()*2, 4);
  
  size_t ii = 0;
  for (auto cam_n:bundleVec_)
  {
    Eigen::MatrixXd P_mat(4-1,4); 
    P_mat.block(0,0,3,3) = cam_n->RCI;
    P_mat.block(0,3,3,1) = -cam_n->RCI*cam_n->rc_I;
    auto P = sensorParams_.K()*P_mat;
    H.row(ii) = sensorParams_.pixelSize()*cam_n->rx[0]*P.row(2) - P.row(0);
    ii = ii + 1;
    H.row(ii) = sensorParams_.pixelSize()*cam_n->rx[1]*P.row(2) - P.row(1);
    ii = ii + 1;
  }

  Eigen::MatrixXd R(2*bundleVec_.size(), 2*bundleVec_.size());
  R  = pow(sensorParams_.pixelSize(),2) * blk(sensorParams_.Rc(), bundleVec_.size());

  Eigen::VectorXd z(2*bundleVec_.size());  
  z  = -H.col(3);   

  Eigen::MatrixXd H_r(2*bundleVec_.size(), 4-1);
  H_r = H.block(0,0,2*bundleVec_.size(),3);
  
  
  Eigen::MatrixXd Rinv = R.inverse();
  point_.Px     = (H_r.transpose() * Rinv * H_r).inverse();
  point_.rXIHat = (H_r.transpose() * Rinv * H_r).ldlt().solve(H_r.transpose() * Rinv * z);

  return point_;
}
