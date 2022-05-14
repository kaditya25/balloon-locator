#include "structurecomputer.h"

#include <Eigen/LU>

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

int computeStructureHelper(const int s, const double dt, const SensorParams& sp, const std::vector<std::shared_ptr<const CameraBundle>>& bV, Point& p) {
	// Throw an error if there are fewer than 2 CameraBundles in bundleVec_,
	// since in this case structure computation is not possible.
	if (s < 2) {
		throw std::runtime_error(
				"At least 2 CameraBundle objects are "
				"needed for structure computation.");
	}

	Eigen::MatrixXd H(2*s,4);
	Eigen::VectorXd R_diag(2*s);

	int support = 0;

	for (int i=0;i<s;i++) {
		Eigen::Matrix<double, 3, 4> P_i;
		P_i.col(0) = bV[i]->RCI.col(0);
		P_i.col(1) = bV[i]->RCI.col(1);
		P_i.col(2) = bV[i]->RCI.col(2);
		P_i.col(3) = -(bV[i]->RCI)*(bV[i]->rc_I);
		P_i = sp.K()*P_i;
		Eigen::Vector2d rx_i = bV[i]->rx;
		H.row(2*i) = sp.pixelSize()*rx_i(0)*P_i.row(2)-P_i.row(0);
		H.row(2*i+1) = sp.pixelSize()*rx_i(1)*P_i.row(2)-P_i.row(1);
		R_diag(2*i) = sp.Rc()(0,0);
		R_diag(2*i+1) = sp.Rc()(1,1);
	}

	Eigen::MatrixXd Hr(2*s,3);
	Hr << H.col(0), H.col(1), H.col(2);
	Eigen::VectorXd z(2*s);
	z = -H.col(3);

	Eigen::DiagonalMatrix<double, Eigen::Dynamic> R(2*s);
	R.diagonal() = std::pow(sp.pixelSize(),2)*R_diag;
	
	Eigen::MatrixXd Rinv = R.inverse();

	p.rXIHat = (Hr.transpose() * Rinv * Hr).ldlt().solve(Hr.transpose() * Rinv * z);
	p.Px = (Hr.transpose() * Rinv * Hr).inverse();

	for (int i=0;i<s;i++) {
		Eigen::Vector2d dist = bV[i]->rx - backProject(bV[i]->RCI,bV[i]->rc_I,p.rXIHat);
		if (dist.norm()<dt) {support+=1;}
	}

	return support;
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

	SensorParams sp;
	int min_samples = 2; // Minimal subset
	double dist_thresh = 25.0; // Distance Threshold
	int supp_thresh = 5; // Support Threshold
	int max_support = 0;
	int i_max = 0;
	for (int i = min_samples; i<=bundleVec_.size(); i++) {
		int curr_support = computeStructureHelper(i,dist_thresh,sp,bundleVec_,point_);
		if (curr_support > max_support) {
			max_support = curr_support;
			i_max = i;
		}
	}
	if (max_support > supp_thresh) {
		std::cout << "Using RANSAC" << std::endl;
		max_support = computeStructureHelper(i_max,dist_thresh,sp,bundleVec_,point_);
	}
	else{
		max_support = computeStructureHelper(bundleVec_.size(),dist_thresh,sp,bundleVec_,point_);
	}
	std::cout << "i: " << i_max << std::endl;
	std::cout << "S_i: " << max_support << std::endl;

	return point_;
}
