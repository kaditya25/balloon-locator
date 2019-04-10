#include "structurecomputer.h"

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
  if(bundleVec_.size() < 2) {
    throw std::runtime_error("At least 2 CameraBundle objects are needed for structure computation.");
  }

  // Fill in below the required steps to calculate the 3D position of the
  // feature point and its covariance.  Put these respectively in
  // point_.rXIHat and point_.Px


  

  return point_;
}
