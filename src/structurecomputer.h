#ifndef __STRUCTURE_COMPUTER_H
#define __STRUCTURE_COMPUTER_H

#include <Eigen/Dense>
#include <cstdlib>
#include <memory>

struct CameraBundle {
  // 2x1 coordinates of feature point as projected on the camera's image
  // plane, in pixels.
  Eigen::Vector2d rx;
  double tmp;
  // 3x3 attitude matrix relating the C and I frames at the instant the image
  // was taken.
  Eigen::Matrix3d RCI;
  // 3x1 position of the camera center at the instant the image was taken,
  // expressed in the I frame in meters.
  Eigen::Vector3d rc;
};

struct Point {
  // 3x1 estimated location of the feature point expressed in I in meters
  Eigen::Vector3d rXIHat;
  // 3x3 error covariance matrix for the estimate rxIHat
  Eigen::Matrix3d Re;
};

class StructureComputer {
public:
  // Constructor: gets called when an instance of StructureComputer is created
  StructureComputer() {}
  // Destructor: gets called when an instance of StructureComputer is destroyed
  ~StructureComputer() {}
  // Clears out contents; puts StructureComputer instance in initial state
  void clear();
  // Add another CameraBundle object to those from which structure will be computed
  void push(std::shared_ptr<const CameraBundle> bundle);
  // Computes a Point from all CameraBundle objects that have been pushed
  Point computeStructure();
  // Returns a copy of the stored Point object
  Point point() { return point_; }
  
private:
  Point point_;
  // Vector of pointers to CameraBundle objects all corresponding to the same
  // 3d feature
  std::vector<std::shared_ptr<const CameraBundle>> bundleVec_;
};
#endif
