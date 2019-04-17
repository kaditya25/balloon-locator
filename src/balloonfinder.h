#ifndef __BALLOON_FINDER_H
#define __BALLOON_FINDER_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <cstdlib>
#include <memory>

#include "sensorparams.h"
#include "structurecomputer.h"

class BalloonFinder {
public:
  enum BalloonColor {
    RED,
    BLUE
  };
  // Constructor: gets called when an instance of BalloonFinder is created
  BalloonFinder() {}
  // Destructor: gets called when an instance of BalloonFinder is destroyed
  ~BalloonFinder() {}
  // Takes in a pointer to an image and outputs a pointer to an N-by-1 vector
  // of CameraBundle objects, one for each balloon found.  N = bundles->size()
  // indicates the number of balloons found.  Also outputs an N-by-1 vector of
  // colors corresponding to the balloons found.  Thus, if two balloons were
  // found in an image, N = bundles->size() = colors->size() = 2.  The input
  // RCI is the 3x3 attitude matrix relating the C and I frames at the instant
  // the input image was taken.  The input rc is the 3x1 position of the
  // camera center at the instant the image was taken, expressed in the I
  // frame in meters.  RCI and rc are input so that the corresponding values
  // in each output CameraBundle can be populated.
  void findBalloons(const cv::Mat* image,
                    const Eigen::Matrix3d RCI,
                    const Eigen::Vector3d rc,
                    std::vector<std::shared_ptr<const CameraBundle>>* bundles,
                    std::vector<BalloonColor>* colors);

private:
  SensorParams sensorParams_;
};
#endif
