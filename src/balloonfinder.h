#ifndef __BALLOON_FINDER_H
#define __BALLOON_FINDER_H

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <cstdlib>
#include <memory>

#include "sensorparams.h"
#include "structurecomputer.h"

class BalloonFinder {
  enum BalloonColor {
    RED,
    BLUE
  };
public:
  // Constructor: gets called when an instance of BalloonFinder is created
  BalloonFinder() {}
  // Destructor: gets called when an instance of BalloonFinder is destroyed
  ~BalloonFinder() {}
  // Takes in a pointer to an image and outputs a pointer to an N-by-1 vector
  // of CameraBundle objects, one for each balloon found.  N = bundles->size()
  // indicates the number of balloons found.  Also outputs an N-by-1 vector of
  // colors corresponding to the balloons found.  Thus, if two balloons were
  // found in an image, N = bundles->size() = colors->size() = 2.  
  void findBalloons(const cv::Mat* image,
                    std::vector<std::shared_ptr<const CameraBundle>>* bundles,
                    std::vector<BalloonColor>* colors);

private:
  SensorParams sensorParams_;
};
#endif
