#include <cassert>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include "balloonfinder.h"
#include "navtoolbox.h"

BalloonFinder::BalloonFinder(bool debuggingEnabled, bool calibrationEnabled,
                const Eigen::Vector3d& blueTrue_I,
                const Eigen::Vector3d& redTrue_I) {
  debuggingEnabled_ = debuggingEnabled;
  calibrationEnabled_ = calibrationEnabled;
  blueTrue_I_ = blueTrue_I;
  redTrue_I_ = redTrue_I;
  V_.resize(3,0);
  W_.resize(3,0);
}

// Returns true if the input contour touches the edge of the input image;
// otherwise returns false.
//
bool touchesEdge(const cv::Mat& image,
                 const std::vector<cv::Point>& contour) {

  const size_t borderWidth = static_cast<size_t>(0.01*image.rows);

  for(const auto& pt : contour) {
    if(pt.x <= borderWidth || pt.x >= (image.cols - borderWidth) ||
       pt.y <= borderWidth || pt.y >= (image.rows - borderWidth))
      return true;
  }
  return false;
}

Eigen::Vector3d BalloonFinder::eCB_calibrated() const {

  using namespace Eigen;
  SensorParams sp;
  const size_t N = V_.cols();
  if(N < 2 || !calibrationEnabled_) {
    return MatrixXd::Identity(3,3);
  }
  MatrixXd aVec = MatrixXd::Ones(N,1);
  Matrix3d dRCB = navtbx::wahbaSolver(aVec,W_,V_);
  Matrix3d RCB = navtbx::euler2dc(sp.eCB());
  std::cout << "Calibration correction deCB (deg): "
            << navtbx::dc2euler(dRCB).transpose()*180/PI << std::endl;
  return navtbx::dc2euler(dRCB*RCB);
}

bool BalloonFinder::
findBalloonsOfSpecifiedColor(const cv::Mat* image,
                             const Eigen::Matrix3d RCI,
                             const Eigen::Vector3d rc_I,
                             const BalloonFinder::BalloonColor color,
                             std::vector<Eigen::Vector2d>* rxVec) {
  using namespace cv;
  bool returnValue = false;
  rxVec->clear();  
  namedWindow("Display", WINDOW_NORMAL);
  resizeWindow("Display", 1000, 1000);
  const size_t nCols_m1 = image->cols - 1;
  const size_t nRows_m1 = image->rows - 1;
  // Clone the original image (for debugging purposes)
  Mat original = image->clone();
  // Blur the image to reduce small-scale noise
  Mat framep;
  GaussianBlur(*image, framep, Size(21,21), 0, 0);

  // ****************************************
  // Implement the rest of the function here.
  // ****************************************
  
  if(debuggingEnabled_) {
    // Plot the back-projection of true balloon 3d location on the original image
    Eigen::Vector2d xc_pixels;
    Scalar trueProjectionColor;
    if(color == BalloonColor::BLUE) {
      xc_pixels = backProject(RCI,rc_I,blueTrue_I_);
      trueProjectionColor = Scalar(255,0,0);
    }
    else {
      xc_pixels = backProject(RCI,rc_I,redTrue_I_);
      trueProjectionColor = Scalar(0,0,255);
    }
    Point2f center;
    center.x = nCols_m1 - xc_pixels(0);
    center.y = nRows_m1 - xc_pixels(1);
    circle(original, center, 20, trueProjectionColor, FILLED);
    imshow("Display", original);
    waitKey(0);
  }
  return returnValue;
}

void BalloonFinder::
findBalloons(const cv::Mat* image,
             const Eigen::Matrix3d RCI,
             const Eigen::Vector3d rc_I,
             std::vector<std::shared_ptr<const CameraBundle>>* bundles,
             std::vector<BalloonColor>* colors) {

  // Crop image to 4k size.  This removes the bottom 16 rows of the image,
  // which are an artifact of the camera API.
  const cv::Rect croppedRegion(0,0,sensorParams_.imageWidthPixels(),
                               sensorParams_.imageHeightPixels());
  cv::Mat croppedImage = (*image)(croppedRegion);
  // Convert camera instrinsic matrix K and distortion parameters to OpenCV
  // format
  cv::Mat K, distortionCoeffs, undistortedImage;
  Eigen::Matrix3d Kpixels = sensorParams_.K()/sensorParams_.pixelSize();
  Kpixels(2,2) = 1;
  cv::eigen2cv(Kpixels, K);
  cv::eigen2cv(sensorParams_.distortionCoeffs(),distortionCoeffs);
  // Undistort image
  cv::undistort(croppedImage, undistortedImage, K, distortionCoeffs);
    
  // Find balloons of specified color
  std::vector<BalloonColor> candidateColors = {BalloonColor::RED, BalloonColor::BLUE};
  for(auto color : candidateColors) {
    std::vector<Eigen::Vector2d> rxVec;
    if(findBalloonsOfSpecifiedColor(&undistortedImage,RCI,rc_I,color,&rxVec)) {
      for(const auto& rx : rxVec) {
        std::shared_ptr<CameraBundle> cb = std::make_shared<CameraBundle>();
        cb->RCI = RCI;
        cb->rc_I = rc_I;
        cb->rx = rx;
        bundles->push_back(cb);
        colors->push_back(color);
      }
    }
  }
}
