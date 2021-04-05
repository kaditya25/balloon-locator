#include <cstdlib>
#include <cstdio>
#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "structurecomputer.h"
#include "balloonfinder.h"

int main(int argc, char** argv) {

  char frameNumber[30];

  std::vector<std::string> frameVec = {
    "/blue_balloon/frame00093.jpg",
    "/blue_balloon/frame00098.jpg",
    "/red_balloon/frame00107.jpg",
    "/red_balloon/frame00114.jpg"};

  // Initialize random number generator
  std::srand(314159);
  BalloonFinder balloonFinder;
  StructureComputer structureComputerBlue, structureComputerRed;
  // Draw in and process each image
  for(auto frame : frameVec) {
    std::string filename = std::string(argv[1]) + frame;
    cv::Mat image = cv::imread(filename);
    if(!image.data) {
      std::cout << "Usage: ./locateBalloon [path to balloons directory]" << std::endl;
      return EXIT_FAILURE;
    }
    // Generate dummy RCI and rc. We use SVD of a random 3x3 matrix to get a
    // dummy orthonormal matrix RCI
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(Eigen::Matrix3d::Random(),
                                          Eigen::ComputeFullU);
    Eigen::Matrix3d RCI = svd.matrixU();
    Eigen::Vector3d rc = 10*Eigen::Vector3d::Random();
    // Send image to balloonFinder
    std::vector<std::shared_ptr<const CameraBundle>> bundles;
    std::vector<BalloonFinder::BalloonColor> colors;
    balloonFinder.findBalloons(&image, RCI, rc, &bundles, &colors);
    for(size_t ii = 0; ii < bundles.size(); ii++) {
      switch(colors[ii]) {
      case BalloonFinder::RED:
        structureComputerRed.push(bundles[ii]);
        break;
      case BalloonFinder::BLUE:
        structureComputerBlue.push(bundles[ii]);
        break;
      default:
        break;
      }
    }
  }      
  
  // Estimate 3D location of feature.  The try and catch blocks are for
  // exception handling -- they handle any errors that might be thrown during
  // a call to computeStructure().  You can use this to cover any portion of
  // your code in main.cc.
  try {
    Point pBlue = structureComputerBlue.computeStructure();
    Point pRed = structureComputerRed.computeStructure();
    std::cout << "Blue balloon 3D location: \n" << pBlue.rXIHat << std::endl;
    std::cout << "Red balloon 3D location: \n" << pRed.rXIHat << std::endl;
  }
  catch(std::exception& e) {
    std::cout << "Error: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  catch(...) {
    std::cout << "Unhandled error" << std::endl;
    return EXIT_FAILURE;
  }
 
  return EXIT_SUCCESS;
}



