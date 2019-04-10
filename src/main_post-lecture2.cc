#include <cstdlib>
#include <string>
#include <iostream>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "structurecomputer.h"

int main(int argc, char** argv) {

  // Construct and manipulate Eigen objects
  Eigen::VectorXd b, x;
  Eigen::MatrixXd A;
  b.resize(3,1);
  b(0) = 4;
  b(1) = 10;
  A.resize(3,3);
  A << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  // Solve an equation of the form b = A*x for x
  x = A.colPivHouseholderQr().solve(b);
  std::cout << "The solution is \n" << x << std::endl;

  // Create an instance of a StructureComputer object
  StructureComputer structureComputer;
  // Create shared pointers to two CameraBundle objects.  The make_shared
  // function creates the objects and returns a shared pointer to each. 
  std::shared_ptr<CameraBundle> cb1 = std::make_shared<CameraBundle>();
  std::shared_ptr<CameraBundle> cb2 = std::make_shared<CameraBundle>();
  // Fill cb1's and cb2's data members with example contents.  
  cb1->rx << 23, 56;
  cb1->RCI.fill(3);
  cb1->rc(0) = 1;  cb1->rc(1) = 0.4;
  // Clear out structureComputer to put it in a known initial state.
  structureComputer.clear();
  // Push cb1 and cb2 to structureComputer
  structureComputer.push(cb1);
  structureComputer.push(cb2);
  // Estimate 3D location of feature
  Point p1 = structureComputer.computeStructure();  

  // Display an image
  cv::Mat image = cv::imread(std::string(argv[1]));
  if(!image.data) {
    std::cout <<  "Could not open or find the image" << std::endl;
    return EXIT_FAILURE;
  }
  cv::namedWindow("Display window", cv::WINDOW_NORMAL);
  cv::resizeWindow("Display window", 600, 600);
  cv::imshow("Display window", image);
  cv::waitKey(0);
  return EXIT_SUCCESS;
}
