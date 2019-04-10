#include <cstdlib>
#include <string>
#include <iostream>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

int main(int argc, char** argv) {

  using namespace Eigen;

  // Construct and manipulate Eigen objects
  VectorXd b, x;
  MatrixXd A;
  b.resize(3,1);
  b(0) = 4;
  b(1) = 10;
  A.resize(3,3);
  A << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  x = A.colPivHouseholderQr().solve(b);
  std::cout << "The solution is \n" << x << std::endl;

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
