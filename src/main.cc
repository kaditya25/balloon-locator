// Author: YOUR NAME GOES HERE

#include <cstdlib>
#include <string>
#include <iostream>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

int main(int argc, char** argv) {

  // Constructing an Eigen::Vector
  Eigen::VectorXd vector;
  vector.resize(2,1);
  vector(0) = 5;
  vector(1) = 10;

  // Displaying an image
  cv::Mat image = cv::imread(std::string(argv[1]));

  if(!image.data) {
    std::cout <<  "Could not open or find the image" << std::endl;
    return EXIT_FAILURE;
  }

  cv::namedWindow( "Display window", cv::WINDOW_NORMAL);
  cv::resizeWindow("Display window", 600, 600);
  cv::imshow( "Display window", image);
  cv::waitKey(0);



  return EXIT_SUCCESS;
}
