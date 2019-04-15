#include <cstdlib>
#include <string>
#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {

  using namespace cv;

  std::string filename = "./input.m4v";
  VideoCapture camera(filename);
  if(!camera.isOpened()) {
    std::cout << "Failed to open " << filename << std::endl; 
    return EXIT_FAILURE;
  }


  return EXIT_SUCCESS;
}



