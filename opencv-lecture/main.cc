#include <cstdlib>
#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {

  using namespace cv;

  // Open video file via a VideoCapture object named camera
  std::string filename = "./input.m4v";
  VideoCapture camera(filename);
  // Check to ensure the video file opened properly
  if(!camera.isOpened()) {
    std::cout << "Failed to open " << filename << std::endl; 
    return EXIT_FAILURE;
  }

  // Process the video file and track the red square
  // Processed image frame
  Mat framep;
  namedWindow("Image", WINDOW_AUTOSIZE);
  while(1) {
    Mat frame;
    // Grab next frame from camera
    bool read_success = camera.read(frame);
    // Break out of the while loop if no more frames to read
    if(!read_success) {
      break;
    }
    // Blur the image to reduce small-scale noise
    GaussianBlur(frame, framep, Size(11,11), 0, 0);
    
    // Take a look at our image
    imshow("Image", framep);
    int keycode = waitKey(0);
    //
    // Quit on 'q'
    if(keycode == 'q') {
      break;
    }
  }

  return EXIT_SUCCESS;
}



