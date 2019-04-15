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

  // Processed image frame
  Mat framep;
  namedWindow("Processed Image", WINDOW_AUTOSIZE);
  while(1) {
    Mat frame;
    // Grab next frame from camera
    camera >> frame;
    // Blur the image to reduce small-scale noise
    GaussianBlur(frame, framep, Size(11,11), 0, 0);
    // Convert the image from original BGR color space to the HSV: easier to
    // segment based on color, which is encoded only in H
    cvtColor(framep, framep, COLOR_BGR2HSV);
    // Filter only red
    Scalar redLower(150,100,100);
    Scalar redUpper(200,255,255);
    inRange(framep, redLower, redUpper, framep);
    // Erode image to eliminate stray wisps of red
    const int iterations = 5;
    erode(framep, framep, Mat(), Point(-1,-1), iterations);
    // Dilate image to restore main red square to original size
    dilate(framep, framep, Mat(), Point(-1,-1), iterations);
    // Find contours
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    findContours(framep, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    Mat drawing = Mat::zeros(framep.size(), CV_8UC3);
    RNG rng(12345);
    for(size_t ii = 0; ii < contours.size(); ++ii) {
      Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
      drawContours(drawing, contours, ii, color, 2, LINE_8, hierarchy, 0);
    }
    
    imshow("Processed Image", drawing);    
    waitKey(0);
  }


  return EXIT_SUCCESS;
}



