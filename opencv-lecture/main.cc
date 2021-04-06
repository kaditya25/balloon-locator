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
    // Break out of while loop if no more frames to read
    if(!read_success) {
      break;
    }
    // Blur the image to reduce small-scale noise
    GaussianBlur(frame, framep, Size(11,11), 0, 0);
    // Convert the image from the original BGR color space to HSV, which is
    // easier to segment based on color: color is encoded only in H.
    cvtColor(framep, framep, COLOR_BGR2HSV);
    // Filter only red: H goes from 0 to 180 and is circular, with the usual
    // 0-to-360 circle mapped to 0-to-180.  Red sits between -10 and 10, or in
    // positive units (which inRange expects), between 170 and 10.  S
    // (saturation, a measure of color purity) and V (value, a measure of
    // intensity) go from 0 to 255.  The code below only gets half of the red
    // interval, but this seems to be adequate in this case.
    Scalar redLower(170,100,100);
    Scalar redUpper(180,255,255);
    inRange(framep, redLower, redUpper, framep);
    // Erode image to eliminate stray wisps of red
    const int iterations = 5;
    erode(framep, framep, Mat(), Point(-1,-1), iterations);
    // Dilate image to restore red square to original size
    dilate(framep, framep, Mat(), Point(-1,-1), iterations);
    // Find contours
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    findContours(framep, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    // Loop through the contours.  Bound each contour by both a bounding
    // (rotated) rectangle and a minimum enclosing circle.  Draw the contour
    // on the original image. If the bounding rectangle indicates that the
    // contour is square enough, and if the enclosing circle indicates it's
    // large enough, then also draw the minimum enclosing circle.
    Mat drawing = Mat::zeros(framep.size(), CV_8UC3);
    RNG rng(12345);
    Point2f center;
    float radius;
    const float maxAspectRatio = 1.5;
    const float minRadius = 35;
    const int minPointsFor_fitEllipse = 5;
    for(size_t ii = 0; ii < contours.size(); ++ii) {
      Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256),
                            rng.uniform(0, 256));
      minEnclosingCircle(contours[ii], center, radius);
      float aspectRatio = maxAspectRatio;
      if(contours[ii].size() >= minPointsFor_fitEllipse) {
        RotatedRect boundingRectangle = fitEllipse(contours[ii]);
        const Size2f rectSize = boundingRectangle.size;
        aspectRatio = static_cast<float>(std::max(rectSize.width,rectSize.height))/
          std::min(rectSize.width,rectSize.height);
      }
      std::cout << aspectRatio << " " << radius << std::endl;
      drawContours(frame, contours, ii, color, 2, LINE_8, hierarchy, 0);
      if(aspectRatio < maxAspectRatio && radius > minRadius) {
        circle(frame, center, (int)radius, color, 2);
      }
    }*/
    imshow("Image", framep);
    int keycode = waitKey(0);
    // Quit on 'q'
    if(keycode == 'q') break;
  }
  
  return EXIT_SUCCESS;
}



