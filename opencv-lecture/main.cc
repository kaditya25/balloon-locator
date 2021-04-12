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
  namedWindow("Image", WINDOW_AUTOSIZE);
  while(1) {
    // Frame will store the next image frame from the video; framep will store
    // the processed version of this frame.
    Mat frame, framep;
    // Grab next frame from camera
    bool readSuccess = camera.read(frame);
    // Break out of the while loop if no more frames to read
    if(!readSuccess) {
      break;
    }
    // Blur the image to reduce small-scale noise
    GaussianBlur(frame, framep, Size(11,11), 0, 0);
    // Convert the image from the original BGR color space to HSV, which is
    // easier to segment based on color: color is encoded only in H.
    cvtColor(framep, framep, COLOR_BGR2HSV);
    // Filter only red: H goes from 0 to 180 and is circular, with the usual
    // 0-to-360 circle mapped to 0-to-180.  Red sits between -10 and 10, or in
    // positive units (which inRange expects), between 170 and 10.  Because
    // red straddles the 0 boundary, we apply two maskings and bitwise 'or'
    // the results, as shown below.  S (saturation, a measure of color purity)
    // and V (value, a measure of intensity) go from 0 to 255.
    Scalar colorLower_l(0,120,100), colorLower_h(10,255,255);
    Scalar colorUpper_l(170,120,100), colorUpper_h(180,255,255);
    Mat mLower, mUpper;
    inRange(framep, colorLower_l, colorLower_h, mLower);
    inRange(framep, colorUpper_l, colorUpper_h, mUpper);
    framep = mLower | mUpper;
    // Erode image to eliminate stray wisps of red
    constexpr int iterations = 5;
    erode(framep, framep, Mat(), Point(-1,-1), iterations);
    // Dilate image to restore red square to original size
    dilate(framep, framep, Mat(), Point(-1,-1), iterations);
    // Find contours
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    findContours(framep, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    // Loop through the contours.  Bound each contour by both a bounding
    // (rotated) retangle and a minimum enclosing circle.  Draw the contour on
    // the original image.  If the bounding rectangle is square enough (aspect
    // ratio near 1), and if the enclosing circle is large enough, then also
    // draw the minimum enclosing circle.
    RNG rng(12345);
    Point2f center;
    float radius;
    constexpr float maxAspectRatio = 1.5;
    constexpr float minRadius = 35;
    constexpr int minPointsFor_fitEllipse = 5;
    for (size_t ii = 0; ii < contours.size(); ii++) {
      const Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256),
                                  rng.uniform(0, 256));
      minEnclosingCircle(contours[ii], center, radius);
      float aspectRatio = maxAspectRatio;
      if (contours[ii].size() >= minPointsFor_fitEllipse) {
        RotatedRect boundingRectangle = fitEllipse(contours[ii]);
        const Size2f rectSize = boundingRectangle.size;
        aspectRatio = static_cast<float>(std::max(rectSize.width, rectSize.height))/
          std::min(rectSize.width, rectSize.height);
      }
      std::cout << "aspectRatio: "  << aspectRatio << ", radius: "
                << radius << std::endl;
      drawContours(frame, contours, ii, color, 2, LINE_8, hierarchy, 0);
      if (aspectRatio < maxAspectRatio && radius > minRadius) {
        circle(frame, center, static_cast<int>(radius), color, 2);
      }
    }
    // View the image
    imshow("Image", frame);
    int keycode = waitKey(0);
    // Quit on 'q'
    if (keycode == 'q') {
      break;
    }
  }

  return EXIT_SUCCESS;
}



