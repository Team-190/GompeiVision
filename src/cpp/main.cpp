#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>

#include <iostream>
#include <opencv2/opencv.hpp>

int main() {
  // Test OpenCV: create a blank image and print its size
  const cv::Mat image = cv::Mat::zeros(cv::Size(320, 240), CV_8UC3);
  std::cout << "OpenCV image created. Size: " << image.cols << "x" << image.rows
            << std::endl;

  apriltag_family_t* tf = tag36h11_create();
  apriltag_detector_t* td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);
  std::cout << "AprilTag detector created with tag36h11." << std::endl;

  // Cleanup
  apriltag_detector_destroy(td);
  tag36h11_destroy(tf);

  return 0;
}
