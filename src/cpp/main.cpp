#include <iostream>
#include <opencv2/opencv.hpp>

#ifdef APRILTAG_FOUND
#include <apriltag/apriltag.h>
#endif

int main() {
  // Test OpenCV: create a blank image and print its size
  cv::Mat image = cv::Mat::zeros(cv::Size(320, 240), CV_8UC3);
  std::cout << "OpenCV image created. Size: " << image.cols << "x" << image.rows
            << std::endl;

#ifdef APRILTAG_FOUND
  // Test AprilTag: create and destroy a detector
  apriltag_family_t* tf = tag36h11_create();
  apriltag_detector_t* td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);
  std::cout << "AprilTag detector created with tag36h11." << std::endl;

  // Cleanup
  apriltag_detector_destroy(td);
  tag36h11_destroy(tf);
#else
  std::cout << "AprilTag not found or not enabled." << std::endl;
#endif

  return 0;
}
