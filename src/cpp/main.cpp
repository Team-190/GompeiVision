#include <apriltag/apriltag.h>
#include <ntcore/networktables/NetworkTableInstance.h>

#include <opencv2/opencv.hpp>
int main() {
  nt::NetworkTableInstance ntinst = nt::NetworkTableInstance::GetDefault();
  apriltag_detection_t* detection = nullptr;
  cv::Mat image = cv::Mat();
  return 0;
}
