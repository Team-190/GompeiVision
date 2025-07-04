#include <apriltag/apriltag.h>
#include <ntcore/networktables/NetworkTableInstance.h>
#include <openpnp-capture.h>

#include <opencv2/opencv.hpp>
int main() {
  nt::NetworkTableInstance ntinst = nt::NetworkTableInstance::GetDefault();
  apriltag_detection_t* detection = nullptr;
  auto image = cv::Mat();
  CapResult result = CapResult();
  std::cout << "Hello Gompei!" << std::endl;
  return 0;
}
