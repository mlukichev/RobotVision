#ifndef APRIL_TAG_DETECTOR
#define APRIL_TAG_DETECTOR

#include "tag_detector.h"
#include "apriltag.h"
#include "tag36h11.h"

namespace robot_vision {

class ApriltagDetector: public TagDetector {
 public:
  ApriltagDetector() = default;
  ApriltagDetector(double, int, int, bool, bool);
  ~ApriltagDetector();

  std::vector<TagPoints> Detect(const cv::Mat& frame) override;

 private:
  apriltag_detector_t* td_;
  apriltag_family_t *tf_;
};

}  // namespace robot_vision

#endif