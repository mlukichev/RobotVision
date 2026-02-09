#ifndef APRIL_TAG_DETECTOR
#define APRIL_TAG_DETECTOR

#include "tag_detector.h"
#include "apriltag.h"
#include "tag36h11.h"

namespace robot_vision {

class ApriltagDetector: public TagDetector {
 public:
  ApriltagDetector(): ApriltagDetector(1.0, 0, 4, false, true) {}
  ApriltagDetector(double quad_decimate, int quad_sigma, int nthreads, bool debug, bool refine_edges);
  ~ApriltagDetector() override;

  std::vector<TagPoints> Detect(cv::Mat& frame, const Cameras& cams, int cam_id) override;

 private:
  apriltag_detector_t* td_;
  apriltag_family_t *tf_;
};

}  // namespace robot_vision

#endif