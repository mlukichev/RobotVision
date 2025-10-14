#ifndef TAG_DETECTOR_H
#define TAG_DETECTOR_H

#include <vector>

#include "tags.h"

namespace robot_vision {

struct TagPoints {
  TagId id;
  std::vector<cv::Point2d> points;
};

class TagDetector {
 public:
  virtual ~TagDetector() {}
  virtual std::vector<TagPoints> Detect(const cv::Mat& frame) = 0;
};

}  // namespace robot_vision
#endif