#include "tags.h"

#include <opencv2/opencv.hpp>
#include <optional>
#include <map>

#include "absl/log/check.h"

namespace robot_vision {

Tags::Tags() {
  tags_[0] = (cv::Mat_<double>(4, 4) << 
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1
  );
  tags_[1] = (cv::Mat_<double>(4, 4) << 
    0.7071067811847432, 0, 0.7071067811883519, 88.9,
    0, 1, 0, 1158.875,
    -0.7071067811883519, 0, 0.7071067811847432, -45.72,
    0, 0, 0, 1
  );
}

bool Tags::TagExists(int tag) const {
  return tags_.find(tag) != tags_.end();
}

cv::Mat Tags::GetTagByID(int tag) const {
  auto it = tags_.find(tag);
  CHECK(it != tags_.end()) << "Tag " << tag << " doesn't exist";
  return it->second;
}

}