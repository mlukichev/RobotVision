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