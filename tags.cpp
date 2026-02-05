#include "tags.h"

#include <opencv2/opencv.hpp>
#include <optional>
#include <map>

#include "absl/log/check.h"

namespace robot_vision {

Tags::Tags() {
  tag_to_world_[0] = Transformation((cv::Mat_<double>(4, 4) << 
    0, -1, 0, 0,
    0, 0, 1, 0,
    -1, 0, 0, 0,
    0, 0, 0, 1
  ));
  tag_to_world_[1] = Transformation((cv::Mat_<double>(4, 4) << 
    -1, 0, 0, 268.2875,
    0, 0, 1, 1030.2875,
    0, 1, 0, 0,
    0, 0, 0, 1
  ));
}

bool Tags::TagExists(TagId tag) const {
  return tag_to_world_.find(tag) != tag_to_world_.end();
}

std::optional<std::reference_wrapper<const Transformation>> Tags::GetTagToWorld(TagId tag) const {
  auto it = tag_to_world_.find(tag);
  // CHECK(it != tag_to_world_.end()) << "Tag " << tag << " doesn't exist";
  if (it == tag_to_world_.end()) {
    return std::nullopt;
  }
  return it->second;
}

}