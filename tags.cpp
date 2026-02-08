#include "tags.h"

#include <opencv2/opencv.hpp>
#include <optional>
#include <map>
#include <iostream>

#include "absl/log/check.h"

namespace robot_vision {

Tags::Tags() {
  int tag_num;
  std::cin >> tag_num;
  for (int i=0; i<tag_num; ++i) {
    int tag;
    std::cin >> tag;
    cv::Mat pos(4, 4, CV_64F);
    for (int j=0; j<4; ++j) {
      for (int k=0; k<4; ++k) {
        double val;
        std::cin >> val;
        pos.at<double>(j, k) = val;
      }
    }

    tag_to_world_[tag] = Transformation(pos);
  }
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