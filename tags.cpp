#include "tags.h"

#include <opencv2/opencv.hpp>
#include <optional>
#include <map>
#include <iostream>
#include <fstream>

#include "absl/log/check.h"

namespace robot_vision {

Tags::Tags(Tags&& other) {
  absl::MutexLock lock_this{&mu_};
  absl::MutexLock lock_other{&other.mu_};
  tag_to_world_ = std::move(other.tag_to_world_);
}

bool Tags::TagExists(TagId tag) const {
  return tag_to_world_.find(tag) != tag_to_world_.end();
}

Tags& Tags::operator=(Tags&& other) {
  absl::MutexLock lock_this{&mu_};
  absl::MutexLock lock_other{&other.mu_};
  tag_to_world_ = std::move(other.tag_to_world_);
  return *this;
}

std::optional<Transformation> Tags::GetTagToWorld(TagId tag) {
  absl::MutexLock lock{&mu_};
  auto it = tag_to_world_.find(tag);
  // CHECK(it != tag_to_world_.end()) << "Tag " << tag << " doesn't exist";
  if (it == tag_to_world_.end()) {
    return std::nullopt;
  }
  return it->second;
}

void Tags::emplace(TagId tag, Transformation pos) {
  absl::MutexLock lock{&mu_};
  tag_to_world_.emplace(tag, pos);
}

Tags ReadTags(const std::string& filename) {
  Tags tags;
  std::ifstream file(filename);
  CHECK(file) << "Could not open file " << filename;

  int tag_num;
  CHECK(file >> tag_num) << "Error reading from " << filename;
  for (int i=0; i<tag_num; ++i) {
    int tag;
    CHECK(file >> tag) << "Error reading from " << filename;
    cv::Mat pos(4, 4, CV_64F);
    for (int j=0; j<4; ++j) {
      for (int k=0; k<4; ++k) {
        double val;
        CHECK(file >> val) << "Error reading from " << filename;
        pos.at<double>(j, k) = val;
      }
    }

    tags.emplace(tag, Transformation(pos));
  }

  return std::move(tags);
}

}