#include <optional>

#include "vision_system.h"
#include "absl/status/statusor.h"
#include "transformations.h"
#include "tags.h"
#include "camera_positions.h"
#include "camera_handling.h"

namespace robot_vision {

using absl::StatusOr;

absl::Status VisionSystemCore::ReportCameraPosition(const CameraPosition& camera_position) {
  std::unordered_map<int, Transformation> camera_tags; // tag -> matrix

  for (const CameraInTagCoords& e : camera_position.camera_in_tag_coords()) {
    if (e.mat_size() != 16) {
      return absl::InvalidArgumentError("Wrong matrix size");
    }
    camera_tags.emplace(e.tag_id(), (cv::Mat_<double>(4,4) << e.mat(0), e.mat(1), e.mat(2), e.mat(3),
      e.mat(4), e.mat(5), e.mat(6), e.mat(7),
      e.mat(8), e.mat(9), e.mat(10), e.mat(11),
      e.mat(12), e.mat(13), e.mat(14), e.mat(15)
    ));
  }

  absl::MutexLock lock(&mu_);
  camera_in_tag_coords_[camera_position.camera_id()] = std::move(camera_tags);
  return absl::OkStatus();
}

std::optional<Transformation> CombineTransformations(const std::vector<Transformation>& mats, double dist) {
  if (mats.size() == 0) {
    return std::nullopt;
  }
  double sdist = dist*dist;
  std::vector<Transformation> a;
  std::vector<Transformation> b;
  a.reserve(mats.size()-1);
  b.reserve(mats.size()-1);
  a.push_back(mats[0]);
  b.push_back(mats[1]);
  for (int i=2; i<mats.size(); ++i) {
    if(TransformationDifference(mats[i], mats[0], 0, 1) < sdist) {
      a.push_back(mats[i]);
    }
    if(TransformationDifference(mats[i], mats[1], 0, 1) < sdist) {
      b.push_back(mats[i]);
    }
  }
  if (a.size() >= b.size()) {
    return TransformationAverage(a);
  } else {
    return TransformationAverage(b);
  }
}

std::optional<Transformation> VisionSystemCore::GetRobotPosition() {
  std::vector<std::pair<int, std::pair<int, Transformation>>> mats;
  mats.reserve(estimated_positions_);
  {
    absl::MutexLock{&mu_};
    for (auto [cam_id, m] : camera_in_tag_coords_) {
      for (auto [tag_id, camera_in_tag] : m) {
        mats.push_back(std::pair(cam_id, std::pair(tag_id, camera_in_tag)));
      }
    }
  }
  std::vector<Transformation> sols;
  sols.reserve(mats.size()+1);
  for (int i=0; i<mats.size(); ++i) {
    sols.push_back(GetRobotInWorldCoords(tags_, mats[i].second.first, camera_positions_, mats[i].first, mats[i].second.second));
  }
  if (mats.empty()) {
    return std::nullopt;
  }
  return CombineTransformations(sols, max_cluster_diameter_);
}

}  // namespace robot_vision