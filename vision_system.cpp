#include <optional>

#include "vision_system.h"
#include "absl/status/statusor.h"
#include "absl/log/log.h"
#include "absl/log/check.h"
#include "transformations.h"
#include "tags.h"
#include "camera_positions.h"
#include "camera_handling.h"
#include "camera.h"
#include "cameras.h"

namespace robot_vision {

using absl::StatusOr;

VisionSystemCore::VisionSystemCore(double max_cluster_diameter, int estimated_positions): max_cluster_diameter_{max_cluster_diameter}, estimated_positions_{estimated_positions} {
  cameras_ = ReadCameraCoefficients("cameras.txt");
  tags_ = ReadTags("tag_locations.txt");
}

absl::Status VisionSystemCore::ReportCameraPosition(const CameraPosition& camera_position) {
  std::unordered_map<int, std::pair<Transformation, Transformation>> camera_tags; // tag -> matrix

  for (const TagInCamCoords& e : camera_position.tag_in_cam_coords()) {
    if (e.mat1_size() != 16 || e.mat2_size() != 16) {
      return absl::InvalidArgumentError("Wrong matrix size");
    }
    camera_tags.emplace(e.tag_id(), std::pair((
      cv::Mat_<double>(4,4) <<
        e.mat1(0), e.mat1(1), e.mat1(2), e.mat1(3),
        e.mat1(4), e.mat1(5), e.mat1(6), e.mat1(7),
        e.mat1(8), e.mat1(9), e.mat1(10), e.mat1(11),
        e.mat1(12), e.mat1(13), e.mat1(14), e.mat1(15)
    ), (
      cv::Mat_<double>(4,4) << 
        e.mat2(0), e.mat2(1), e.mat2(2), e.mat2(3),
        e.mat2(4), e.mat2(5), e.mat2(6), e.mat2(7),
        e.mat2(8), e.mat2(9), e.mat2(10), e.mat2(11),
        e.mat2(12), e.mat2(13), e.mat2(14), e.mat2(15)
    )));
  }

  absl::MutexLock lock(&mu_);
  tag_to_cam_[camera_position.camera_id()] = std::move(camera_tags);
  return absl::OkStatus();
}

void VisionSystemCore::ClearCameraPosition() {
  absl::MutexLock lock(&mu_);
  tag_to_cam_.clear();
}

std::optional<Transformation> CombineTransformations(const std::vector<Transformation>& mats, double dist) {
  if (mats.size() == 0) {
    return std::nullopt;
  }
  CHECK(mats.size() >= 2) << "mats.size() = " << mats.size();
  double sdist = dist*dist;
  std::vector<Transformation> a;
  std::vector<Transformation> b;
  a.reserve(mats.size()-1);
  b.reserve(mats.size()-1);
  a.push_back(mats[0]);
  b.push_back(mats[1]);
  // LOG(INFO) << "mat 0: (" << mats[0].ToVec3d()[0] << ", " << mats[0].ToVec3d()[1] << ", " << mats[0].ToVec3d()[2] << ")";
  // LOG(INFO) << "mat 1: (" << mats[1].ToVec3d()[0] << ", " << mats[1].ToVec3d()[1] << ", " << mats[1].ToVec3d()[2] << ")";
  for (int i=2; i<mats.size(); ++i) {
    // LOG(INFO) << "mat " << i << ": (" << mats[i].ToVec3d()[0] << ", " << mats[i].ToVec3d()[1] << ", " << mats[i].ToVec3d()[2] << ")";
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
  // Vector: (camera_id, (tag_id, [tag->cam]))
  std::vector<std::pair<int, std::pair<int, Transformation>>> mats;
  mats.reserve(estimated_positions_);
  {
    absl::MutexLock{&mu_};
    // tag_in_cam_coords_:
    // Map: camera_id -> (tag_id -> (First Solution, Second Solution))
    for (auto [cam_id, m] : tag_to_cam_) {
      for (auto [tag_id, tag_to_cam] : m) {
        cv::Vec3d vec1 = tag_to_cam.first.ToVec3d();
        cv::Vec3d vec2 = tag_to_cam.second.ToVec3d();
        // LOG(INFO) << "tag in cam " << tag_id << ": (" << vec1[0] << ", " << vec1[1] << ", " << vec1[2] << ")";
        // LOG(INFO) << "tag in cam " << tag_id << ": (" << vec2[0] << ", " << vec2[1] << ", " << vec2[2] << ")";
        // LOG(INFO) << "distance to " << tag_id << ": " << cv::norm(vec1);
        // LOG(INFO) << "distance to " << tag_id << ": " << cv::norm(vec2);
        mats.push_back(std::pair(cam_id, std::pair(tag_id, tag_to_cam.first)));
        mats.push_back(std::pair(cam_id, std::pair(tag_id, tag_to_cam.second)));
      }
    }
  }
  // Possible
  std::vector<Transformation> sols;
  sols.reserve(mats.size()+1);
  for (const auto& [cam_id, tag_transform] : mats) {
    const auto& [tag_id, tag_to_cam] = tag_transform;
    std::optional<Transformation> solution = GetRobotToWorld(
      tags_, tag_id, camera_positions_, cam_id, /*cam_to_tag=*/tag_to_cam.Inverse());
    if (solution.has_value()) {
      sols.push_back(std::move(*solution));
    } else {
      LOG(WARNING) << "No solution found for cam_id=" << cam_id << " tag_id=" << tag_id;
    }
  }
  if (mats.empty()) {
    return std::nullopt;
  }
  return CombineTransformations(sols, max_cluster_diameter_);
}

std::optional<std::pair<cv::Mat, cv::Mat>> VisionSystemCore::GetCameraById(int id) {
  absl::MutexLock lock{&mu_};
  if (!cameras_.CameraExists(id)) {
    return std::nullopt;
  }
  Camera cam = cameras_.GetCameraByID(id);
  return std::pair(cam.GetCamMat(), cam.GetDistCoef());
}

std::vector<int> VisionSystemCore::GetKeys() {
  absl::MutexLock lock{&mu_};
  return cameras_.GetKeys();
}

void VisionSystemCore::SetCurrentPosition(const Transformation& position) {
  absl::MutexLock lock{&mu_};
  position_ = position;
}

Transformation VisionSystemCore::GetCurrentPosition() {
  absl::MutexLock lock{&mu_};
  return position_;
}

}  // namespace robot_vision
