#include "vision_system.h"
#include "absl/status/statusor.h"

namespace robot_vision {

using absl::StatusOr;
using google::protobuf::RepeatedField; 

namespace {

absl::StatusOr<cv::Mat> FromProto(const RepeatedField<double>& mat) {
  if (mat.size() != 16) {
    return absl::InvalidArgumentError("Transformation matrix must be 4x4");
  }

  // TODO convert mat to cv::Mat
}

}  // namespace

absl::Status VisionSystemCore::ReportCameraPosition(const CameraPosition& camera_position) {
  absl::WriterMutexLock lock(&camera_in_tag_coords_mutex_);
  absl::flat_hash_map<int, cv::Mat> camera_tags = camera_in_tag_coords_[camera_position.camera_id()];
  for (const CameraInTagCoords& camera_in_tag_coords : camera_position.camera_in_tag_coords()) {
    absl::StatusOr<cv::Mat> coords_or =  FromProto(camera_in_tag_coords.mat());
    if (!coords_or.ok()) {
      return coords_or.status();
    }
    camera_tags[camera_in_tag_coords.tag_id()] = *coords_or;
  }
}

}  // namespace robot_vision