#ifndef VISION_SYSTEM_H
#define VISION_SYSTEM_H

#include <unordered_map>

#include "opencv2/opencv.hpp"
#include "absl/synchronization/mutex.h"
#include "absl/status/status.h"
#include "data_handling.pb.h"
#include "tags.h"
#include "camera_positions.h"
#include "cameras.h"
#include "transformations.h"
#include "camera_handling.h"

namespace robot_vision {

class VisionSystemCore {
 public:
  VisionSystemCore(double max_cluster_diameter, int estimated_positions,
    const std::string& camera_coefficients, const std::string& tag_locations, const std::string& camera_locations);
  absl::Status ReportCameraPosition(const CameraPosition& camera_position);
  void ClearCameraPosition();
  std::optional<Transformation> GetRobotPosition();
  std::optional<std::pair<cv::Mat, cv::Mat>> GetCameraById(int id);
  std::vector<int> GetKeys();
  void SetCurrentPosition(const Transformation& position);
  Transformation GetCurrentPosition();
  void SetTags(Tags&& tags);

 private:
  absl::Mutex mu_;
  // camera id -> tag id -> matrix
  std::unordered_map<int, std::unordered_map<int, AmbiguousTransformation>> tag_to_cam_;
  Tags tags_;
  CameraPositions camera_positions_;
  Cameras cameras_;
  Transformation position_;

  double max_cluster_diameter_;
  int estimated_positions_;
};

}  // namespace robot_vision

#endif