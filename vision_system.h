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

namespace robot_vision {

class VisionSystemCore {
 public:
  VisionSystemCore(double max_cluster_diameter, int estimated_positions): max_cluster_diameter_{max_cluster_diameter}, estimated_positions_{estimated_positions} {}
  absl::Status ReportCameraPosition(const CameraPosition& camera_position);
  void ClearCameraPosition();
  std::optional<Transformation> GetRobotPosition();
  std::optional<std::pair<cv::Mat, cv::Mat>> GetCameraById(int id);
  std::vector<int> GetKeys();
  void SetCurrentPosition(const Transformation& position);
  Transformation GetCurrentPosition();

 private:
  absl::Mutex mu_;
  // camera id -> tag id -> matrix
  std::unordered_map<int, std::unordered_map<int, std::pair<Transformation, Transformation>>> camera_in_tag_coords_;
  Tags tags_;
  CameraPositions camera_positions_;
  Cameras cameras_;
  Transformation position_;

  double max_cluster_diameter_;
  int estimated_positions_;
};

}  // namespace robot_vision

#endif