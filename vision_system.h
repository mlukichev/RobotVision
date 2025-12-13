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

namespace robot_vision {

class VisionSystemCore {
 public:
  VisionSystemCore(double max_cluster_diameter, int estimated_positions): max_cluster_diameter_{max_cluster_diameter}, estimated_positions_{estimated_positions} {}
  absl::Status ReportCameraPosition(const CameraPosition& camera_position);
  std::optional<Transformation> GetRobotPosition();
  std::optional<std::pair<cv::Mat, cv::Mat>> GetCameraById(int id);
  std::vector<int> GetKeys();

 private:
  absl::Mutex mu_;
  // camera id -> tag id -> matrix
  std::unordered_map<int, std::unordered_map<int, Transformation>> camera_in_tag_coords_;
  Tags tags_;
  CameraPositions camera_positions_;
  Cameras cameras_;

  double max_cluster_diameter_;
  int estimated_positions_;
};

}  // namespace robot_vision

#endif