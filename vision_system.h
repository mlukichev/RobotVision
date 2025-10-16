#ifndef VISION_SYSTEM_H
#define VISION_SYSTEM_H

#include "opencv2/opencv.hpp"
#include "absl/container/flat_hash_map.h"
#include "absl/synchronization/mutex.h"
#include "absl/status/status.h"
#include "data_handling.pb.h"

namespace robot_vision {

class VisionSystemCore {
 public:
  absl::Status ReportCameraPosition(const CameraPosition& camera_position);
 private:
  absl::Mutex camera_in_tag_coords_mutex_;
  // camera id -> tag id -> matrix
  absl::flat_hash_map<int, absl::flat_hash_map<int, cv::Mat>> camera_in_tag_coords_;
};

}  // namespace robot_vision

#endif