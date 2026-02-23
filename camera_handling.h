#ifndef CAMERA_HANDLING_H
#define CAMERA_HANDLING_H

#include <opencv2/opencv.hpp>
#include <optional>
#include "camera.h"
#include "tags.h"
#include "camera_positions.h"

namespace robot_vision {

struct TransformationWithError {
  Transformation pos;
  double err;
};

struct AmbiguousTransformation {
  TransformationWithError best;
  TransformationWithError worse;
};



std::optional<AmbiguousTransformation> GetTagToCam(
  const Camera& cam, const std::vector<cv::Point2d>& image, double apriltag_size);

// std::optional<std::pair<Transformation, Transformation>> GetCamToWorld(
//   const Camera& cam, Tags& tags, TagId tag, const std::vector<cv::Point2d>& image, double apriltag_size);

std::optional<Transformation> GetCamToWorld(Tags& tags, TagId tag, const Transformation& camera_to_tag);

// std::optional<std::pair<Transformation, Transformation>> GetRobotInWorldCoords(const Camera& cam, const Tags& tags, TagId tag, const CameraPositions& cams, CameraId cam_id, const std::vector<cv::Point2d>& image, double apriltag_size);

std::optional<Transformation> GetRobotToWorld
(
  Tags& tags, TagId tag, const CameraPositions& cams, CameraId cam_id, const Transformation& camera_to_tag);

}  // namespace robot_vision

#endif  // CAMERA_HANDLING_H