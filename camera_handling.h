#ifndef CAMERA_HANDLING_H
#define CAMERA_HANDLING_H

#include <opencv2/opencv.hpp>
#include <optional>
#include "camera.h"
#include "tags.h"
#include "camera_positions.h"

namespace robot_vision {

std::optional<std::pair<Transformation, Transformation>> GetTagInCamCoords(const Camera& cam, const std::vector<cv::Point2d>& image, double apriltag_size);

std::optional<std::pair<Transformation, Transformation>> GetTagInCamCoords(const Camera& cam, const std::vector<cv::Point2d>& image, double apriltag_size);

std::optional<std::pair<Transformation, Transformation>> GetCamInWorldCoords(const Camera& cam, const Tags& tags, TagId tag, const std::vector<cv::Point2d>& image, double apriltag_size);

Transformation GetCamInWorldCoords(const Tags& tags, TagId tag, const Transformation& camera_in_tag);

std::optional<std::pair<Transformation, Transformation>> GetRobotInWorldCoords(const Camera& cam, const Tags& tags, TagId tag, const CameraPositions& cams, CameraId cam_id, const std::vector<cv::Point2d>& image, double apriltag_size);

Transformation GetRobotInWorldCoords(const Tags& tags, TagId tag, const CameraPositions& cams, CameraId cam_id, const Transformation& camera_in_tag);

}  // namespace robot_vision

#endif  // CAMERA_HANDLING_H