#ifndef CAMERA_HANDLING_H
#define CAMERA_HANDLING_H

#include <opencv2/opencv.hpp>
#include <optional>
#include "camera.h"
#include "apriltag.h"
#include "tags.h"

namespace robot_vision {

std::optional<std::pair<cv::Mat, cv::Mat>> GetCameraInTagCoords(const Camera& cam, const std::vector<cv::Point2d>& image, double apriltag_size);

// std::optional<std::pair<cv::Mat, cv::Mat>> RobotInTagCoords(const Camera& cam, const std::vector<cv::Point2d>& image, double apriltag_size);

// std::optional<std::pair<cv::Mat, cv::Mat>> RobotInWorldCoords(const Camera& cam, Tags tags, int tag, const std::vector<cv::Point2d>& image, double apriltag_size);

// std::optional<std::pair<cv::Mat, cv::Mat>> GetCameraInWorldCoords(const Camera& cam, Tags tags, int tag, const std::vector<cv::Point2d>& image, double apriltag_size);

std::optional<std::pair<cv::Mat, cv::Mat>> TransformTagToCam(const Camera& cam, const std::vector<cv::Point2d>& image, double apriltag_size);

}  // namespace robot_vision

#endif  // CAMERA_HANDLING_H