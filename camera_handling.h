#ifndef CAMERA_HANDLING_H
#define CAMERA_HANDLING_H

#include <opencv2/opencv.hpp>
#include <optional>
#include "camera.h"
#include "tag36h11.h"

namespace robot_vision {

// Finds the two transformatoin matricies to convert from tag coordinate space to camera coordinate space
std::optional<std::pair<cv::Mat, cv::Mat>> TransformTagToCam(const robot_vision::Camera& cam, int tag, const std::vector<cv::Point2i>& image);

// Gets the 2 possible transformation matricies from World coordinate space to Robot coordinate space
std::optional<std::pair<cv::Mat, cv::Mat>> GetRobotTransform(const robot_vision::Camera& cam, int tag, const std::vector<cv::Point2i>& image);

}  // namespace robot_vision

#endif  // CAMERA_HANDLING_H