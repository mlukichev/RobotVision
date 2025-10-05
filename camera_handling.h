#ifndef CAMERA_HANDLING_H
#define CAMERA_HANDLING_H

#include <opencv2/opencv.hpp>
#include <optional>
#include "camera.h"
#include "apriltag.h"
#include "tags.h"

namespace robot_vision {

std::vector<std::pair<int, std::vector<cv::Point2i>>> GetImage(const Camera& cam, cv::VideoCapture cap, apriltag_detector_t* td);

std::optional<std::pair<cv::Mat, cv::Mat>> GetRobotPosition(const Camera& cam, const Tags& tags, int tag, const std::vector<cv::Point2i>& image, double apriltag_size);

}  // namespace robot_vision

#endif  // CAMERA_HANDLING_H