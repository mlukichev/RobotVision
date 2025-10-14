#include <iostream>
#include <utility>
#include <algorithm>
#include <optional>
#include <opencv2/opencv.hpp>

#include "camera.h"
#include "transformations.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "apriltag.h"
#include "tag36h11.h"
#include "tags.h"

namespace robot_vision {

// Finds Tranfomration from Tag coord sys to Cam coord sys
// Apriltag size is the length of one square
std::optional<std::pair<Transformation, Transformation>> TransformTagToCam(const Camera& cam, const std::vector<cv::Point2d>& image, double apriltag_size) {
  
  // half of apritag size
  double hs = apriltag_size;
  std::vector<cv::Point3d> object = {
    cv::Point3d(-hs, hs, 0),
    cv::Point3d(hs, hs, 0),
    cv::Point3d(hs, -hs, 0),
    cv::Point3d(-hs, -hs, 0)
  };

  std::vector<cv::Mat> tvec;
  std::vector<cv::Mat> rvec;

  if (cv::solvePnPGeneric(
    object,
    image,
    cam.cam_mat,
    cam.dist_coef,
    rvec,
    tvec,
    false,
    cv::SOLVEPNP_IPPE_SQUARE
  )) {

    cv::Mat rot1, rot2;
    cv::Rodrigues(rvec[0], rot1);
    cv::Rodrigues(rvec[1], rot2);
    
    return std::pair{
      Transformation(tvec[0], rot1), 
      Transformation(tvec[1], rot2)
    };
  }

  return std::nullopt;
}

// Returns 4D transformation: Camera -> Tag coord
std::optional<std::pair<Transformation, Transformation>> GetCameraInTagCoords(const Camera& cam, const std::vector<cv::Point2d>& image, double apriltag_size) {
  // Transofrmation: Tag -> Camera coord
  std::optional<std::pair<Transformation, Transformation>> tag_to_cam = TransformTagToCam(cam, image, apriltag_size);
  if (!tag_to_cam.has_value()) {
    return std::nullopt;
  }

  auto [tag_to_cam1, tag_to_cam2] = *tag_to_cam;
  return std::pair{
    tag_to_cam1.Inverse(), 
    tag_to_cam2.Inverse()
  };
}

// cv::Mat GetRobotInCameraCoords(const Camera& cam) {
//   Transformation inv = cam.pos.Inverse();
//   cv::Mat adjusted = (cv::Mat_<double>(4, 4) << 
//     -inv.at<double>(2, 0), -inv.at<double>(2, 1), -inv.at<double>(2, 2), -inv.at<double>(2, 3),
//     -inv.at<double>(0, 0), -inv.at<double>(0, 1), -inv.at<double>(0, 2), -inv.at<double>(0, 3),
//     inv.at<double>(1, 0), inv.at<double>(1, 1), inv.at<double>(1, 2), inv.at<double>(1, 3),
//     0, 0, 0, 1
//   );
//   return adjusted;
// }

// std::optional<std::pair<cv::Mat, cv::Mat>> RobotInTagCoords(const Camera& cam, const std::vector<cv::Point2d>& image, double apriltag_size) {
//   std::optional<std::pair<Transformation, Transformation>> cam_in_tag = GetCameraInTagCoords(cam, image, apriltag_size);
//   if (!cam_in_tag.has_value()) {
//     return std::nullopt;
//   }
//   cv::Mat rob_in_cam = GetRobotInCameraCoords(cam);
//   return std::optional(std::pair(rob_in_cam*cam_in_tag->first, rob_in_cam*cam_in_tag->second));
// }

// std::optional<std::pair<cv::Mat, cv::Mat>> RobotInWorldCoords(const Camera& cam, Tags tags, int tag, const std::vector<cv::Point2d>& image, double apriltag_size) {
//   std::optional<std::pair<cv::Mat, cv::Mat>> rob_in_wor = RobotInTagCoords(cam, image, apriltag_size);
//   if (!rob_in_wor.has_value()) {
//     return std::nullopt;
//   }
//   return std::optional(std::pair(rob_in_wor->first*tags.GetTagByID(tag), rob_in_wor->second*tags.GetTagByID(tag)));
// }

std::optional<std::pair<Transformation, Transformation>> GetCameraInWorldCoords(const Camera& cam, Tags tags, TagId tag, const std::vector<cv::Point2d>& image, double apriltag_size) {
  std::optional<std::pair<Transformation, Transformation>> camera_in_tag = GetCameraInTagCoords(cam, image, apriltag_size);
  if (!camera_in_tag.has_value()) {
    return std::nullopt;
  }
  return std::optional(
    std::pair{
      tags.GetTagByID(tag) * camera_in_tag->first,
      tags.GetTagByID(tag) * camera_in_tag->second
    }
  );
}

}  // namespace robot_vision