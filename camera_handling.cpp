#include <iostream>
#include <utility>
#include <algorithm>
#include <optional>

#include "opencv2/opencv.hpp"
#include "camera.h"
#include "transformations.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "apriltag.h"
#include "tag36h11.h"
#include "tags.h"
#include "camera_positions.h"

namespace robot_vision {

// Finds Tranfomration from Tag coord sys to Cam coord sys
// Apriltag size is the length of one square
std::optional<std::pair<Transformation, Transformation>> GetTagInCamCoords(const Camera& cam, const std::vector<cv::Point2d>& image, double apriltag_size) {
  
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
    cam.GetCamMat(),
    cam.GetDistCoef(),
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
std::optional<std::pair<Transformation, Transformation>> GetCamInTagCoords(const Camera& cam, const std::vector<cv::Point2d>& image, double apriltag_size) {
  // Transofrmation: Tag -> Camera coord
  std::optional<std::pair<Transformation, Transformation>> tag_to_cam = GetTagInCamCoords(cam, image, apriltag_size);
  if (!tag_to_cam.has_value()) {
    return std::nullopt;
  }

  auto [tag_to_cam1, tag_to_cam2] = *tag_to_cam;
  return std::pair{
    tag_to_cam1.Inverse(), 
    tag_to_cam2.Inverse()
  };
}

std::optional<std::pair<Transformation, Transformation>> GetCamInWorldCoords(const Camera& cam, const Tags& tags, TagId tag, const std::vector<cv::Point2d>& image, double apriltag_size) {
  std::optional<std::pair<Transformation, Transformation>> camera_in_tag = GetCamInTagCoords(cam, image, apriltag_size);
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

Transformation GetCamInWorldCoords(const Tags& tags, TagId tag, const Transformation& camera_in_tag) {
  return tags.GetTagByID(tag) * camera_in_tag;
}


std::optional<std::pair<Transformation, Transformation>> GetRobotInWorldCoords(const Camera& cam, const Tags& tags, TagId tag, const CameraPositions& cams, CameraId cam_id, const std::vector<cv::Point2d>& image, double apriltag_size) {
  std::optional<std::pair<Transformation, Transformation>> camera_in_world = GetCamInWorldCoords(cam, tags, tag, image, apriltag_size);
  if (!camera_in_world.has_value()) {
    return std::nullopt;
  }
  return std::optional(
    std::pair{
      camera_in_world->first * cams.GetCameraPositionById(cam_id).Inverse(),
      camera_in_world->second * cams.GetCameraPositionById(cam_id).Inverse()
    }
  );
}

Transformation GetRobotInWorldCoords(const Tags& tags, TagId tag, const CameraPositions& cams, CameraId cam_id, const Transformation& camera_in_tag) {
  Transformation camera_in_world = GetCamInWorldCoords(tags, tag, camera_in_tag);
  return (camera_in_world*cams.GetCameraPositionById(cam_id).Inverse());
}

}  // namespace robot_vision