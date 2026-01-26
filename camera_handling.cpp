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
// Apriltag size is the 1/2 length of one square
std::optional<std::pair<Transformation, Transformation>> GetTagToCam(const Camera& cam, const std::vector<cv::Point2d>& image, double apriltag_size) {
  
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

  // cv::Mat rot_e = (cv::Mat_<double>(3, 3) << 
  //   1, 0, 0,
  //   0, 1, 0,
  //   0, 0, 1
  // );

  // cv::Mat rvec_e;

  // cv::Rodrigues(rot_e, rvec_e);

  // cv::Vec3d tvec_e(0, 0, -1102.36);

  // cv::Mat rerror;

  if (cv::solvePnPGeneric(
    object,
    image,
    cam.GetCamMat(),
    cam.GetDistCoef(),
    rvec,
    tvec,
    false,
    cv::SOLVEPNP_IPPE_SQUARE/*,*/
    // rvec_e,
    // tvec_e,
    // rerror
  )) {

    // LOG(INFO) << "Reprojection error: \n" << rerror;

    cv::Mat rot1, rot2;
    cv::Rodrigues(rvec[0], rot1);
    cv::Rodrigues(rvec[1], rot2);

    Transformation p1 = Transformation(tvec[0], rot1);
    Transformation p2 = Transformation(tvec[1], rot2);
    
    return std::pair(p1, p2);
  }

  return std::nullopt;
}

// Returns 4D transformation: Camera -> Tag coord
std::optional<std::pair<Transformation, Transformation>> GetCamToTag(
  const Camera& cam, const std::vector<cv::Point2d>& image, double apriltag_size) {
  // Transofrmation: Tag -> Camera coord
  std::optional<std::pair<Transformation, Transformation>> tag_to_cam = GetTagToCam(cam, image, apriltag_size);
  if (!tag_to_cam.has_value()) {
    return std::nullopt;
  }

  auto [tag_to_cam1, tag_to_cam2] = *tag_to_cam;
  return std::pair{
    tag_to_cam1.Inverse(), 
    tag_to_cam2.Inverse()
  };
}

std::optional<std::pair<Transformation, Transformation>> GetCamToWorld(
  const Camera& cam, const Tags& tags, TagId tag, const std::vector<cv::Point2d>& image, double apriltag_size) {
  std::optional<std::pair<Transformation, Transformation>> camera_to_tag = GetCamToTag(cam, image, apriltag_size);
  std::optional<std::reference_wrapper<const Transformation>> tag_to_world = tags.GetTagToWorld(tag);
  if (!camera_to_tag.has_value() || !tag_to_world.has_value()) {
    return std::nullopt;
  }
  const auto& [cam_to_tag1, cam_to_tag2] = *camera_to_tag;
  return std::optional(
    std::pair{
      cam_to_tag1 * (*tags.GetTagToWorld(tag)),
      cam_to_tag2 * (*tags.GetTagToWorld(tag))
    }
  );
}

std::optional<Transformation> GetCamToWorld(const Tags& tags, TagId tag, const Transformation& cam_to_tag) {
  std::optional<std::reference_wrapper<const Transformation>> tag_to_world = tags.GetTagToWorld(tag);
  if (!tag_to_world.has_value()) {
    return std::nullopt;
  }
  return cam_to_tag * (*tag_to_world);
}


std::optional<std::pair<Transformation, Transformation>> GetRobotToWorld(
  const Camera& cam, const Tags& tags, TagId tag, const CameraPositions& cams, CameraId cam_id, const std::vector<cv::Point2d>& image, double apriltag_size) {
  std::optional<std::pair<Transformation, Transformation>> camera_to_world = GetCamToWorld(cam, tags, tag, image, apriltag_size);
  if (!camera_to_world.has_value()) {
    return std::nullopt;
  }
  const auto& [cam_to_world1, cam_to_world2] = *camera_to_world;
  return std::optional(
    std::pair{
      cams.GetRobotToCamera(cam_id) * cam_to_world1,
      cams.GetRobotToCamera(cam_id) * cam_to_world2 
    }
  );
}

std::optional<Transformation> GetRobotToWorld(
  const Tags& tags, TagId tag, const CameraPositions& cams, CameraId cam_id, const Transformation& cam_to_tag) {
  std::optional<Transformation> cam_to_world = GetCamToWorld(tags, tag, cam_to_tag);
  if (!cam_to_world.has_value()) {
    return std::nullopt;
  }
  return cams.GetRobotToCamera(cam_id) * (*cam_to_world);
}

}  // namespace robot_vision