#include <iostream>
#include <utility>
#include <algorithm>
#include <optional>
#include <opencv2/opencv.hpp>

#include "camera.h"
#include "transformations.h"
#include "absl/log/check.h"
#include "apriltag.h"
#include "tag36h11.h"
#include "tags.h"

namespace robot_vision {

namespace {

// Finds Tranfomration from Tag coord sys to Cam coord sys
// Apriltag size is the length of one square
std::optional<std::pair<cv::Mat, cv::Mat>> TransformTagToCam(const Camera& cam, const std::vector<cv::Point2i>& image, double apriltag_size) {
  std::vector<cv::Point3d> object;
  
  // half of apritag size
  double hs = apriltag_size*5;
  object.push_back(cv::Point3d(-hs, hs, 0));
  object.push_back(cv::Point3d(hs, hs, 0));
  object.push_back(cv::Point3d(hs, -hs, 0));
  object.push_back(cv::Point3d(-hs, -hs, 0));

  std::vector<cv::Vec3d>tvec;
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
    
    return std::make_pair(CombRotVec(tvec[0], rot1*MatToRot(cam.pos)), CombRotVec(tvec[1], rot2*MatToRot(cam.pos)));
  }

  return std::nullopt;
}

std::optional<std::pair<cv::Mat, cv::Mat>> GetRobotTransform(const Camera& cam, const Tags& tags, int tag, const std::vector<cv::Point2i>& image, double apriltag_size) {
  std::optional<std::pair<cv::Mat, cv::Mat>> tag_to_cam = TransformTagToCam(cam, image, apriltag_size);
  if (!tag_to_cam.has_value()) {
    return std::nullopt;
  }
  // T (R<-C) * T (T<-C)^-1 * T (W<-T)^-1
  // cam.pos  *  TagtoCam   * tags[]
  cv::Mat t_c_i1, t_c_i2, w_t_i;
  cv::invert(tag_to_cam->first, t_c_i1);
  cv::invert(tag_to_cam->second, t_c_i2);
  cv::invert(tags.GetTagByID(tag), w_t_i);
  return std::make_pair(cam.pos*t_c_i1*w_t_i, cam.pos*t_c_i2*w_t_i);
}

// THE GREAT WALL OF VISION: ^ Math | Other Vision Stuff v

}  // namespace

std::optional<std::pair<cv::Mat, cv::Mat>> GetRobotPosition(const Camera& cam, const Tags& tags, int tag, const std::vector<cv::Point2i>& image, double apriltag_size) {
  if (!tags.TagExists(tag)) {
    return std::nullopt;
  }
  cv::Mat tag_pos = tags.GetTagByID(tag);
  std::optional<std::pair<cv::Mat, cv::Mat>> transform = GetRobotTransform(cam, tags, tag, image, apriltag_size);
  if (!transform.has_value()) {
    return std::nullopt;
  }
  cv::Mat first_pos = (transform->first)*MatToVec(tag_pos);
  cv::Mat second_pos = (transform->second)*MatToVec(tag_pos);
  return std::make_pair(CombRotVec(first_pos, MatToRot(transform->first)), CombRotVec(second_pos, MatToRot(transform->second)));
}

std::vector<std::pair<int, std::vector<cv::Point2i>>> GetImage(const Camera& cam, cv::VideoCapture cap, apriltag_detector_t* td) {
  std::vector<std::pair<int, std::vector<cv::Point2i>>> images;
  cv::Mat frame;
  errno = 0;
  cap >> frame;

  image_u8_t im = {frame.cols, frame.rows, frame.cols, frame.data};

  zarray_t *detections = apriltag_detector_detect(td, &im);

  PCHECK(errno == 0) << "Unable to create " << td->nthreads << " threads requested.";

  for (int i=0; i<detections->size; ++i) {
    std::vector<cv::Point2i> image;
    apriltag_detection_t *det;
    zarray_get(detections, 0, &det);
    image.push_back(cv::Point2i(det->p[0][0], det->p[0][1]));
    image.push_back(cv::Point2i(det->p[1][0], det->p[1][1]));
    image.push_back(cv::Point2i(det->p[2][0], det->p[2][1]));
    image.push_back(cv::Point2i(det->p[3][0], det->p[3][1]));

    images.push_back(std::make_pair(det->id, image));
  }

  apriltag_detections_destroy(detections);

  return images;
}

}  // namespace robot_vision

