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

namespace {

// Finds Tranfomration from Tag coord sys to Cam coord sys
// Apriltag size is the length of one square
std::optional<std::pair<cv::Mat, cv::Mat>> TransformTagToCam(const Camera& cam, const std::vector<cv::Point2d>& image, double apriltag_size) {
  
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

    // LOG(INFO) << "tvec[0]:\n" << tvec[0] << "\n" << sqrt(tvec[0].at<double>(0,0)*tvec[0].at<double>(0,0)+tvec[0].at<double>(1,0)*tvec[0].at<double>(1,0)+tvec[0].at<double>(2,0)*tvec[0].at<double>(2,0)) << " mm";
    // LOG(INFO) << "tvec[1]:\n" << tvec[1];
    // LOG(INFO) << "Tag to Cam 1\n" << CombRotVec(tvec[0], rot1*MatToRot(cam.pos));
    // LOG(INFO) << "Tag to Cam 2\n" << CombRotVec(tvec[1], rot2*MatToRot(cam.pos));
    
    return std::pair{
      CombRotVec(tvec[0], rot1*MatToRot(cam.pos)), 
      CombRotVec(tvec[1], rot2*MatToRot(cam.pos))
    };
  }

  return std::nullopt;
}

std::optional<std::pair<cv::Mat, cv::Mat>> GetRobotTransform(const Camera& cam, const Tags& tags, int tag, const std::vector<cv::Point2d>& image, double apriltag_size) {
  std::optional<std::pair<cv::Mat, cv::Mat>> tag_to_cam = TransformTagToCam(cam, image, apriltag_size);
  if (!tag_to_cam.has_value()) {
    return std::nullopt;
  }
  // T (R<-C) * T (T<-C)^-1 * T (W<-T)^-1
  // cam.pos  *  TagtoCam   * tags[]
  cv::Mat t_c_i1r, t_c_i2r, w_t_ir, t_c_i1, t_c_i2, w_t_i;
  cv::invert(MatToRot(tag_to_cam->first), t_c_i1r);
  cv::invert(MatToRot(tag_to_cam->second), t_c_i2r);
  cv::invert(MatToRot(tags.GetTagByID(tag)), w_t_ir);
  t_c_i1 = CombRotVec(MatToVec(tag_to_cam->first), t_c_i1r);
  t_c_i2 = CombRotVec(MatToVec(tag_to_cam->second), t_c_i2r);
  w_t_i = CombRotVec(MatToVec(tags.GetTagByID(tag)), w_t_ir);
  // LOG(INFO) << "GetRobtTranform Log: (" << log.rows << ", " << log.cols << ")\n" << log;
  return std::pair{
    CombineTransform(CombineTransform(cam.pos, t_c_i1), w_t_i),
    CombineTransform(CombineTransform(cam.pos, t_c_i2), w_t_i)
  };
}

// THE GREAT WALL OF VISION: ^ Math | Other Vision Stuff v

}  // namespace

std::optional<std::pair<cv::Mat, cv::Mat>> GetRobotPosition(const Camera& cam, const Tags& tags, int tag, const std::vector<cv::Point2d>& image, double apriltag_size) {
  if (!tags.TagExists(tag)) {
    return std::nullopt;
  }
  cv::Mat tag_pos = tags.GetTagByID(tag);
  std::optional<std::pair<cv::Mat, cv::Mat>> transform = GetRobotTransform(cam, tags, tag, image, apriltag_size);
  if (!transform.has_value()) {
    return std::nullopt;
  }
  // LOG(INFO) << "Robot Position 1:\n " << transform->first;
  // LOG(INFO) << "Robot Position 2:\n " << transform->second;
  // LOG(INFO) << "CameraDirection1(x)\n" << (transform->first)*MatToVec(VecToMat(cv::Vec3d(1, 0, 0)));
  // LOG(INFO) << "CameraDirection2(x)\n" << (transform->second)*MatToVec(VecToMat(cv::Vec3d(1, 0, 0)));
  cv::Mat first_pos = (transform->first)*MatToVec(tag_pos);
  cv::Mat second_pos = (transform->second)*MatToVec(tag_pos);
  return std::make_pair(CombRotVec(first_pos, MatToRot(transform->first)), CombRotVec(second_pos, MatToRot(transform->second)));
}

// Returns 4D transformation: Camera -> Tag coord
std::optional<std::pair<cv::Mat, cv::Mat>> GetCameraInTagCoords(const Camera& cam, const std::vector<cv::Point2d>& image, double apriltag_size) {
  // Transofrmation: Tag -> Camera coord
  std::optional<std::pair<cv::Mat, cv::Mat>> tag_to_cam = TransformTagToCam(cam, image, apriltag_size);
  if (!tag_to_cam.has_value()) {
    return std::nullopt;
  }

  auto [tag_to_cam1, tag_to_cam2] = *tag_to_cam;
  cv::Mat inv_tag_to_cam1 = Inverse(tag_to_cam1);
  cv::Mat inv_tag_to_cam2 = Inverse(tag_to_cam2);
  return std::pair{
    inv_tag_to_cam1, 
    inv_tag_to_cam2/*  */
  };
}

cv::Mat GetRobotInCameraCoords(const Camera& cam) {
  cv::Mat inv = Inverse(cam.pos);
  cv::Mat adjusted = (cv::Mat_<double>(4, 4) << 
    inv.at<double>(2, 0), inv.at<double>(2, 1), inv.at<double>(2, 2), inv.at<double>(2, 3),
    inv.at<double>(0, 0), inv.at<double>(0, 1), inv.at<double>(0, 2), inv.at<double>(0, 3),
    -inv.at<double>(1, 0), -inv.at<double>(1, 1), -inv.at<double>(1, 2), -inv.at<double>(1, 3),
    0, 0, 0, 1
  );
  return adjusted;
}

std::optional<std::pair<cv::Mat, cv::Mat>> RobotInTagCoords(const Camera& cam, const std::vector<cv::Point2d>& image, double apriltag_size) {
  std::optional<std::pair<cv::Mat, cv::Mat>> cam_in_tag = GetCameraInTagCoords(cam, image, apriltag_size);
  if (!cam_in_tag.has_value()) {
    return std::nullopt;
  }
  cv::Mat rob_in_cam = GetRobotInCameraCoords(cam);
  return std::optional(std::pair(rob_in_cam*cam_in_tag->first, rob_in_cam*cam_in_tag->second));
}

std::vector<std::pair<int, std::vector<cv::Point2d>>> GetImage(const Camera& cam, const cv::Mat& frame, apriltag_detector_t* td) {
  errno = 0;
  std::vector<std::pair<int, std::vector<cv::Point2d>>> images;

  image_u8_t im = {frame.cols, frame.rows, frame.cols, frame.data};

  zarray_t *detections = apriltag_detector_detect(td, &im);

  //PCHECK(errno == 0) << "Unable to create " << td->nthreads << " threads requested.";
   if (errno == EAGAIN) {
            printf("Unable to create the %d threads requested.\n",td->nthreads);
            exit(-1);
        }


  for (int i=0; i<detections->size; ++i) {
    apriltag_detection_t *det;
    zarray_get(detections, 0, &det);
    std::vector<cv::Point2d> image = {
      cv::Point2d(det->p[0][0], det->p[0][1]),
      cv::Point2d(det->p[1][0], det->p[1][1]),
      cv::Point2d(det->p[2][0], det->p[2][1]),
      cv::Point2d(det->p[3][0], det->p[3][1])
    };

    images.push_back(std::make_pair(det->id, image));
  }

  apriltag_detections_destroy(detections);

  return images;
}

}  // namespace robot_vision