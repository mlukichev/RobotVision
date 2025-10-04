#include <iostream>
#include <utility>
#include <algorithm>
#include <optional>
#include <opencv2/opencv.hpp>

#include "camera.h"
#include "transformations.h"
#include "absl/flags/flag.h"
#include "tag36h11.h"

namespace robot_vision {

namespace {

constexpr char famname[] = "tag36h11";

const int tags_len = 1;
const std::pair<int, cv::Mat> tags[] = {
  std::make_pair(
    0,
    (cv::Mat_<double>(4, 4) << 
      0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 1
    )
  )
};

int FindTagByID(int tag) {
  for (int i=0; i<tags_len; ++i) {
    if (tags[i].first == tag) {
      return i;
    }
  }
  return -1;
}

// Apriltag size is the length of one square
std::optional<std::pair<cv::Mat, cv::Mat>> TransformTagToCam(const Camera& cam, int tag, const std::vector<cv::Point2i>& image, double apriltag_size) {
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

    cv::Mat pos;
    int t_pos = FindTagByID(tag);
    if (t_pos == -1) {
      t_pos = 0;
    }
    pos = tags[t_pos].second;

    return std::make_pair(CombRotVec(tvec[0], rot1*MatToRot(cam.pos)), CombRotVec(tvec[1], rot2*MatToRot(cam.pos)));
  }

  return std::nullopt;
}

std::optional<std::pair<cv::Mat, cv::Mat>> GetRobotTransform(const Camera& cam, int tag, const std::vector<cv::Point2i>& image, double apriltag_size) {
  std::optional<std::pair<cv::Mat, cv::Mat>> tag_to_cam = TransformTagToCam(cam, tag, image, apriltag_size);
  if (tag_to_cam == std::nullopt) {
    return std::nullopt;
  }
  // T (R<-C) * T (T<-C)^-1 * T (W<-T)^-1
  // cam.pos  *  TagtoCam   * tags[]
  cv::Mat t_c_i1, t_c_i2, w_t_i;
  cv::invert(tag_to_cam->first, t_c_i1);
  cv::invert(tag_to_cam->second, t_c_i2);
  cv::invert(tags[tag].second, w_t_i);
  return std::make_pair(cam.pos*t_c_i1*w_t_i, cam.pos*t_c_i2*w_t_i);
}

// THE GREAT WALL OF VISION: ^ Math | Other Vision Stuff v


std::pair<bool, std::vector<std::vector<cv::Point2i>>> GetImage(const Camera& cam, cv::VideoCapture cap, apriltag_detector_t* td) {
  std::vector<std::vector<cv::Point2i>> images;
  cv::Mat frame;
  errno = 0;
  cap >> frame;

  image_u8_t im = {frame.cols, frame.rows, frame.cols, frame.data};

  zarray_t *detections = apriltag_detector_detect(td, &im);

  if (detections->size == 0) {
    return std::make_pair(false, std::vector<std::vector<cv::Point2i>>());
  }

  if (errno == EAGAIN) {
    printf("Unable to create the %d threads requested.\n",td->nthreads);
    exit(-1);
  }

  for (int i=0; i<detections->size; ++i) {
    std::vector<cv::Point2i> image;
    apriltag_detection_t *det;
    zarray_get(detections, 0, &det);
    image.push_back(cv::Point2i(det->p[0][0], det->p[0][1]));
    image.push_back(cv::Point2i(det->p[1][0], det->p[1][1]));
    image.push_back(cv::Point2i(det->p[2][0], det->p[2][1]));
    image.push_back(cv::Point2i(det->p[3][0], det->p[3][1]));

    images.push_back(image);
  }

  apriltag_detections_destroy(detections);
}

void GetRobotPositionTest(const Camera& cam) {
  cv::TickMeter meter;
  meter.start();

  cv::VideoCapture cap(0);
  if (!cap.isOpened()) {
    std::cerr << "Couldn't open video capture device" << std::endl;
    return;
  }

  apriltag_family_t *tf = tag36h11_create();
  
  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);

  if (errno == ENOMEM) {
    printf("Unable to add family to detector due to insufficient memory to allocate the tag-family decoder with the default maximum hamming value of 2. Try choosing an alternative tag family.\n");
    exit(-1);
  }

  td->quad_decimate = 2.0;
  td->quad_sigma = 0;
  td->nthreads = 1;
  td->debug = false;
  td->refine_edges = true;

  meter.stop();
  std::cout << "Detector " << famname << " initialized in " << std::fixed << std::setprecision(3) << meter.getTimeSec() << " seconds" << std::endl;
  meter.reset();

  std::cout << "Enter any key to begin calculation" << std::endl;
  cv::waitKey(0);

  // Main Loop
  while (true) {
    auto [detected, images] = GetImage(cam, cap, td);
    if (detected) {
      for (std::vector<cv::Point2i> e : images) {
        
        std::cout << std::endl;
      }
    }
    if (cv::waitKey(30) >= 0) {
      break;
    }
  }

  apriltag_detector_destroy(td);

  tag36h11_destroy(tf);
}

}  // namespace

std::optional<std::pair<cv::Mat, cv::Mat>> GetRobotPosition(const Camera& cam, int tag, const std::vector<cv::Point2i>& image, double apriltag_size) {
  std::optional<std::pair<cv::Mat, cv::Mat>> transform = GetRobotTransform(cam, tag, image, apriltag_size);
  if (transform == std::nullopt) {
    return std::nullopt;
  }
  return std::make_pair((transform->first)*MatToVec(tags[tag].second), (transform->second)*MatToVec(tags[tag].second));
}

}  // namespace robot_vision

