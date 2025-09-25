#include <iostream>
#include <utility>
#include <opencv2/opencv.hpp>

#include "absl/flags/flag.h"
#include "tag36h11.h"

constexpr char famname[] = "tag36h11";

// Read the docs here: https://abseil.io/docs/cpp/guides/flags
ABSL_FLAG(int, camera, 0, "Camera ID");

class RobotVisionError : public std::runtime_error {
  public:
    RobotVisionError(const std::string& error_message) : std::runtime_error(error_message) {}
};

void SerializeMat(std::ostream& out, const cv::Mat& mat) {
  out << mat.rows << " " << mat.cols << std::endl;
  for (int i=0; i<mat.rows; ++i) {
    for (int j=0; j<mat.cols; ++j) {
      out << mat.at<double>(i, j) << " ";
    }
    out << std::endl;
  }
}

cv::Mat DeserializeMat(std::istream& in) {
  int m, n;
  in >> m >> n;
  cv::Mat out(m, n, CV_64F);
  for (int i=0; i<m; ++i) {
    for (int j=0; j<n; ++j) {
      in >> out.at<double>(i, j); 
    }
  }
  return out;
}

struct Camera {
  cv::Mat cam_mat;
  cv::Mat dist_coef;
  cv::Mat pos; // Tranformation matrix to convert from camera position to robot position
  std::vector<cv::Point2i> image;
  cv::Mat tag;

  void Serialize(std::ostream& out) {
    SerializeMat(out, cam_mat);
    SerializeMat(out, dist_coef);
    SerializeMat(out, pos);
    image.clear();
    SerializeMat(out, tag);
  }

  void Deserialize(std::istream& in) {
    cam_mat = DeserializeMat(in);
    dist_coef = DeserializeMat(in);
    pos = DeserializeMat(in);
    image.clear();
    tag = DeserializeMat(in);
  }

};

Camera ConstructCamera(cv::Vec3d cam_pos, double p, double w, double r, cv::Mat cam_mat, cv::Mat dist_coef, std::vector<cv::Point2i> image, cv::Vec3d tag) {
  Camera out;
  cv::Mat pitch = (cv::Mat_<double>(3, 3) <<
    std::cos(p), -std::sin(p), 0,
    std::sin(p), std::cos(p), 0,
    0, 0, 1
  );
  cv::Mat yaw = (cv::Mat_<double>(3, 3) <<
    std::cos(w), 0, std::sin(w),
    0, 1, 0,
    -std::sin(w), 0, std::cos(w)
  );
  cv::Mat roll = (cv::Mat_<double>(3, 3) << 
    1, 0, 0,
    0, std::cos(r), -std::sin(r),
    0, std::sin(r), std::cos(r)
  );
  cv::Mat rot_mat = pitch.mul(yaw).mul(roll);
  out.pos = (cv::Mat_<double>(4, 4) << 
    rot_mat.at<double>(0, 0), rot_mat.at<double>(0, 1), rot_mat.at<double>(0, 2), cam_pos[0],
    rot_mat.at<double>(1, 0), rot_mat.at<double>(1, 1), rot_mat.at<double>(1, 2), cam_pos[1],
    rot_mat.at<double>(2, 0), rot_mat.at<double>(2, 1), rot_mat.at<double>(2, 2), cam_pos[2],
    0, 0, 0, 1
  );
  out.image = image;
  out.tag = (cv::Mat_<double>(4, 1) << 
    tag[0], 
    tag[1], 
    tag[2], 
    1
  );
  out.cam_mat = cam_mat;
  out.dist_coef = dist_coef;
  return out;
}

Camera CreateCamFromImages(int cam_n, int pic_num)
{
  cv::Mat frame;

  cv::VideoCapture cap;
  cap.open(cam_n);

  std::vector<std::vector<cv::Point2i>> points;

  if (!cap.isOpened()) {
    std::cerr << "ERROR! Unable to open camera\n";
  }

  while (true) {
    cap.read(frame);

    if (frame.empty()) {
      std::cerr << "ERROR! blank frame grabbed\n";
      break;
    }

    int key = cv::waitKey(30);

    imshow("Live", frame);

    if (key == ' ') {
      std::vector<cv::Point2i> find_out;
      cv::findChessboardCorners(frame, cv::Size(5, 5), find_out);
      points.push_back(find_out);
    }

    if (key >= 0) {
      break;
    }
  }
}


/*
TODO:
Implement not succesful case

Create an error message fromat
*/

std::vector<cv::Mat> TagTransformCam(const Camera& cam) {
  std::vector<cv::Point3d> object;
  std::vector<cv::Mat> out;
  
  // half of apritag size
  double hs = 82.25;
  object.push_back(cv::Point3d(-hs, hs, 0));
  object.push_back(cv::Point3d(hs, hs, 0));
  object.push_back(cv::Point3d(hs, -hs, 0));
  object.push_back(cv::Point3d(-hs, -hs, 0));

  std::vector<cv::Vec3d>tvec;
  std::vector<cv::Mat> rvec;

  bool succ = cv::solvePnPGeneric(
    object,
    cam.image,
    cam.cam_mat,
    cam.dist_coef,
    rvec,
    tvec,
    false,
    cv::SOLVEPNP_IPPE_SQUARE
  );

  if (succ) {
    cv::Mat rot1, rot2;
    cv::Rodrigues(rvec[0], rot1);
    cv::Rodrigues(rvec[1], rot2);

    cv::Mat mat1 = (cv::Mat_<double>(4, 4) << 
    rot1.at<double>(0, 0), rot1.at<double>(0, 1), rot1.at<double>(0, 2), tvec[0][0],
    rot1.at<double>(1, 0), rot1.at<double>(1, 1), rot1.at<double>(1, 2), tvec[0][1],
    rot1.at<double>(2, 0), rot1.at<double>(2, 1), rot1.at<double>(2, 2), tvec[0][2],
    0, 0, 0, 1
    );
    cv::Mat mat2 = (cv::Mat_<double>(4, 4) <<
    rot2.at<double>(0, 0), rot2.at<double>(0, 1), rot2.at<double>(0, 2), tvec[1][0],
    rot2.at<double>(1, 0), rot2.at<double>(1, 1), rot2.at<double>(1, 2), tvec[1][1],
    rot2.at<double>(2, 0), rot2.at<double>(2, 1), rot2.at<double>(2, 2), tvec[1][2],
    0, 0, 0, 1
    );
    out.push_back(mat1);
    out.push_back(mat2);
  }

  return out;
}

void Translation(cv::Mat& mat, const cv::Mat& add) {
  cv::Mat to_add = (cv::Mat_<double>(4, 4) << 
    0, 0, 0, add.at<double>(0, 0),
    0, 0, 0, add.at<double>(1, 0),
    0, 0, 0, add.at<double>(2, 0),
    0, 0, 0, 0
  );
  mat += add;
}

std::vector<cv::Mat> GetRobotPositionFromCam(Camera cam) {
  std::vector<cv::Mat> transform_cam = TagTransformCam(cam);;
  
  cv::Mat ans1 = cam.tag.mul(transform_cam[0]).mul(cam.pos);
  Translation(ans1, cam.pos);
  cv::Mat ans2 = cam.tag.mul(transform_cam[1]).mul(cam.pos);
  Translation(ans2, cam.pos);

  std::vector<cv::Mat> out;

  out.push_back(ans1);
  out.push_back(ans2);

  return out;
}

double GetDif(cv::Mat a, cv::Mat b) {
  return (b.at<double>(3, 0)-a.at<double>(3,0))*(b.at<double>(3,0)-a.at<double>(3,0))+(b.at<double>(3,1)-a.at<double>(3,1))*(b.at<double>(3,1)-a.at<double>(3,1))+(b.at<double>(3,2)-a.at<double>(3,2))*(b.at<double>(3,2)-a.at<double>(3,2));
}

// TODO implement error break

cv::Vec3d RobotPosition(std::vector<Camera> cams, double tol) {
  if (cams.size() == 0) {
    std::cout << "No cameras found :(" << std::endl;
    return cv::Vec3d(0, 0, 0);
  }
  std::vector<cv::Mat> ovr = std::vector<cv::Mat>(cams.size()*2);
  for (const Camera& e : cams) {
    std::vector<cv::Mat> pos = GetRobotPositionFromCam(e);
    for (cv::Mat m : pos) {
      ovr.push_back(m);
    }
  }
  cv::Mat p1 = ovr[0];
  std::vector<cv::Mat> p1vec;
  cv::Mat p2 = ovr[1];
  std::vector<cv::Mat> p2vec;
  p1vec.push_back(p1);
  p2vec.push_back(p2);
  for (int i=2; i<ovr.size(); ++i) {
    if (GetDif(p1, ovr[i]) < tol) {
      p1vec.push_back(ovr[i]);
    }
    if (GetDif(p2, ovr[i]) < tol) {
      p2vec.push_back(ovr[i]);
    }
  }
  std::vector<cv::Mat> larger = (p1vec.size() > p2vec.size() ? p1vec : p2vec);
  cv::Mat sum = (cv::Mat_<double>(4, 1) << 
    0, 
    0, 
    0,
    0
  );
  for (int i=0; i<larger.size(); ++i) {
    sum += larger[i];
  }
  cv::Mat divd = sum/larger.size();
  cv::Vec3d out = cv::Vec3d(divd.at<double>(0, 0), divd.at<double>(0, 1), divd.at<double>(0, 2));
  return out;
}

void GetImages(Camera cam) {
  std::cout << "Enabling video capture" << std::endl;

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

  cv::Mat frame;
  while (true) {
    errno = 0;
    cap >> frame;

    image_u8_t im = {frame.cols, frame.rows, frame.cols, frame.data};

    zarray_t *detections = apriltag_detector_detect(td, &im);

    if (errno == EAGAIN) {
      printf("Unable to create the %d threads requested.\n",td->nthreads);
      exit(-1);
    }

    apriltag_detection_t *det;
    zarray_get(detections, 0, &det);
    cam.image.clear();
    cam.image.push_back(cv::Point2i(det->p[0][0], det->p[0][1]));
    cam.image.push_back(cv::Point2i(det->p[1][0], det->p[1][1]));
    cam.image.push_back(cv::Point2i(det->p[2][0], det->p[2][1]));
    cam.image.push_back(cv::Point2i(det->p[3][0], det->p[3][1]));

    apriltag_detections_destroy(detections);

    std::vector<cv::Mat> pos = GetRobotPositionFromCam(cam);

    for (cv::Mat e : pos) {
      std::cout << e.at<double>(0, 0) << " " << e.at<double>(0, 1) << " " << e.at<double>(0, 2) << std::endl;
    }
 
    if (cv::waitKey(30) >= 0) {
      break;
    }
  }

  apriltag_detector_destroy(td);

  tag36h11_destroy(tf);
}

int main() {
  return 0;
}