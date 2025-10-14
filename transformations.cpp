#include "transformations.h"

#include <opencv2/opencv.hpp>

namespace robot_vision {

namespace {

cv::Mat AddTransform(const cv::Mat& mat1, const cv::Mat& mat2) {
  cv::Mat out = mat1+mat2;
  out.at<double>(3, 3) = 1;
  return out;
}

}

cv::Mat VecToMat(const cv::Vec3d& vec) {
  cv::Mat out = (cv::Mat_<double>(4, 4) << 
    0, 0, 0, vec[0],
    0, 0, 0, vec[1],
    0, 0, 0, vec[2],
    0, 0, 0, 1
  );
  return out;
}

cv::Mat VecToMat(const cv::Mat& vec) {
  cv::Mat out = (cv::Mat_<double>(4, 4) << 
    0, 0, 0, vec.at<double>(0, 0),
    0, 0, 0, vec.at<double>(1, 0),
    0, 0, 0, vec.at<double>(2, 0),
    0, 0, 0, 1
  );
  return out;
}

cv::Vec3d MatToVec3d(const cv::Mat& mat) {
  cv::Vec3d out = cv::Vec3d(
    mat.at<double>(0, 3),
    mat.at<double>(1, 3),
    mat.at<double>(2, 3)
  );
  return out;
}

cv::Mat MatToVec(const cv::Mat& mat) {
  cv::Mat out = (cv::Mat_<double>(4, 1) << 
    mat.at<double>(0, 3),
    mat.at<double>(1, 3),
    mat.at<double>(2, 3),
    1
  );
  return out;
}

cv::Mat MatToRot(const cv::Mat& mat) {
  cv::Mat out = (cv::Mat_<double>(3, 3) << 
    mat.at<double>(0, 0), mat.at<double>(0, 1), mat.at<double>(0, 2),
    mat.at<double>(1, 0), mat.at<double>(1, 1), mat.at<double>(1, 2),
    mat.at<double>(2, 0), mat.at<double>(2, 1), mat.at<double>(2, 2)
  );
  return out;
}

cv::Mat RotToMat(const cv::Mat& rot) {
  cv::Mat out = (cv::Mat_<double>(4, 4) << 
    rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2), 0,
    rot.at<double>(1, 0), rot.at<double>(1, 1), rot.at<double>(1, 2), 0,
    rot.at<double>(2, 0), rot.at<double>(2, 1), rot.at<double>(2, 2), 0,
    0, 0, 0, 1
  );
  return out;
}

cv::Mat CombRotVec(const cv::Mat& vec, const cv::Mat& rot) {
  return AddTransform(RotToMat(rot), VecToMat(vec));
}

cv::Mat CombRotVec(const cv::Vec3d& vec, const cv::Mat& rot) {
  return AddTransform(RotToMat(rot), VecToMat(vec));
}

cv::Mat Translation(const cv::Mat& mat, const cv::Mat& translation) {
  return AddTransform(mat, VecToMat(translation));
}

cv::Mat Translation(const cv::Mat& mat, const cv::Vec3d& translation) {
  return AddTransform(mat, VecToMat(translation));
}

cv::Mat Rotation(const cv::Mat& mat, const cv::Mat& rotation) {
  return RotToMat(rotation*MatToRot(mat));
}

cv::Mat CombineTransform(const cv::Mat& mat1, const cv::Mat& mat2) {
  return CombRotVec(mat1*MatToVec(mat2), MatToRot(mat1)*MatToRot(mat2));
}

cv::Mat CombineRotation(cv::Mat mat) {
  cv::Mat U, W, Vt;
  cv::SVD::compute(mat, W, U, Vt);

  double det_U = cv::determinant(U);
  double det_Vt = cv::determinant(Vt);

  if (abs(det_U*det_Vt - 1) <= 0.01) {
    return U*Vt;
  } else {
    cv::Mat flip = (cv::Mat_<double>(3, 3) <<
      1, 0, 0,
      0, 1, 0,
      0, 0, -1
    );
    return U*(Vt*flip);
  }
}

cv::Mat Inverse(const cv::Mat& mat) {
  cv::Mat inv;
  cv::invert(mat, inv);
  return inv;
}

cv::Mat CheckOrtho(const cv::Mat& mat) {
  return mat * mat.t();
}

double GetSquareDist(const cv::Vec3d& a, const cv::Vec3d& b) {
  return (a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2]);
}

}  // namespace robot_vision