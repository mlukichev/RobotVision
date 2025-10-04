#ifndef TRANSFORMATIONS_H
#define TRANSFORMATIONS_H

#include <opencv2/opencv.hpp>

namespace robot_vision {

cv::Mat VecToMat(const cv::Vec3d& vec);

cv::Mat VecToMat(const cv::Mat& vec);

cv::Vec3d MatToVec3d(const cv::Mat& mat);

cv::Mat MatToVec(const cv::Mat& mat);

cv::Mat MatToRot(const cv::Mat& mat);

cv::Mat RotToMat(const cv::Mat& rot);

cv::Mat CombRotVec(const cv::Mat& vec, const cv::Mat& rot);

cv::Mat CombRotVec(const cv::Vec3d& vec, const cv::Mat& rot);

cv::Mat Translation(const cv::Mat& mat, const cv::Mat& translation);

cv::Mat Translation(const cv::Mat& mat, const cv::Vec3d& translation);

cv::Mat Rotation(const cv::Mat& mat, const cv::Mat& rotation);

cv::Mat CombineTransform(const cv::Mat& mat1, const cv::Mat& mat2);

}  // namespace robot_vision

#endif