#include <opencv2/opencv.hpp>
#include "camera_handling.h"

double GetDif(cv::Mat a, cv::Mat b) {
  double dif1 = b.at<double>(0, 0)-a.at<double>(0, 0);
  double dif2 = b.at<double>(1, 0)-a.at<double>(1, 0);
  double dif3 = b.at<double>(2, 0)-a.at<double>(2, 0);
  return sqrt(dif1*dif1+dif2*dif2+dif3*dif3);
}

cv::Mat RobotPosition(const std::vector<cv::Mat>& solutions, double tol) {

}

// Will handle sending data n stuff
int main() {

}