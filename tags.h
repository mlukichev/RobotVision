#ifndef TAGS_H
#define TAGS_H

#include <opencv2/opencv.hpp>
#include <optional>
#include <unordered_map>

namespace robot_vision {

class Tags {
 public:
  Tags();
  bool TagExists(int tag) const;
  cv::Mat GetTagByID(int tag) const;
   
 private:
  std::unordered_map<int, cv::Mat> tags_;
};

}

#endif // TAGS_H