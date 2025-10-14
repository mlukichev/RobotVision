#ifndef TAGS_H
#define TAGS_H

#include <opencv2/opencv.hpp>
#include <optional>
#include <unordered_map>

#include "transformations.h"

namespace robot_vision {

using TagId = int;

class Tags {
 public:
  Tags();
  bool TagExists(TagId tag) const;
  Transformation GetTagByID(TagId tag) const;
   
 private:
  std::unordered_map<int, Transformation> tags_;
};

}

#endif // TAGS_H