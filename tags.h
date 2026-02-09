#ifndef TAGS_H
#define TAGS_H

#include <opencv2/opencv.hpp>
#include <optional>
#include <unordered_map>
#include <iostream>
#include <fstream>

#include "transformations.h"

namespace robot_vision {

using TagId = int;

class Tags {
 public:
  bool TagExists(TagId tag) const;
  std::optional<std::reference_wrapper<const Transformation>> GetTagToWorld(TagId tag) const;
  void emplace(TagId tag, Transformation pos);
   
 private:
  std::unordered_map<int, Transformation> tag_to_world_;
};

Tags ReadTags(const std::string& filename);

}

#endif // TAGS_H