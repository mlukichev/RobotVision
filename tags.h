#ifndef TAGS_H
#define TAGS_H

#include <opencv2/opencv.hpp>
#include <optional>
#include <unordered_map>
#include <iostream>
#include <fstream>

#include "transformations.h"
#include "absl/synchronization/mutex.h"

namespace robot_vision {

using TagId = int;

class Tags {
 public:
  Tags() = default;
  Tags(Tags&& other);

  bool TagExists(TagId tag) const;
  std::optional<Transformation> GetTagToWorld(TagId tag);
  void emplace(TagId tag, Transformation pos);
  Tags& operator=(Tags&& other);
   
 private:
  absl::Mutex mu_;
  std::unordered_map<int, Transformation> tag_to_world_;
};

Tags ReadTags(const std::string& filename);

}

#endif // TAGS_H