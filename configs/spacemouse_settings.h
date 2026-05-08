#pragma once

#include <vector>

#include "drake/common/yaml/yaml_read_archive.h"

struct SpacemouseSettings {
  bool zero_when_static = true;
  int static_count_threshold = 30;
  double static_trans_deadband = 0.1;
  double static_rot_deadband = 0.1;
  double full_scale = 512.0;
  std::vector<int> linear_axis = {0, 2, 1};
  std::vector<double> linear_sign = {1.0, 1.0, 1.0};
  std::vector<int> angular_axis = {0, 2, 1};
  std::vector<double> angular_sign = {1.0, 1.0, 1.0};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(zero_when_static));
    a->Visit(DRAKE_NVP(static_count_threshold));
    a->Visit(DRAKE_NVP(static_trans_deadband));
    a->Visit(DRAKE_NVP(static_rot_deadband));
    a->Visit(DRAKE_NVP(full_scale));
    a->Visit(DRAKE_NVP(linear_axis));
    a->Visit(DRAKE_NVP(linear_sign));
    a->Visit(DRAKE_NVP(angular_axis));
    a->Visit(DRAKE_NVP(angular_sign));
  }
};
