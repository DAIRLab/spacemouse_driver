#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

struct SpacemouseSettings {
  bool zero_when_static = true;
  int static_count_threshold = 30;
  double static_trans_deadband = 0.1;
  double static_rot_deadband = 0.1;
  double full_scale = 512.0;
  std::vector<double> linear_scale = {1, 1, 1};
  std::vector<double> angular_scale = {1, 1, 1};

  template <typename Archive> void Serialize(Archive *a) {
    a->Visit(DRAKE_NVP(zero_when_static));
    a->Visit(DRAKE_NVP(static_count_threshold));
    a->Visit(DRAKE_NVP(static_trans_deadband));
    a->Visit(DRAKE_NVP(static_rot_deadband));
    a->Visit(DRAKE_NVP(full_scale));
    a->Visit(DRAKE_NVP(linear_scale));
    a->Visit(DRAKE_NVP(angular_scale));
  }
};
