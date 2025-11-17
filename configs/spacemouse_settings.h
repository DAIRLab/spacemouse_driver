#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

struct SpacemouseSettings {
  bool zero_when_static = true;
  int static_count_threshold = 30;
  double static_trans_deadband = 0.1;
  double static_rot_deadband = 0.1;
  double full_scale = 512.0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(zero_when_static));
    a->Visit(DRAKE_NVP(static_count_threshold));
    a->Visit(DRAKE_NVP(static_trans_deadband));
    a->Visit(DRAKE_NVP(static_rot_deadband));
    a->Visit(DRAKE_NVP(full_scale));
  }
};

struct URSpacemouseSettings : public SpacemouseSettings {
  std::vector<double> ur_linear_scale = {0.01, 0.01, 0.01};
  std::vector<double> ur_angular_scale = {0.05, 0.05, 0.05};

  template <typename Archive>
  void Serialize(Archive* a) {
    SpacemouseSettings::Serialize(a);
    a->Visit(DRAKE_NVP(ur_linear_scale));
    a->Visit(DRAKE_NVP(ur_angular_scale));
  }
};

struct FrankaSpacemouseSettings : public SpacemouseSettings {
  double franka_linear_scale = 1;
  double franka_angular_scale = 1;

  template <typename Archive>
  void Serialize(Archive* a) {
    SpacemouseSettings::Serialize(a);
    a->Visit(DRAKE_NVP(franka_linear_scale));
    a->Visit(DRAKE_NVP(franka_angular_scale));
  }
};
