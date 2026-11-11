// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include "nav2_elevation_difference_costmap_plugin/terrain_model.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <stdexcept>

namespace nav2_elevation_difference_costmap_plugin
{
inline std::vector<Point> readPoints(const sensor_msgs::msg::PointCloud2 & cloud)
{
  if (cloud.is_bigendian || !cloud.point_step || cloud.point_step > 1024 ||
    uint64_t(cloud.width) * cloud.height > 1000000 ||
    cloud.row_step < uint64_t(cloud.point_step) * cloud.width ||
    uint64_t(cloud.row_step) * cloud.height > cloud.data.size())
  {
    throw std::invalid_argument("Unsupported or truncated PointCloud2 layout");
  }
  std::array<int, 3> offsets{-1, -1, -1};
  for (const auto & field:cloud.fields) {
    for (size_t i = 0; i < 3; ++i) {
      if (field.name == std::array<const char *, 3>{"x", "y", "z"}[i]) {
        if (field.datatype != sensor_msgs::msg::PointField::FLOAT32 || field.count != 1 ||
          uint64_t(field.offset) + 4 > cloud.point_step)
        {
          throw std::invalid_argument("Expected FLOAT32 XYZ");
        }
        offsets[i] = int(field.offset);
      }
    }
  }
  if (std::any_of(offsets.begin(), offsets.end(), [](int v) {return v < 0;})) {
    throw std::invalid_argument("Missing XYZ fields");
  }
  std::vector<Point> result;
  result.reserve(size_t(cloud.width) * cloud.height);
  for (size_t row = 0; row < cloud.height; ++row) {
    for (size_t col = 0; col < cloud.width; ++col) {
      std::array<float, 3> p;
      const auto * data = cloud.data.data() + row * cloud.row_step + col * cloud.point_step;
      for (size_t i = 0; i < 3; ++i) {
        std::memcpy(&p[i], data + offsets[i], 4);
      }
      if (std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2])) {
        result.push_back({p[0], p[1], p[2]});
      }
    }
  }
  return result;
}
}
