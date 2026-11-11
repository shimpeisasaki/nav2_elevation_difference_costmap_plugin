// BSD 3-Clause License
//
// Copyright (c) 2026 Shuma Suzuki
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "nav2_elevation_difference_costmap_plugin/elevation_layer.hpp"
#include "nav2_elevation_difference_costmap_plugin/point_cloud.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include <chrono>

namespace nav2_elevation_difference_costmap_plugin
{
void ElevationLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {throw std::runtime_error("ElevationLayer node expired");}
  const auto parameter = [&](const std::string & key, auto fallback) {
      declareParameter(key, rclcpp::ParameterValue(fallback));
      return node->get_parameter(name_ + "." + key).get_value<decltype(fallback)>();
    };
  enabled_ = parameter("enabled", true);
  auto topic = parameter("topic", std::string("/livox/lidar"));
  evidence_frame_ = parameter("evidence_frame", std::string("odom"));
  max_age_ = parameter("max_cloud_age", .8);
  TerrainOptions options;
  options.resolution = parameter("evidence_resolution", .1);
  options.step_height = parameter("step_height", .03);
  options.max_spread = parameter("max_height_spread", .03);
  options.max_slope_deg = parameter("max_slope_deg", 7.);
  options.safe_lifetime = parameter("safe_lifetime", 30.);
  const int64_t min_points = parameter("min_points", int64_t(6));
  const int64_t frames = parameter("accumulation_frames", int64_t(5));
  if (min_points < 3 || min_points > 10000 || frames < 1 || frames > 20 ||
    !std::isfinite(max_age_) || max_age_ <= 0 || max_age_ > 2. || evidence_frame_.empty() ||
    topic.empty())
  {
    throw std::invalid_argument("Invalid elevation layer parameters");
  }
  options.min_points = unsigned(min_points); options.accumulation_frames = unsigned(frames);
  model_ = std::make_unique<TerrainModel>(options);
  beam_enabled_ = parameter("beam_filter.enabled", false);
  beam_negative_ = parameter("beam_filter.negative", true);
  const std::array<std::string, 6> keys{"xmin", "xmax", "ymin", "ymax", "zmin", "zmax"};
  const std::array<double, 6> defaults{-.70, .70, -.45, 1., -2., 2.};
  for (size_t i = 0; i < 6; ++i) {
    beam_[i] = parameter("beam_filter." + keys[i], defaults[i]);
  }
  for (size_t i = 0; i < 6; i += 2) {
    if (!std::isfinite(beam_[i]) || !std::isfinite(beam_[i + 1]) || beam_[i] >= beam_[i + 1]) {
      throw std::invalid_argument("Invalid beam filter box");
    }
  }
  default_value_ = nav2_costmap_2d::NO_INFORMATION;
  matchSize();
  subscription_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      cloud_ = msg; received_ = std::chrono::steady_clock::now();
    });
  current_ = false;
}
void ElevationLayer::updateBounds(
  double, double, double, double * xmin, double * ymin,
  double * xmax, double * ymax)
{
  if (!enabled_) {return;}
  auto * master = layered_costmap_->getCostmap();
  *xmin = std::min(*xmin, master->getOriginX()); *ymin = std::min(*ymin, master->getOriginY());
  *xmax =
    std::max(*xmax, master->getOriginX() + master->getSizeInCellsX() * master->getResolution());
  *ymax =
    std::max(*ymax, master->getOriginY() + master->getSizeInCellsY() * master->getResolution());
}
void ElevationLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master, int min_i, int min_j,
  int max_i, int max_j)
{
  if (!enabled_) {current_ = true; return;}
  auto node = node_.lock(); if (!node) {return;}
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
  double received_age;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    cloud = cloud_;
    received_age =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - received_).count();
  }
  std::lock_guard<std::mutex> model_lock(model_mutex_);
  current_ = false;
  if (!cloud || cloud->header.frame_id.empty()) {return;}
  const rclcpp::Time stamp(cloud->header.stamp, node->get_clock()->get_clock_type());
  const double age = (node->now() - stamp).seconds();
  if (stamp.nanoseconds() <= 0 || age < -.1 || age > max_age_ || received_age > max_age_) {return;}
  try {
    if (stamp.seconds() < model_->lastStamp()) {return;}
    if (stamp.seconds() > model_->lastStamp()) {
      tf2::Transform transform;
      tf2::fromMsg(
        tf_->lookupTransform(
          evidence_frame_, cloud->header.frame_id,
          stamp).transform, transform);
      std::vector<Point> points;
      for (const auto & p:readPoints(*cloud)) {
        const bool inside = p.x >= beam_[0] && p.x <= beam_[1] && p.y >= beam_[2] &&
          p.y <= beam_[3] &&
          p.z >= beam_[4] && p.z <= beam_[5];
        if (beam_enabled_ && (beam_negative_ ? inside : !inside)) {continue;}
        auto q = transform * tf2::Vector3(p.x, p.y, p.z); points.push_back({q.x(), q.y(), q.z()});
      }
      if (!model_->observe(points, stamp.seconds())) {return;}
    }
    // Reprojection may follow SLAM corrections; observations stay fixed in odom.
    tf2::Transform transform;
    tf2::fromMsg(
      tf_->lookupTransform(
        layered_costmap_->getGlobalFrameID(), evidence_frame_, stamp).transform, transform);
    const auto & b = transform.getBasis(); const auto & t = transform.getOrigin();
    GridProjection projection({b[0][0], b[0][1], t.x(), b[1][0], b[1][1], t.y()});
    resizeMap(
      master.getSizeInCellsX(), master.getSizeInCellsY(),
      master.getResolution(), master.getOriginX(), master.getOriginY());
    for (int y = std::max(0, min_j); y < std::min(int(getSizeInCellsY()), max_j); ++y) {
      for (int x = std::max(0, min_i); x < std::min(int(getSizeInCellsX()), max_i); ++x) {
        const double wx = getOriginX() + x * getResolution(),
          wy = getOriginY() + y * getResolution();
        setCost(
          x, y,
          model_->area(
            projection.sourceQuad(wx, wy, getResolution()),
            node->now().seconds()).cost);
      }
    }
    updateWithMax(master, min_i, min_j, max_i, max_j);
    current_ = true;
  } catch (const std::exception & error) {
    RCLCPP_WARN_THROTTLE(
      logger_,
      *node->get_clock(), 2000, "Terrain update rejected: %s", error.what());
  }
}
void ElevationLayer::reset()
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  std::lock_guard<std::mutex> model_lock(model_mutex_);
  cloud_.reset(); model_->reset(); resetMaps(); current_ = false;
}
}
PLUGINLIB_EXPORT_CLASS(
  nav2_elevation_difference_costmap_plugin::ElevationLayer,
  nav2_costmap_2d::Layer)
