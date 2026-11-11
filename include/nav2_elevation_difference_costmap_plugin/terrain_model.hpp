// SPDX-License-Identifier: BSD-3-Clause
#pragma once
#include <array>
#include <cstdint>
#include <cstddef>
#include <deque>
#include <optional>
#include <unordered_map>
#include <vector>

namespace nav2_elevation_difference_costmap_plugin
{
struct Point {double x, y, z;};
struct Key
{
  int x, y;
  bool operator==(const Key & other) const {return x == other.x && y == other.y;}
};
struct KeyHash
{
  size_t operator()(const Key & k) const
  {return (uint64_t(uint32_t(k.x)) << 32) ^ uint32_t(k.y);}
};
struct TerrainOptions
{
  double resolution{.1}, step_height{.03}, max_spread{.03}, max_slope_deg{7.};
  double safe_lifetime{30.};
  unsigned min_points{20}, accumulation_frames{5};
  // Enable only for the exploration adapter, not the standalone local layer.
  bool retain_global_evidence{false};
};
struct HeightStatistics
{
  Point centroid;
  double lower, median, upper;
  size_t count;
};
// Nearest-rank p10/p90 and upper median; centroid uses those same p10..p90 points.
std::optional<HeightStatistics> heightStatistics(
  std::vector<Point> points,
  unsigned min_points = 20);
enum class EvidenceView {Local, Retained};
struct TerrainCell
{
  // Nav2 raw costs; reason codes agree with the published recovery legend.
  uint8_t cost{255}, reason{10};
  double stamp{0.};
};
// Affine XY projection of a source z=0 grid into the target map. Query whole
// target cells in the source grid, never interpolate a safe center over hazards.
class GridProjection
{
public:
  explicit GridProjection(const std::array<double, 6> & source_to_target);
  std::array<std::array<double, 2>, 4> sourceQuad(double x, double y, double r) const;

private:
  std::array<double, 6> inverse_;
};
// Sensor-independent evidence in one continuous, gravity-aligned frame (odom).
// Classification is height quantiles + neighborhood grade and edge difference.
class TerrainModel
{
public:
  explicit TerrainModel(TerrainOptions options = {});
  bool observe(const std::vector<Point> & points, double stamp);
  TerrainCell at(Key key, double now, EvidenceView view = EvidenceView::Local) const;
  // Conservative cell-area query: all overlapping source cells must be safe.
  TerrainCell area(
    const std::array<std::array<double, 2>, 4> & quad, double now,
    EvidenceView view = EvidenceView::Local) const;
  void reset();
  double lastStamp() const {return last_stamp_;}
  size_t size() const {return cells_.size();}
  size_t retainedSize() const {return retained_.size();}
  const TerrainOptions & options() const {return options_;}

private:
  TerrainOptions options_;
  double last_stamp_{-1.};
  std::deque<std::vector<Point>> frames_;
  std::unordered_map<Key, TerrainCell, KeyHash> cells_;
  // Measured ground and geometrically supported hazards. Never initial assumptions
  // or a mixed-height distribution alone; corrected by later sufficient evidence.
  std::unordered_map<Key, TerrainCell, KeyHash> retained_;
};
}  // namespace nav2_elevation_difference_costmap_plugin
