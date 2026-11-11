// SPDX-License-Identifier: BSD-3-Clause
#include "nav2_elevation_difference_costmap_plugin/terrain_model.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace nav2_elevation_difference_costmap_plugin
{
std::optional<HeightStatistics> heightStatistics(std::vector<Point> points, unsigned min_points)
{
  points.erase(
    std::remove_if(
      points.begin(), points.end(), [](const Point & p) {
        return !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z);
      }), points.end());
  const size_t n = points.size();
  if (min_points < 3 || n < min_points) {return std::nullopt;}
  std::sort(
    points.begin(), points.end(), [](const Point & a, const Point & b) {
      return a.z < b.z;
    });
  const size_t lo = size_t(std::ceil(.1 * n)) - 1, hi = size_t(std::ceil(.9 * n)) - 1;
  Point center{0, 0, 0};
  for (size_t i = lo; i <= hi; ++i) {
    center.x += points[i].x; center.y += points[i].y; center.z += points[i].z;
  }
  const double count = hi - lo + 1;
  return HeightStatistics{{center.x / count, center.y / count, center.z / count},
    points[lo].z, points[n / 2].z, points[hi].z, n};
}
GridProjection::GridProjection(const std::array<double, 6> & t)
{
  const double det = t[0] * t[4] - t[1] * t[3];
  if (!std::all_of(
      t.begin(), t.end(),
      [](double v) {return std::isfinite(v);}) || std::abs(det) < .5)
  {
    throw std::invalid_argument("Invalid terrain projection");
  }
  inverse_ = {t[4] / det, -t[1] / det, (t[1] * t[5] - t[4] * t[2]) / det,
    -t[3] / det, t[0] / det, (t[3] * t[2] - t[0] * t[5]) / det};
}
std::array<std::array<double, 2>, 4> GridProjection::sourceQuad(double x, double y, double r) const
{
  std::array<std::array<double, 2>, 4> q{{{x, y}, {x + r, y}, {x + r, y + r}, {x, y + r}}};
  for (auto & p:q) {
    const double px = p[0], py = p[1];
    p =
    {inverse_[0] * px + inverse_[1] * py + inverse_[2],
      inverse_[3] * px + inverse_[4] * py + inverse_[5]};
  }
  return q;
}
namespace
{
struct Surface {double x, y, z, spread; unsigned count;};
using Surfaces = std::unordered_map<Key, Surface, KeyHash>;
struct Plane {double a, b, c; double z(double x, double y) const {return a * x + b * y + c;}};
// Centered least squares avoids loss of precision at large odom coordinates.
bool plane(const std::vector<Surface> & rows, Plane & out)
{
  if (rows.size() < 6) {return false;}
  double x = 0, y = 0, z = 0;
  for (const auto & p : rows) {
    x += p.x; y += p.y; z += p.z;
  }
  x /= rows.size(); y /= rows.size(); z /= rows.size();
  double xx = 0, yy = 0, xy = 0, xz = 0, yz = 0;
  for (const auto & p : rows) {
    const double dx = p.x - x, dy = p.y - y, dz = p.z - z;
    xx += dx * dx; yy += dy * dy; xy += dx * dy; xz += dx * dz; yz += dy * dz;
  }
  const double determinant = xx * yy - xy * xy;
  if (determinant <= 1e-8 || determinant <= 1e-4 * (xx + yy) * (xx + yy)) {return false;}
  out.a = (xz * yy - yz * xy) / determinant;
  out.b = (yz * xx - xz * xy) / determinant;
  out.c = z - out.a * x - out.b * y;
  return true;
}
bool surfacePlane(const std::vector<Surface> & rows, double max_residual, Plane & out)
{
  if (!plane(rows, out)) {return false;}
  std::vector<Surface> inliers;
  for (const auto & p : rows) {
    if (std::abs(p.z - out.z(p.x, p.y)) <= max_residual) {inliers.push_back(p);}
  }
  return inliers.size() == rows.size() || plane(inliers, out);
}
bool retainable(const TerrainCell & cell)
{
  return (cell.cost == 0 && cell.reason == 20) ||
         (cell.cost == 254 && (cell.reason == 40 || cell.reason == 50 || cell.reason == 60));
}
bool overlap(const std::array<std::array<double, 2>, 4> & q, double x, double y, double r)
{
  for (int axis = 0; axis < 6; ++axis) {
    double ax = axis == 0 ? 1. : 0., ay = axis == 1 ? 1. : 0.;
    if (axis >= 2) {
      const auto & a = q[axis - 2]; const auto & b = q[(axis - 1) % 4];
      ax = -(b[1] - a[1]); ay = b[0] - a[0];
    }
    double lo = INFINITY, hi = -INFINITY;
    for (const auto & p:q) {
      const double v = ax * p[0] + ay * p[1]; lo = std::min(lo, v); hi = std::max(hi, v);
    }
    const double center = ax * (x + r / 2) + ay * (y + r / 2),
      extent = r / 2 * (std::abs(ax) + std::abs(ay));
    if (lo >= center + extent - 1e-10 || hi <= center - extent + 1e-10) {return false;}
  }
  return true;
}
}
TerrainModel::TerrainModel(TerrainOptions options)
: options_(options)
{
  if (!std::isfinite(options.resolution) || options.resolution < .02 || options.resolution > 1. ||
    !std::isfinite(options.step_height) || options.step_height <= 0 ||
    !std::isfinite(options.max_spread) || options.max_spread <= 0 ||
    !std::isfinite(options.max_slope_deg) || options.max_slope_deg <= 0 ||
    options.max_slope_deg >= 60 ||
    !std::isfinite(options.safe_lifetime) || options.safe_lifetime <= 0 ||
    options.min_points < 3 || options.min_points > 10000 || options.accumulation_frames < 1 ||
    options.accumulation_frames > 20) {throw std::invalid_argument("Invalid terrain options");}
}
void TerrainModel::reset() {frames_.clear(); cells_.clear(); retained_.clear(); last_stamp_ = -1.;}
bool TerrainModel::observe(const std::vector<Point> & input, double stamp)
{
  if (!std::isfinite(stamp) || stamp <= last_stamp_ || stamp < 0 || input.size() > 1000000) {
    return false;
  }
  std::vector<Point> finite;
  for (const auto & p:input) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
      std::abs(p.x) < 1e5 && std::abs(p.y) < 1e5 && std::abs(p.z) < 1e4) {finite.push_back(p);}
  }
  // Empty input advances the observation window, but never proves free ground.
  frames_.push_back(std::move(finite));
  while (frames_.size() > options_.accumulation_frames) {frames_.pop_front();}
  const double r = options_.resolution;
  struct Bucket {std::vector<Point> points;};
  std::unordered_map<Key, Bucket, KeyHash> buckets;
  std::unordered_map<Key, bool, KeyHash> observed;
  for (const auto & p:frames_.back()) {
    observed[{int(std::floor(p.x / r)), int(std::floor(p.y / r))}] = true;
  }
  for (const auto & frame:frames_) {
    for (const auto & p:frame) {
      auto & b = buckets[{int(std::floor(p.x / r)), int(std::floor(p.y / r))}];
      b.points.push_back(p);
    }
  }
  Surfaces surfaces;
  for (auto & item:buckets) {
    const auto stats = heightStatistics(std::move(item.second.points), options_.min_points);
    if (!stats) {continue;}
    // XYZ must come from the same retained height interval, not the cell center.
    surfaces[item.first] = {stats->centroid.x, stats->centroid.y, stats->centroid.z,
      stats->upper - stats->lower, unsigned(stats->count)};
  }
  const auto neighborhood = [&](Key key, int axis, int side) {
      std::vector<Surface> rows;
      for (int dy = -2; dy <= 2; ++dy) {
        for (int dx = -2; dx <= 2; ++dx) {
          const int offset = axis == 0 ? dx : dy;
          if (side < 0 && offset > 0) {continue;}
          if (side > 0 && offset < 1) {continue;}
          auto found = surfaces.find({key.x + dx, key.y + dy});
          if (found != surfaces.end() && found->second.spread < options_.max_spread) {
            rows.push_back(found->second);
          }
        }
      }
      return rows;
    };
  std::unordered_map<Key, TerrainCell, KeyHash> evaluated;
  for (const auto & item:observed) {
    const Key key = item.first;
    auto found = surfaces.find(key);
    if (found == surfaces.end()) {continue;}
    const auto & s = found->second;
    TerrainCell cell{255, 10, stamp};
    if (s.spread >= options_.max_spread - 1e-8) {cell = {254, 30, stamp};} else {
      Plane fitted;
      auto neighbors = neighborhood(key, -1, 0);
      neighbors.erase(
        std::remove_if(
          neighbors.begin(), neighbors.end(), [&s](const Surface & p) {
            return p.x == s.x && p.y == s.y;
          }), neighbors.end());
      // When enough surrounding cells exist, the tested low obstacle must not
      // pull its own reference ground plane upward and hide a threshold step.
      if (neighbors.size() < 6) {neighbors = neighborhood(key, -1, 0);}
      if (surfacePlane(neighbors, options_.max_spread, fitted)) {
        const double residual = s.z - fitted.z(s.x, s.y);
        if (std::abs(residual) >= options_.step_height - 1e-8) {
          cell = {254, uint8_t(residual > 0 ? 40 : 50), stamp};
        } else if (std::hypot(
            fitted.a,
            fitted.b) > std::tan(options_.max_slope_deg * std::acos(-1.) / 180.) + 1e-8)
        {
          cell = {254, 60, stamp};
        } else {cell = {0, 20, stamp};}
      }
    }
    evaluated[key] = cell;
  }
  // Compare independent surfaces at their common edge, including steps on ramps.
  for (const auto & item:surfaces) {
    const auto key = item.first;
    for (int axis = 0; axis < 2; ++axis) {
      Key neighbor{key.x + (axis == 0), key.y + (axis == 1)};
      auto other = surfaces.find(neighbor);
      if (other == surfaces.end() || (!observed.count(key) && !observed.count(neighbor)) ||
        item.second.spread >= options_.max_spread - 1e-8 ||
        other->second.spread >= options_.max_spread - 1e-8)
      {
        continue;
      }
      double dz = other->second.z - item.second.z;
      Plane left, right;
      if (plane(neighborhood(key, axis, -1), left) && plane(neighborhood(key, axis, 1), right)) {
        const double x = (key.x + (axis == 0 ? 1. : .5)) * r;
        const double y = (key.y + (axis == 1 ? 1. : .5)) * r;
        dz = right.z(x, y) - left.z(x, y);
      }
      if (std::abs(dz) >= options_.step_height - 1e-8) {
        evaluated[key] = {254, uint8_t(dz > 0 ? 40 : 50), stamp};
        evaluated[neighbor] = {254, uint8_t(dz > 0 ? 40 : 50), stamp};
      }
    }
  }
  for (auto it = cells_.begin(); it != cells_.end(); ) {
    if ((it->second.cost == 0 || it->second.reason == 30) &&
      stamp - it->second.stamp > options_.safe_lifetime)
    {
      it = cells_.erase(it);
    } else {++it;}
  }
  size_t additions = 0;
  size_t retained_additions = 0;
  for (const auto & item:evaluated) {
    additions += item.second.cost != 255 && !cells_.count(item.first);
    retained_additions += options_.retain_global_evidence && retainable(item.second) &&
      !retained_.count(item.first);
  }
  if (cells_.size() + additions > 250000 || retained_.size() + retained_additions > 250000) {
    throw std::length_error("Terrain evidence limit; explicit reset required");
  }
  // Insufficient observations cannot clear a previously measured hazard.
  for (const auto & item:evaluated) {
    if (item.second.cost != 255) {cells_[item.first] = item.second;}
    if (options_.retain_global_evidence && retainable(item.second)) {
      retained_[item.first] = item.second;
    }
  }
  last_stamp_ = stamp;
  return true;
}
TerrainCell TerrainModel::at(Key key, double now, EvidenceView view) const
{
  const auto & source = view == EvidenceView::Retained ? retained_ : cells_;
  auto it = source.find(key);
  if (it == source.end() ||
    (view == EvidenceView::Local && (it->second.cost == 0 || it->second.reason == 30) &&
    now - it->second.stamp > options_.safe_lifetime)) {return {};}
  return it->second;
}
TerrainCell TerrainModel::area(
  const std::array<std::array<double, 2>, 4> & q, double now,
  EvidenceView view) const
{
  double xmin = INFINITY, ymin = INFINITY, xmax = -INFINITY, ymax = -INFINITY;
  for (const auto & p:q) {
    if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || std::abs(p[0]) > 1e5 ||
      std::abs(p[1]) > 1e5) {return {};}
    xmin = std::min(xmin, p[0]); xmax = std::max(xmax, p[0]);
    ymin = std::min(ymin, p[1]); ymax = std::max(ymax, p[1]);
  }
  const double r = options_.resolution;
  if ((xmax - xmin) / r > 100 || (ymax - ymin) / r > 100) {return {};}
  TerrainCell result{0, 20, last_stamp_}; bool any = false, unknown = false;
  for (int y = int(std::floor(ymin / r)); y <= int(std::floor(ymax / r)); ++y) {
    for (int x = int(std::floor(xmin / r)); x <= int(std::floor(xmax / r)); ++x) {
      if (!overlap(q, x * r, y * r, r)) {continue;}
      any = true; auto value = at({x, y}, now, view);
      if (value.cost == 254) {return value;}
      unknown |= value.cost == 255;
    }
  }
  return !any || unknown ? TerrainCell{} : result;
}
}  // namespace nav2_elevation_difference_costmap_plugin
