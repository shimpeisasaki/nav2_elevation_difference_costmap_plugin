// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include "nav2_elevation_difference_costmap_plugin/terrain_model.hpp"
#include "nav2_elevation_difference_costmap_plugin/point_cloud.hpp"
#include <cmath>
#include <cstring>

namespace terrain = nav2_elevation_difference_costmap_plugin;
using terrain::Point;
std::vector<Point> scene(double sx = 0., double sy = 0., double step = 0., double edge = 0.)
{
  std::vector<Point> points;
  for (int y = -5; y <= 5; ++y) {
    for (int x = -6; x <= 6; ++x) {
      for (double dx:{.01, .03, .05, .07, .09}) {
        for (double dy:{.01, .03, .05, .07, .09}) {
          const double wx = x * .1 + dx, wy = y * .1 + dy;
          points.push_back({wx, wy, sx * wx + sy * wy + (wx >= edge ? step : 0.)});
        }
      }
    }
  }
  return points;
}
terrain::TerrainOptions options()
{terrain::TerrainOptions o; o.accumulation_frames = 1; return o;}

TEST(HeightStatistics, RejectsSmallSamplesAndTrimsSparseExtremeReturns)
{
  std::vector<Point> six(6, {.04, .07, 0.});
  six.front().z = -.06; six.back().z = .08;
  EXPECT_FALSE(terrain::heightStatistics(six));
  const auto comparison = terrain::heightStatistics(six, 6);
  ASSERT_TRUE(comparison);
  EXPECT_DOUBLE_EQ(comparison->upper - comparison->lower, .14);
  std::vector<Point> points(18, {.04, .07, 0.});
  points.push_back({99., 99., -.06});
  points.push_back({-99., -99., .08});
  const auto stats = terrain::heightStatistics(points);
  ASSERT_TRUE(stats); EXPECT_EQ(stats->count, 20u);
  EXPECT_DOUBLE_EQ(stats->lower, 0.); EXPECT_DOUBLE_EQ(stats->median, 0.);
  EXPECT_DOUBLE_EQ(stats->upper, 0.);
  EXPECT_NEAR(stats->centroid.x, .04, 1e-12);
  EXPECT_NEAR(stats->centroid.y, .07, 1e-12);
  EXPECT_DOUBLE_EQ(stats->centroid.z, 0.);
}
TEST(HeightStatistics, CentroidUsesTheSameQuantilePointsAndMedianStaysSeparate)
{
  std::vector<Point> points;
  for (int i = 0; i < 20; ++i) {
    points.push_back({i * .002, i * .003, i * .001});
  }
  auto stats = terrain::heightStatistics(points);
  ASSERT_TRUE(stats);
  EXPECT_NEAR(stats->lower, .001, 1e-12);
  EXPECT_NEAR(stats->median, .010, 1e-12);
  EXPECT_NEAR(stats->upper, .017, 1e-12);
  EXPECT_NEAR(stats->centroid.x, .018, 1e-12);
  EXPECT_NEAR(stats->centroid.y, .027, 1e-12);
  EXPECT_NEAR(stats->centroid.z, .009, 1e-12);
  points.push_back({NAN, 0., 0.});
  auto filtered = terrain::heightStatistics(points);
  ASSERT_TRUE(filtered); EXPECT_EQ(filtered->count, 20u);
}
TEST(TerrainModel, SixReturnsPerCellNeedDistinctInputsToReachDefaultMinimum)
{
  std::vector<Point> sparse;
  for (int y = -5; y <= 5; ++y) {
    for (int x = -6; x <= 6; ++x) {
      for (int i = 0; i < 6; ++i) {
        sparse.push_back({x * .1 + .05, y * .1 + .05, 0.});
      }
    }
  }
  terrain::TerrainModel model;
  for (int frame = 1; frame <= 3; ++frame) {
    ASSERT_TRUE(model.observe(sparse, frame));
    EXPECT_EQ(model.at({0, 0}, frame).cost, 255);
    EXPECT_FALSE(model.observe(sparse, frame));
  }
  ASSERT_TRUE(model.observe(sparse, 4.));
  EXPECT_EQ(model.at({0, 0}, 4.).cost, 0);
}
TEST(TerrainModel, RetainsMeasuredGroundAndLowObstacleBeyondWindowAndLocalLifetime)
{
  auto o = options(); o.retain_global_evidence = true;
  terrain::TerrainModel model(o);
  auto points = scene();
  for (auto & p : points) {
    if (p.x > 0 && p.x < .1 && p.y > 0 && p.y < .1) {p.z = .06;}
  }
  ASSERT_TRUE(model.observe(points, 1.));
  EXPECT_EQ(model.at({0, 0}, 1.).cost, 254);
  EXPECT_EQ(model.at({0, 0}, 1., terrain::EvidenceView::Retained).cost, 254);
  ASSERT_TRUE(model.observe({}, 100.));
  EXPECT_EQ(model.at({-3, 0}, 100.).cost, 255);
  EXPECT_EQ(model.at({-3, 0}, 100., terrain::EvidenceView::Retained).cost, 0);
  EXPECT_EQ(model.at({0, 0}, 100., terrain::EvidenceView::Retained).cost, 254);
  terrain::GridProjection identity({1, 0, 0, 0, 1, 0});
  EXPECT_EQ(
    model.area(
      identity.sourceQuad(
        -.3, 0.,
        .1), 100., terrain::EvidenceView::Retained).cost, 0);
  EXPECT_EQ(
    model.area(
      identity.sourceQuad(
        0., 0.,
        .1), 100., terrain::EvidenceView::Retained).cost, 254);
  ASSERT_TRUE(model.observe(scene(), 101.));
  EXPECT_EQ(model.at({0, 0}, 101., terrain::EvidenceView::Retained).cost, 0);
  ASSERT_TRUE(model.observe(points, 102.));
  EXPECT_EQ(model.at({0, 0}, 102., terrain::EvidenceView::Retained).cost, 254);
  auto mixed = scene();
  for (int i = 0; i < 25; ++i) {
    mixed.push_back({.05, .05, .12});
  }
  ASSERT_TRUE(model.observe(mixed, 103.));
  EXPECT_EQ(model.at({0, 0}, 103.).reason, 30);
  EXPECT_EQ(model.at({0, 0}, 103., terrain::EvidenceView::Retained).cost, 254);
  model.reset(); EXPECT_EQ(model.retainedSize(), 0u);
  EXPECT_EQ(model.at({0, 0}, 103., terrain::EvidenceView::Retained).cost, 255);
}
TEST(TerrainModel, ThreeCentimeterLowObstacleDoesNotBiasItsOwnReferencePlane)
{
  auto o = options(); o.retain_global_evidence = true;
  terrain::TerrainModel model(o);
  auto points = scene();
  for (auto & p : points) {
    if (p.x > 0 && p.x < .1 && p.y > 0 && p.y < .1) {p.z = .03;}
  }
  ASSERT_TRUE(model.observe(points, 1.));
  EXPECT_EQ(model.at({0, 0}, 1.).cost, 254);
  EXPECT_EQ(model.at({0, 0}, 1., terrain::EvidenceView::Retained).cost, 254);
}
TEST(TerrainModel, MixedHeightsAloneDoNotBecomeRetainedObstacles)
{
  auto o = options(); o.retain_global_evidence = true;
  terrain::TerrainModel model(o);
  auto points = scene();
  // Two substantial height groups: do not pretend the extra group is a lone
  // outlier or promote the mixed cell to a confirmed step via neighboring cells.
  for (int i = 0; i < 25; ++i) {
    points.push_back({.05, .05, .12});
  }
  ASSERT_TRUE(model.observe(points, 1.));
  EXPECT_EQ(model.at({0, 0}, 1.).cost, 254);
  EXPECT_EQ(model.at({0, 0}, 1.).reason, 30);
  EXPECT_EQ(model.at({0, 0}, 1., terrain::EvidenceView::Retained).cost, 255);
  ASSERT_TRUE(model.observe({}, 40.));
  EXPECT_EQ(model.at({0, 0}, 40.).cost, 255);
  ASSERT_TRUE(model.observe(scene(), 41.));
  EXPECT_EQ(model.at({0, 0}, 41., terrain::EvidenceView::Retained).cost, 0);
  ASSERT_TRUE(model.observe(points, 42.));
  EXPECT_EQ(model.at({0, 0}, 42., terrain::EvidenceView::Retained).cost, 0);
}
TEST(TerrainModel, LocalLayerDoesNotAllocateExplorationMemory)
{
  terrain::TerrainModel model(options()); model.observe(scene(), 1.);
  EXPECT_EQ(model.retainedSize(), 0u);
  EXPECT_EQ(model.at({0, 0}, 1., terrain::EvidenceView::Retained).cost, 255);
}
TEST(TerrainModel, FlatAndSixDegreeRampsRemainSafe)
{
  for (double angle:{0., 6., -6.}) {
    for (double direction:{0., .7853981633974483, 1.5707963267948966}) {
      terrain::TerrainModel model(options());
      const double grade = std::tan(angle * std::acos(-1.) / 180.);
      ASSERT_TRUE(
        model.observe(
          scene(grade * std::cos(direction), grade * std::sin(direction)),
          1.));
      EXPECT_EQ(model.at({0, 0}, 1.).cost, 0) << angle << " " << direction;
    }
  }
}
TEST(TerrainModel, UnevenWithinCellSamplingDoesNotInventGrade)
{
  const double grade = std::tan(6. * std::acos(-1.) / 180.);
  auto points = scene(grade);
  std::vector<Point> biased;
  for (const auto & p : points) {
    const int ix = int(std::floor(p.x / .1));
    const double offset = p.x - ix * .1;
    if ((ix % 2 == 0 && offset < .06) || (ix % 2 != 0 && offset > .06)) {
      biased.push_back(p); biased.push_back(p);
    }
  }
  terrain::TerrainModel model(options()); model.observe(biased, 1.);
  EXPECT_EQ(model.at({0, 0}, 1.).cost, 0);
  EXPECT_EQ(model.at({1, 0}, 1.).cost, 0);
}
TEST(TerrainModel, ExcessiveSlopeIsHazardInAllDirections)
{
  for (double direction:{0., .7853981633974483, 1.5707963267948966}) {
    terrain::TerrainModel model(options());
    const double grade = std::tan(9. * std::acos(-1.) / 180.);
    model.observe(scene(grade * std::cos(direction), grade * std::sin(direction)), 1.);
    EXPECT_EQ(model.at({0, 0}, 1.).cost, 254);
  }
}
TEST(TerrainModel, CellAlignedStepsAndDropsAlsoOnDownhillRamp)
{
  for (double grade:{0., -std::tan(6. * std::acos(-1.) / 180.)}) {
    for (double step:{.03, -.03}) {
      terrain::TerrainModel model(options()); model.observe(scene(grade, 0, step), 1.);
      EXPECT_EQ(model.at({-1, 0}, 1.).cost, 254) << grade << " " << step;
      EXPECT_EQ(model.at({0, 0}, 1.).cost, 254) << grade << " " << step;
    }
  }
}
TEST(TerrainModel, StepDetectionDoesNotDependOnCellBoundaryAlignment)
{
  for (double edge : {.015, .025, .065, .085}) {
    for (double step : {-.03, .03}) {
      terrain::TerrainModel model(options());
      model.observe(scene(0., 0., step, edge), 1.);
      EXPECT_EQ(model.at({0, 0}, 1.).cost, 254) << edge << " " << step;
    }
  }
}
TEST(TerrainModel, OneOutlierDoesNotBlockOtherwiseFlatSurface)
{
  auto points = scene();
  for (int i = 0; i < 20; ++i) {
    points.push_back({.05, .05, 0.});
  }
  points.push_back({.05, .05, .02});
  terrain::TerrainModel model(options());model.observe(points, 1.);
  EXPECT_EQ(model.at({0, 0}, 1.).cost, 0);
}
TEST(TerrainModel, MixedSurfaceIsHazard)
{
  auto points = scene();
  for (int i = 0; i < 12; ++i) {
    points.push_back({.05, .05, .05});
  }
  terrain::TerrainModel model(options());model.observe(points, 1.);
  EXPECT_EQ(model.at({0, 0}, 1.).cost, 254);
}
TEST(TerrainModel, SparseOrCollinearSupportDoesNotEstablishGround)
{
  terrain::TerrainModel model;
  model.observe({{.05, .05, 0}}, 1.);
  EXPECT_EQ(model.at({0, 0}, 1.).cost, 255);
  std::vector<Point> line;
  for (int x = -5; x < 5; ++x) {
    for (int i = 0; i < 20; ++i) {
      line.push_back({x *.1 + .05, .05, 0.});
    }
  }
  model.observe(line, 2.);
  EXPECT_EQ(model.at({0, 0}, 2.).cost, 255);
}
TEST(TerrainModel, WindowAccumulatesOnlyDistinctInputs)
{
  terrain::TerrainOptions o; o.min_points = 50; o.accumulation_frames = 2;
  terrain::TerrainModel model(o);
  model.observe(scene(), 1.);
  EXPECT_EQ(model.at({0, 0}, 1.).cost, 255);
  EXPECT_FALSE(model.observe(scene(), 1.));
  EXPECT_EQ(model.at({0, 0}, 1.).cost, 255);
  EXPECT_TRUE(model.observe(scene(), 2.));
  EXPECT_EQ(model.at({0, 0}, 2.).cost, 0);
  EXPECT_FALSE(model.observe(scene(), 1.5));
}
TEST(TerrainModel, MissingDataCannotClearDangerAndSafeEvidenceExpires)
{
  terrain::TerrainModel model(options());
  model.observe(scene(0, 0, .05), 1.);
  model.observe({}, 50.);
  EXPECT_EQ(model.at({0, 0}, 50.).cost, 254);
  model.observe(scene(), 51.);
  EXPECT_EQ(model.at({0, 0}, 51.).cost, 0);
  EXPECT_EQ(model.at({0, 0}, 82.).cost, 255);
  model.reset(); EXPECT_EQ(model.at({0, 0}, 82.).cost, 255);
}
TEST(TerrainModel, CellAreaRejectsUnknownAndRetainsHazardsAtEdges)
{
  terrain::TerrainModel model(options()); model.observe(scene(0, 0, .05), 1.);
  terrain::GridProjection identity({1, 0, 0, 0, 1, 0});
  EXPECT_EQ(model.area(identity.sourceQuad(-.2, 0, .1), 1.).cost, 0);
  EXPECT_EQ(model.area(identity.sourceQuad(-.15, 0, .1), 1.).cost, 254);
  EXPECT_EQ(model.area(identity.sourceQuad(1., 0, .1), 1.).cost, 255);
  terrain::GridProjection shift({1, 0, 2., 0, 1, -1.});
  EXPECT_EQ(model.area(shift.sourceQuad(1.85, -1., .1), 1.).cost, 254);
}
TEST(TerrainModel, InvalidParametersAndNonfiniteInput)
{
  auto o = options();o.resolution = 0;EXPECT_THROW(
    terrain::TerrainModel model(
      o), std::invalid_argument);
  EXPECT_THROW(terrain::GridProjection({0, 0, 0, 0, 0, 0}), std::invalid_argument);
  terrain::TerrainModel model;
  EXPECT_FALSE(model.observe({}, NAN));
  EXPECT_TRUE(model.observe({{NAN, 0, 0}, {0, INFINITY, 0}, {0, 0, NAN}}, 1.));
  EXPECT_EQ(model.size(), 0u);
}
TEST(PointCloud, ValidatesLayoutAndHandlesOrganizedRowPadding)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.width = 2;cloud.height = 2;cloud.point_step = 16;cloud.row_step = 40;cloud.data.resize(80);
  for (size_t i = 0; i < 3; ++i) {
    sensor_msgs::msg::PointField f;f.name = std::array<const char *, 3>{"x", "y", "z"}[i];
    f.offset = 4 * i;f.datatype = sensor_msgs::msg::PointField::FLOAT32;f.count = 1;
    cloud.fields.push_back(f);
  }
  for (size_t row = 0; row < 2; ++row) {
    for (size_t col = 0; col < 2; ++col) {
      const float x = float(row * 2 + col), y = 1, z = 2;
      auto * p = cloud.data.data() + row * 40 + col * 16;
      std::memcpy(p, &x, 4);std::memcpy(p + 4, &y, 4);std::memcpy(p + 8, &z, 4);
    }
  }
  auto points = terrain::readPoints(cloud);ASSERT_EQ(points.size(), 4u);EXPECT_EQ(
    points.back().x, 3.);
  cloud.data.resize(60);EXPECT_THROW(terrain::readPoints(cloud), std::invalid_argument);
  cloud.data.resize(80);cloud.fields[0].datatype = 8;
  EXPECT_THROW(terrain::readPoints(cloud), std::invalid_argument);
}
