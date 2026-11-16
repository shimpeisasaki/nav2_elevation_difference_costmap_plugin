// SPDX-License-Identifier: BSD-3-Clause
#include <gtest/gtest.h>
#include "nav2_elevation_difference_costmap_plugin/elevation_layer.hpp"
#include <nav2_util/lifecycle_node.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <thread>

TEST(ElevationLayer, PreservesExistingObstaclesAndRejectsStaleObservations)
{
  int argc = 0;rclcpp::init(argc, nullptr);
  {
    auto node = std::make_shared<nav2_util::LifecycleNode>("terrain_layer_contract");
    node->declare_parameter("terrain.topic", std::string("/offline_layer/points"));
    node->declare_parameter("terrain.accumulation_frames", int64_t(1));
    nav2_costmap_2d::LayeredCostmap map("map", false, true);map.resizeMap(80, 80, .1, -4, -4);
    auto * master = map.getCostmap();
    tf2_ros::Buffer tf(node->get_clock());
    tf.setUsingDedicatedThread(true);  // This fixture injects transforms synchronously.
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";transform.child_frame_id = "odom";
    transform.transform.rotation.w = 1.;
    ASSERT_TRUE(tf.setTransform(transform, "fixture", true));
    transform.header.frame_id = "odom";transform.child_frame_id = "sensor";
    ASSERT_TRUE(tf.setTransform(transform, "fixture", true));
    nav2_elevation_difference_costmap_plugin::ElevationLayer layer;
    layer.initialize(
      &map, "terrain", &tf, node,
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
    auto publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/offline_layer/points",
      rclcpp::SensorDataQoS());
    publisher->on_activate();
    rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(
      node->get_node_base_interface());
    const auto spin = [&]() {
        for (int i = 0; i < 12; ++i) {
          executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      };
    spin();
    const auto send = [&](double step, double age = 0.) {
        sensor_msgs::msg::PointCloud2 cloud;cloud.header.frame_id = "sensor";
        cloud.header.stamp = node->now() - rclcpp::Duration::from_seconds(age);
        sensor_msgs::PointCloud2Modifier modifier(cloud);modifier.setPointCloud2FieldsByString(
          1,
          "xyz");
        modifier.resize(13 * 11 * 25);
        sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
        for (int iy = -5; iy <= 5; ++iy) {
          for (int ix = -6; ix <= 6; ++ix) {
            for (double dx:{.01, .03, .05, .07, .09}) {
              for (double dy:{.01, .03, .05, .07, .09}) {
                *x = ix * .1 + dx;*y = iy * .1 + dy;
                *z = std::tan(6. * std::acos(-1.) / 180.) * (*x) + (ix >= 0 ? step : 0.);
                ++x;++y;++z;
              }
            }
          }
        }
        publisher->publish(cloud);spin();
      };
    const auto update = [&]() {layer.updateCosts(*master, 0, 0, 80, 80);};
    EXPECT_FALSE(layer.isCurrent());
    send(0.);
    master->setCost(40, 40, 254);update();
    EXPECT_TRUE(layer.isCurrent());
    EXPECT_EQ(master->getCost(40, 40), 254);  // A preceding wall must survive flat terrain.
    EXPECT_EQ(master->getCost(41, 40), 0);  // A supported six-degree ramp is traversable.
    EXPECT_EQ(master->getCost(60, 40), 255);
    master->resetMap(0, 0, 80, 80);send(.03);update();
    EXPECT_EQ(master->getCost(40, 40), 254);
    EXPECT_EQ(master->getCost(39, 40), 254);
    master->resetMap(0, 0, 80, 80);send(.05, 10.);update();
    EXPECT_FALSE(layer.isCurrent());
    EXPECT_EQ(master->getCost(40, 40), 255);
    layer.reset();EXPECT_FALSE(layer.isCurrent());
    executor.remove_node(node->get_node_base_interface());
  }
  rclcpp::shutdown();
}
