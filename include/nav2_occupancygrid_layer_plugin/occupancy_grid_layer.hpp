#ifndef OCCUPANCY_GRID_LAYER_HPP_
#define OCCUPANCY_GRID_LAYER_HPP_

#include <mutex>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include "nav2_costmap_2d/footprint.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace nav2_occupancygrid_layer_plugin
{

class OccupancyGridLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  OccupancyGridLayer();
  virtual ~OccupancyGridLayer() = default;

  virtual void onInitialize() override;
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double* min_x, double* min_y, double* max_x, double* max_y) override;

  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  virtual void reset() override {}

  virtual bool isClearable() override { return false; }


private:
  void incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  nav_msgs::msg::OccupancyGrid last_map_;
  std::mutex mutex_;
  bool has_map_;
  bool footprint_clearing_enabled_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

};

}  // namespace nav2_occupancygrid_layer_plugin

#endif  // OCCUPANCY_GRID_LAYER_HPP_
