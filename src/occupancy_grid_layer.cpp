#include "../include/nav2_occupancygrid_layer_plugin/occupancy_grid_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_occupancygrid_layer_plugin
{

OccupancyGridLayer::OccupancyGridLayer() : has_map_(false)
{
}

void OccupancyGridLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in OccupancyGridLayer");
  }

  current_ = true;

  std::string topic = node->declare_parameter<std::string>(
    name_ + ".topic", "risk_grid");

  footprint_clearing_enabled_ = node->declare_parameter(
    name_ + ".footprint_clearing_enabled", true);
    
  sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic, rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&OccupancyGridLayer::incomingMap, this, std::placeholders::_1));

  RCLCPP_INFO(node->get_logger(), "OccupancyGridLayer subscribed to %s", topic.c_str());
}

void OccupancyGridLayer::incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_map_ = *msg;
  has_map_ = true;
}

void OccupancyGridLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!has_map_) return;

  std::lock_guard<std::mutex> lock(mutex_);
  double wx = last_map_.info.origin.position.x;
  double wy = last_map_.info.origin.position.y;
  double w = last_map_.info.width * last_map_.info.resolution;
  double h = last_map_.info.height * last_map_.info.resolution;

  *min_x = wx;
  *min_y = wy;
  *max_x = wx + w;
  *max_y = wy + h;
}

void OccupancyGridLayer::updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int, int, int, int)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_map_) return;

  const double res = last_map_.info.resolution;
  const unsigned int width = last_map_.info.width;
  const unsigned int height = last_map_.info.height;

  // rotation de la grille
  tf2::Quaternion q(
      last_map_.info.origin.orientation.x,
      last_map_.info.origin.orientation.y,
      last_map_.info.origin.orientation.z,
      last_map_.info.origin.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  yaw = -yaw;  // rotation inverse

  // centre de la grille
  const double center_x = width * 0.5 * res;
  const double center_y = height * 0.5 * res;

  // position du robot dans le frame de la costmap
  double robot_x = 0.0;
  double robot_y = 0.0;
  master_grid.mapToWorld(master_grid.getSizeInCellsX()/2,
                         master_grid.getSizeInCellsY()/2,
                         robot_x, robot_y);

  for (unsigned int j = 0; j < height; ++j) {
    for (unsigned int i = 0; i < width; ++i) {
      int8_t value = last_map_.data[j * width + i];      

      // cellule relative au centre
      double x_rel = (i + 0.5) * res - center_x;
      double y_rel = (j + 0.5) * res - center_y;

      // rotation autour du robot
      double x_rot = cos(yaw) * x_rel - sin(yaw) * y_rel;
      double y_rot = sin(yaw) * x_rel + cos(yaw) * y_rel;

      // position finale
      double x_final = robot_x + x_rot;
      double y_final = robot_y + y_rot;

      unsigned int mx, my;
      if (master_grid.worldToMap(x_final, y_final, mx, my)) {
        if (value < 0) {
            double distance_to_robot = sqrt((x_final - robot_x) * (x_final - robot_x) + (y_final - robot_y) * (y_final - robot_y));
            if (distance_to_robot < 0.8 && footprint_clearing_enabled_)
              master_grid.setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
            else
              master_grid.setCost(mx, my, nav2_costmap_2d::NO_INFORMATION);            
        } else if (value == 0) {
            uint8_t cost = static_cast<uint8_t>(value);
            master_grid.setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
        } else if (value == 100) {
            master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
        } else {
            // Scale 1-99 to 1-254 (avoiding 0 and 255)
            uint8_t cost = static_cast<uint8_t>(1 + (254 - 1) * (value - 1) / 98);
            master_grid.setCost(mx, my, cost);
        }
      }
    }
  }
}

}  // namespace nav2_occupancygrid_layer_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_occupancygrid_layer_plugin::OccupancyGridLayer, nav2_costmap_2d::Layer)
