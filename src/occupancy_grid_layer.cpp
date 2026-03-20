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

  // init TF buffer + listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node, true);

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

  // Étendre les bounds au footprint du robot
  useExtraBounds(min_x, min_y, max_x, max_y);
  if (footprint_clearing_enabled_) {
    for (auto &p : getFootprint()) {
      *min_x = std::min(*min_x, p.x);
      *min_y = std::min(*min_y, p.y);
      *max_x = std::max(*max_x, p.x);
      *max_y = std::max(*max_y, p.y);
    }
  }

}



// void OccupancyGridLayer::updateCosts(
//     nav2_costmap_2d::Costmap2D & master_grid,
//     int, int, int, int)
// {
//   std::lock_guard<std::mutex> lock(mutex_);
//   if (!has_map_) return;

//   const double res = last_map_.info.resolution;
//   const unsigned int width = last_map_.info.width;
//   const unsigned int height = last_map_.info.height;

//   // rotation de la grille
//   tf2::Quaternion q(
//       last_map_.info.origin.orientation.x,
//       last_map_.info.origin.orientation.y,
//       last_map_.info.origin.orientation.z,
//       last_map_.info.origin.orientation.w);
//   double roll, pitch, yaw;
//   tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
//   yaw = -yaw;  // rotation inverse

//   // centre de la grille
//   const double center_x = width * 0.5 * res;
//   const double center_y = height * 0.5 * res;

//   // position du robot dans le frame de la costmap
//   double robot_x = 0.0;
//   double robot_y = 0.0;
//   master_grid.mapToWorld(master_grid.getSizeInCellsX()/2,
//                          master_grid.getSizeInCellsY()/2,
//                          robot_x, robot_y);

//   for (unsigned int j = 0; j < height; ++j) {
//     for (unsigned int i = 0; i < width; ++i) {
//       int8_t value = last_map_.data[j * width + i];      

//       // cellule relative au centre
//       double x_rel = (i + 0.5) * res - center_x;
//       double y_rel = (j + 0.5) * res - center_y;

//       // rotation autour du robot
//       double x_rot = cos(yaw) * x_rel - sin(yaw) * y_rel;
//       double y_rot = sin(yaw) * x_rel + cos(yaw) * y_rel;

//       // position finale
//       double x_final = robot_x + x_rot;
//       double y_final = robot_y + y_rot;

//       unsigned int mx, my;
//       if (master_grid.worldToMap(x_final, y_final, mx, my)) {
//         if (value < 0) {
//             double distance_to_robot = sqrt((x_final - robot_x) * (x_final - robot_x) + (y_final - robot_y) * (y_final - robot_y));
//             if (distance_to_robot < 0.8 && footprint_clearing_enabled_)
//               master_grid.setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
//             else
//               master_grid.setCost(mx, my, nav2_costmap_2d::NO_INFORMATION);            
//         } else if (value == 0) {
//             uint8_t cost = static_cast<uint8_t>(value);
//             master_grid.setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
//         } else if (value == 100) {
//             master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
//         } else {
//             // Scale 1-99 to 1-254 (avoiding 0 and 255)
//             uint8_t cost = static_cast<uint8_t>(1 + (254 - 1) * (value - 1) / 98);
//             master_grid.setCost(mx, my, cost);
//         }
//       }
//     }
//   }
// }

void OccupancyGridLayer::updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_map_) return;

    // Propriétés, Origine et rotation de la grille occupancy
    const double res = last_map_.info.resolution;
    const unsigned int width = last_map_.info.width;
    const unsigned int height = last_map_.info.height;

    double ox, oy, roll, pitch, yaw;
    //   if(last_map_.header.frame_id == "map"){            
      ox = last_map_.info.origin.position.x;
      oy = last_map_.info.origin.position.y;
      tf2::Quaternion q(
          last_map_.info.origin.orientation.x,
          last_map_.info.origin.orientation.y,
          last_map_.info.origin.orientation.z,
          last_map_.info.origin.orientation.w);
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    // } else {
    //   // build PoseStamped properly and use tf_buffer_->transform(...)
    //   geometry_msgs::msg::PoseStamped origin_in;
    //   origin_in.header = last_map_.header;
    //   origin_in.pose.position = last_map_.info.origin.position;
    //   origin_in.pose.orientation = last_map_.info.origin.orientation;

    //   geometry_msgs::msg::PoseStamped origin_world;
    //   try {
    //     origin_world = tf_buffer_->transform(origin_in, std::string("map"), tf2::durationFromSec(0.2));
    //     ox = origin_world.pose.position.x;
    //     oy = origin_world.pose.position.y;
    //     tf2::Quaternion q(
    //         origin_world.pose.orientation.x,
    //         origin_world.pose.orientation.y,
    //         origin_world.pose.orientation.z,
    //         origin_world.pose.orientation.w);
    //     tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    //   } catch (const tf2::TransformException &ex) {
    //     RCLCPP_WARN(rclcpp::get_logger("OccupancyGridLayer"), "Could not transform occupancy grid origin to map frame: %s", ex.what());
    //     return;
    //   }
    // }



  // Accès rapide à la data
  auto getVal = [&](int i, int j) {
    return last_map_.data[j * width + i];
  };

  // For each cell in the master_grid
  for (unsigned int my = 0; my < master_grid.getSizeInCellsY(); ++my) {
    for (unsigned int mx = 0; mx < master_grid.getSizeInCellsX(); ++mx) {

      // Get world coordinates of the cell
      double wx, wy;
      master_grid.mapToWorld(mx, my, wx, wy);

      // Find location in the occupancy grid
      double i_f = (wx - ox);
      double j_f = (wy - oy);
      double x_f = cos(yaw) * i_f - sin(yaw) * j_f;
      double y_f = sin(yaw) * i_f + cos(yaw) * j_f;
      
      // Find cell in the occupancy grid
      x_f /= res;
      y_f /= res;
    
      int i = static_cast<int>(floor(x_f));
      int j = static_cast<int>(floor(y_f));
      double dx = x_f - i;
      double dy = y_f - j;
    
      // Interpolation bilinéaire si possible
      if (i >= 0 && j >= 0 && i < static_cast<int>(width) - 1 && j < static_cast<int>(height) - 1) {
        int v00 = getVal(i, j);
        int v10 = getVal(i + 1, j);
        int v01 = getVal(i, j + 1);
        int v11 = getVal(i + 1, j + 1);

        // Si une des valeurs est inconnue, on met NO_INFORMATION
        if (v00 < 0 || v10 < 0 || v01 < 0 || v11 < 0) {
          master_grid.setCost(mx, my, nav2_costmap_2d::NO_INFORMATION);
          continue;
        }

        double interp =
          v00 * (1.0 - dx) * (1.0 - dy) +
          v10 * dx * (1.0 - dy) +
          v01 * (1.0 - dx) * dy +
          v11 * dx * dy;

        int value = static_cast<int>(round(interp));

        // Conversion OccupancyGrid → costmap
        if (value == 0) {
          master_grid.setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
          continue;
        } else if (value >= 100) {
          master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
        } else {
          uint8_t cost = static_cast<uint8_t>(1 + (254 - 1) * (value - 1) / 98);
          master_grid.setCost(mx, my, cost);
        }
      }
      // Sinon, pas d'interpolation possible, on prend la valeur la plus proche si dans la grille
      else if (i >= 0 && j >= 0 && i < static_cast<int>(width) && j < static_cast<int>(height)) {
        int value = getVal(i, j);
        if (value < 0) {          
          master_grid.setCost(mx, my, nav2_costmap_2d::NO_INFORMATION);
        } else if (value == 0) {
          master_grid.setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
        } else if (value >= 100) {
          master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
        } else {
          uint8_t cost = static_cast<uint8_t>(1 + (254 - 1) * (value - 1) / 98);
          master_grid.setCost(mx, my, cost);
        }
      }
      // Sinon, hors de la grille
      else {
        master_grid.setCost(mx, my, nav2_costmap_2d::NO_INFORMATION);
      } 
    }           
  }

  // Libérer la zone sous le footprint du robot
  if(footprint_clearing_enabled_){
    std::vector<geometry_msgs::msg::Point> footprint = getFootprint();

    // Get robot position in world coordinates
    double robot_x, robot_y;
    master_grid.mapToWorld(master_grid.getSizeInCellsX()/2, master_grid.getSizeInCellsY()/2, robot_x, robot_y);

    for (auto &pt : footprint) {
      double x_rot = cos(-yaw) * pt.x - sin(-yaw) * pt.y;
      double y_rot = sin(-yaw) * pt.x + cos(-yaw) * pt.y;
      pt.x = robot_x + x_rot;
      pt.y = robot_y + y_rot;
      pt.z = 0.0;
    }

    if (!footprint.empty()) {
      master_grid.setConvexPolygonCost(footprint, nav2_costmap_2d::FREE_SPACE);
    }
  }  

}
}  // namespace nav2_occupancygrid_layer_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_occupancygrid_layer_plugin::OccupancyGridLayer, nav2_costmap_2d::Layer)
