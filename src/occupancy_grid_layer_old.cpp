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

// void OccupancyGridLayer::updateCosts(
//     nav2_costmap_2d::Costmap2D & master_grid,
//     int, int, int, int)
// {
//   std::lock_guard<std::mutex> lock(mutex_);
//   if (!has_map_) return;

//   const double res = last_map_.info.resolution;
//   const unsigned int width  = last_map_.info.width;
//   const unsigned int height = last_map_.info.height;
//   const double ox = last_map_.info.origin.position.x;
//   const double oy = last_map_.info.origin.position.y;

//   // Accès rapide à la data
//   auto getVal = [&](int i, int j) {
//     return last_map_.data[j * width + i];
//   };

//   // Parcours de chaque cellule de la costmap
//   for (unsigned int my = 0; my < master_grid.getSizeInCellsY(); ++my) {
//     for (unsigned int mx = 0; mx < master_grid.getSizeInCellsX(); ++mx) {

//       double wx, wy;
//       master_grid.mapToWorld(mx, my, wx, wy);

//       // Transformation world -> occupancy grid (robot frame)
//       double i_f = (wx - ox) / res;
//       double j_f = (wy - oy) / res;

//       // Vérifie si dans la carte
//       if (i_f < 0 || j_f < 0 || i_f >= width - 1 || j_f >= height - 1) {
//         master_grid.setCost(mx, my, nav2_costmap_2d::NO_INFORMATION);
//         continue;
//       }

//       int i0 = floor(i_f);
//       int j0 = floor(j_f);
//       double dx = i_f - i0;
//       double dy = j_f - j0;

//       int v00 = getVal(i0,   j0);
//       int v10 = getVal(i0+1, j0);
//       int v01 = getVal(i0,   j0+1);
//       int v11 = getVal(i0+1, j0+1);

//       // Plus proche voisin
//       int v_nearest = (dx < 0.5 ? (dy < 0.5 ? v00 : v01)
//                                 : (dy < 0.5 ? v10 : v11));

//       if (v_nearest < 0) {
//         master_grid.setCost(mx, my, nav2_costmap_2d::NO_INFORMATION);
//         continue;
//       }
//       if (v_nearest == 100) {
//         master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
//         continue;
//       }

//       // Interpolation bilinéaire
//       double v_interp =
//           v00 * (1 - dx) * (1 - dy) +
//           v10 * dx       * (1 - dy) +
//           v01 * (1 - dx) * dy +
//           v11 * dx       * dy;

//       // Conversion en coût costmap
//       if (v_interp <= 0.5) {
//         master_grid.setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
//       } else if (v_interp >= 99.5) {
//         master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
//       } else {
//         uint8_t cost = static_cast<uint8_t>(
//             1 + (254 - 1) * (v_interp - 1) / 98);
//         master_grid.setCost(mx, my, cost);
//       }
//     }
//   }
void OccupancyGridLayer::updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int, int, int, int)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_map_) return;

  const double res = last_map_.info.resolution;
  const unsigned int width  = last_map_.info.width;
  const unsigned int height = last_map_.info.height;
  const double ox = last_map_.info.origin.position.x;
  const double oy = last_map_.info.origin.position.y;

  // Accès rapide à la data
  auto getVal = [&](int i, int j) {
    return last_map_.data[j * width + i];
  };

  // ✅ Position du robot pour le clear footprint
  double robot_x, robot_y;
  master_grid.mapToWorld(master_grid.getSizeInCellsX()/2,
                         master_grid.getSizeInCellsY()/2,
                         robot_x, robot_y);

  // ✅ Parcours de chaque cellule de la costmap
  for (unsigned int my = 0; my < master_grid.getSizeInCellsY(); ++my) {
    for (unsigned int mx = 0; mx < master_grid.getSizeInCellsX(); ++mx) {

      double wx, wy;
      master_grid.mapToWorld(mx, my, wx, wy);

      // ✅ SIMPLIFICATION : Transformation directe (même repère map)
      // Plus besoin de transformation TF complexe !
      double i_f = (wx - ox) / res;
      double j_f = (wy - oy) / res;

      // Vérifie si dans la carte
      if (i_f < 0 || j_f < 0 || i_f >= width - 1 || j_f >= height - 1) {
        // ✅ CORRECTION : Plus sécurisé pour navigation
        double distance_to_robot = sqrt((wx - robot_x) * (wx - robot_x) + 
                                      (wy - robot_y) * (wy - robot_y));
        if (footprint_clearing_enabled_ && distance_to_robot <= 0.7) {
          master_grid.setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
        } else {
          master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);  // Plus sûr
        }
        continue;
      }

      int i0 = floor(i_f);
      int j0 = floor(j_f);
      double dx = i_f - i0;
      double dy = j_f - j0;

      int v00 = getVal(i0,   j0);
      int v10 = getVal(i0+1, j0);
      int v01 = getVal(i0,   j0+1);
      int v11 = getVal(i0+1, j0+1);

      // ✅ Traitement des valeurs inconnues plus intelligent
      if (v00 < 0 || v10 < 0 || v01 < 0 || v11 < 0) {
        double distance_to_robot = sqrt((wx - robot_x) * (wx - robot_x) + 
                                      (wy - robot_y) * (wy - robot_y));
        if (footprint_clearing_enabled_ && distance_to_robot <= 0.7) {
          master_grid.setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
        } else {
          master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);  // Plus sûr
        }
        continue;
      }

      // Plus proche voisin pour les obstacles letaux
      int v_nearest = (dx < 0.5 ? (dy < 0.5 ? v00 : v01)
                                : (dy < 0.5 ? v10 : v11));

      if (v_nearest >= 100) {
        master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
        continue;
      }

      // ✅ Interpolation bilinéaire (plus lisse)
      double v_interp =
          v00 * (1.0 - dx) * (1.0 - dy) +
          v10 * dx * (1.0 - dy) +
          v01 * (1.0 - dx) * dy +
          v11 * dx * dy;

      // Conversion en coût costmap
      if (v_interp <= 0.5) {
        master_grid.setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
      } else if (v_interp >= 99.5) {
        master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
      } else {
        // ✅ Scaling amélioré (évite 254 et 255)
        uint8_t cost = static_cast<uint8_t>(1 + (252 * (v_interp - 1)) / 98);
        master_grid.setCost(mx, my, cost);
      }
    }
  }
  // Clear un cercle de 60cm de rayon
  for (unsigned int my = 0; my < master_grid.getSizeInCellsY(); ++my) {
    for (unsigned int mx = 0; mx < master_grid.getSizeInCellsX(); ++mx) {
      double wx, wy;
      master_grid.mapToWorld(mx, my, wx, wy);
      if (wx*wx + wy*wy < 0.7*0.7) {
        master_grid.setCost(mx, my, 0.5);
      }
    }
  }

}

}  // namespace nav2_occupancygrid_layer_plugin

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_occupancygrid_layer_plugin::OccupancyGridLayer, nav2_costmap_2d::Layer)
