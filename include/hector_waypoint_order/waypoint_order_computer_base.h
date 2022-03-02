#ifndef HECTOR_WAYPOINT_ORDER_WAYPOINT_ORDER_COMPUTER_BASE_H
#define HECTOR_WAYPOINT_ORDER_WAYPOINT_ORDER_COMPUTER_BASE_H

#include "hector_waypoint_order/utils.h"

#include <geometry_msgs/PoseStamped.h>

namespace hector_waypoint_order
{


class WaypointOrderComputerBase
{
public:

  virtual void initialize(std::vector<geometry_msgs::PoseStamped> waypoints_unordered, CostMap cost_map);

  virtual std::vector<geometry_msgs::PoseStamped> computeWaypointOrder() = 0;

  double getPathCosts();

protected:

  std::vector<geometry_msgs::PoseStamped> waypoints_unordered_;

  CostMap cost_map_;

  std::vector<geometry_msgs::PoseStamped> path_;
  double path_costs_;

};
} // end namespace hector_waypoint_order

#endif //HECTOR_WAYPOINT_ORDER_WAYPOINT_ORDER_COMPUTER_BASE_H
