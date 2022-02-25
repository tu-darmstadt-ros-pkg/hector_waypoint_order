#ifndef HECTOR_WAYPOINT_ORDER_WAYPOINT_ORDER_COMPUTER_BASE_H
#define HECTOR_WAYPOINT_ORDER_WAYPOINT_ORDER_COMPUTER_BASE_H

#include "hector_waypoint_order/utils.h"

#include <geometry_msgs/PoseStamped.h>

namespace hector_waypoint_order
{


class WaypointOrderComputerBase
{
public:

  std::vector<geometry_msgs::PoseStamped> computeWaypointOrder(const CostMap& cost_map);
};
} // end namespace hector_waypoint_order

#endif //HECTOR_WAYPOINT_ORDER_WAYPOINT_ORDER_COMPUTER_BASE_H
