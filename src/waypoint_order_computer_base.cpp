#include "hector_waypoint_order/waypoint_order_computer_base.h"

#include <utility>

namespace hector_waypoint_order
{
void WaypointOrderComputerBase::initialize(std::vector<geometry_msgs::PoseStamped> waypoints_unordered,
                                           CostMap cost_map)
{
  waypoints_unordered_ = std::move(waypoints_unordered);
  cost_map_ = std::move(cost_map);
}

double WaypointOrderComputerBase::getPathCosts()
{
  // compute path costs if not set
  if (path_costs_ == 0 && path_.size() > 1)
  {
    for(unsigned int i = 1; i < path_.size(); ++i)
    {
      auto costs = cost_map_.at({path_[i-1], path_[i]});
      if(costs == -1)
      {
        // TODO how to handle path_costs -1 in getPathCosts?
        path_costs_ = DBL_MAX;
        break;
      }
      path_costs_ += costs;
    }
  }

  return path_costs_;
}
} // end namespace hector_waypoint_order