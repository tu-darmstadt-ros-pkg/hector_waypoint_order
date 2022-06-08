#ifndef HECTOR_WAYPOINT_ORDER_BRUTE_FORCE_TSP_SOLVER_H
#define HECTOR_WAYPOINT_ORDER_BRUTE_FORCE_TSP_SOLVER_H


#include "hector_waypoint_order/waypoint_order_computer_base.h"


namespace hector_waypoint_order
{

class BruteForceTspSolver : public WaypointOrderComputerBase
{
public:

  void initialize(ros::NodeHandle& nh, std::vector<geometry_msgs::PoseStamped> waypoints_unordered,
                  CostMap cost_map) override;

  std::vector<geometry_msgs::PoseStamped> computeWaypointOrder() override;
};
} // end namespace hector_waypoint_order


#endif //HECTOR_WAYPOINT_ORDER_BRUTE_FORCE_TSP_SOLVER_H
