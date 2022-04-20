#ifndef HECTOR_WAYPOINT_ORDER_GREEDY_TSP_SOLVER_H
#define HECTOR_WAYPOINT_ORDER_GREEDY_TSP_SOLVER_H


#include "hector_waypoint_order/waypoint_order_computer_base.h"

namespace hector_waypoint_order
{

class GreedyTspSolver : public WaypointOrderComputerBase
{
public:

  void initialize(ros::NodeHandle& nh, std::vector<geometry_msgs::PoseStamped> waypoints_unordered,
                  CostMap cost_map) override;

  std::vector<geometry_msgs::PoseStamped> computeWaypointOrder() override;


private:

  geometry_msgs::PoseStamped getNearestNotVisitedWaypoint(const geometry_msgs::PoseStamped& current_waypoint);

  std::vector<geometry_msgs::PoseStamped> waypoints_not_visited_;

};


} // end namespace hector_waypoint_order

#endif //HECTOR_WAYPOINT_ORDER_GREEDY_TSP_SOLVER_H
