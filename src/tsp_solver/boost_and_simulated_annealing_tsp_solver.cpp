#include "hector_waypoint_order/tsp_solver/boost_and_simulated_annealing_tsp_solver.h"


#include <pluginlib/class_list_macros.h>

namespace hector_waypoint_order
{

void BoostAndSimulatedAnnealingTspSolver::initialize(ros::NodeHandle& nh,
                                                     std::vector<geometry_msgs::PoseStamped> waypoints_unordered,
                                                     CostMap cost_map)
{
  WaypointOrderComputerBase::initialize(nh, waypoints_unordered, cost_map);

  // init other solvers
  boost_tsp_solver_.initialize(nh_, waypoints_unordered_, cost_map_);
  simulated_annealing_tsp_solver_.initialize(nh_, waypoints_unordered_, cost_map_);
}

std::vector<geometry_msgs::PoseStamped> BoostAndSimulatedAnnealingTspSolver::computeWaypointOrder()
{
  // compute solution with boost as initial path for simulated annealing
  auto boost_result = boost_tsp_solver_.computeWaypointOrder();

  // TEMPORARY sleep
  sleep(2);

  simulated_annealing_tsp_solver_.setInitialPath(boost_result);

  // improve solution using simulated annealing
  path_ = simulated_annealing_tsp_solver_.computeWaypointOrder();

  path_costs_ = simulated_annealing_tsp_solver_.getPathCosts();

  return path_;
}

} // end namespace hector_waypoint_order


PLUGINLIB_EXPORT_CLASS(hector_waypoint_order::BoostAndSimulatedAnnealingTspSolver,
                       hector_waypoint_order::WaypointOrderComputerBase)

