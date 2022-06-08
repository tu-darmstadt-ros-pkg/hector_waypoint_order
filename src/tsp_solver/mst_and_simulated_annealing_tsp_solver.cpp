#include "hector_waypoint_order/tsp_solver/mst_and_simulated_annealing_tsp_solver.h"


#include <pluginlib/class_list_macros.h>

namespace hector_waypoint_order
{

void MstAndSimulatedAnnealingTspSolver::initialize(ros::NodeHandle& nh,
                                                   std::vector<geometry_msgs::PoseStamped> waypoints_unordered,
                                                   CostMap cost_map)
{
  WaypointOrderComputerBase::initialize(nh, waypoints_unordered, cost_map);

  // init other solvers
  mst_tsp_solver_.initialize(nh_, waypoints_unordered_, cost_map_);
  simulated_annealing_tsp_solver_.initialize(nh_, waypoints_unordered_, cost_map_);
}

std::vector<geometry_msgs::PoseStamped> MstAndSimulatedAnnealingTspSolver::computeWaypointOrder()
{
  // compute solution with boost as initial path for simulated annealing
  auto mst_result = mst_tsp_solver_.computeWaypointOrder();

  simulated_annealing_tsp_solver_.setInitialPath(mst_result);

  // improve solution using simulated annealing
  path_ = simulated_annealing_tsp_solver_.computeWaypointOrder();

  path_costs_ = simulated_annealing_tsp_solver_.getPathCosts();

  num_steps_last_change_ = simulated_annealing_tsp_solver_.getNumStepsOfLastChange();

  return path_;
}
} // end namespace hector_waypoint_order


PLUGINLIB_EXPORT_CLASS(hector_waypoint_order::MstAndSimulatedAnnealingTspSolver,
                       hector_waypoint_order::WaypointOrderComputerBase)

