#include "hector_waypoint_order/tsp_solver/brute_force_tsp_solver.h"

#include "hector_waypoint_order/utils/utils.h"
#include <algorithm>
#include <pluginlib/class_list_macros.h>


namespace hector_waypoint_order
{
void BruteForceTspSolver::initialize(ros::NodeHandle& nh, std::vector<geometry_msgs::PoseStamped> waypoints_unordered,
                                     CostMap cost_map)
{
  WaypointOrderComputerBase::initialize(nh, waypoints_unordered, cost_map);
}

std::vector<geometry_msgs::PoseStamped> BruteForceTspSolver::computeWaypointOrder()
{

  // Init first candidate path and make it circular
  std::vector<geometry_msgs::PoseStamped> candidate = waypoints_unordered_;
  candidate.push_back(candidate[0]);

  double min_path_costs = DBL_MAX;

  int permutation_counter = 0;
  long num_permutations = utils::fact(static_cast<int>(waypoints_unordered_.size()-1));

  ROS_WARN_STREAM("Permutations to check: " << num_permutations);

  do
  {
    // pass min path costs in order to break path cost computation if min_path_costs are exceeded
    double costs = computePathCosts(candidate, min_path_costs);

    if (costs < min_path_costs)
    {

      path_ = candidate;

      min_path_costs = costs;

      ROS_INFO_STREAM("Current minimal path costs: " << min_path_costs);

      publishPath();
    }

    ++permutation_counter;

    // print progress (not after each permutation for large num_permutations!)
    if(permutation_counter % static_cast<int>(std::ceil(static_cast<long double>(num_permutations) / 50000.0)) == 0)
    {
      auto progress = static_cast<double>(permutation_counter / (static_cast<long double>(num_permutations) / 100.0));
      std::cout << "Brute force TSP solver: Permutations checked: " << progress << "%\r";
      std::cout.flush();
    }



    // get next permutation but keep first = last point, as path is circular
  } while (ros::ok() && std::next_permutation(candidate.begin() + 1, candidate.end() - 1, utils::PoseLessComparator()));

  path_costs_ = min_path_costs;
  return path_;
}
} // end namespace hector_waypoint_order

PLUGINLIB_EXPORT_CLASS(hector_waypoint_order::BruteForceTspSolver,
                       hector_waypoint_order::WaypointOrderComputerBase)

