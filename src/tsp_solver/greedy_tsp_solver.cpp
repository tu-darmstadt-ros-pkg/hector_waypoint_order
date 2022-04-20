#include "hector_waypoint_order/tsp_solver/greedy_tsp_solver.h"

#include <pluginlib/class_list_macros.h>
#include <random>


namespace hector_waypoint_order
{


void GreedyTspSolver::initialize(ros::NodeHandle& nh, std::vector<geometry_msgs::PoseStamped> waypoints_unordered,
                                 CostMap cost_map)
{
  WaypointOrderComputerBase::initialize(nh, waypoints_unordered, cost_map);

  waypoints_not_visited_ = waypoints_unordered_;
}


std::vector<geometry_msgs::PoseStamped> GreedyTspSolver::computeWaypointOrder()
{

  // randomly select a start node
  std::random_device rd;
  std::mt19937 start_node_gen = std::mt19937(rd());
  std::uniform_int_distribution<> start_node_dist = std::uniform_int_distribution<>(0, static_cast<int>(waypoints_not_visited_.size()) - 1);

  int start_idx = start_node_dist(start_node_gen);
  path_.push_back(waypoints_not_visited_[start_idx]);
  waypoints_not_visited_.erase(waypoints_not_visited_.begin() + start_idx);

  // add always the nearest waypoint that is not in path yet
  while (path_.size() < waypoints_unordered_.size())
  {
    // get nearest and remove from list
    auto nearest = getNearestNotVisitedWaypoint(path_.back());
    waypoints_not_visited_.erase(std::remove(waypoints_not_visited_.begin(), waypoints_not_visited_.end(), nearest),
                                 waypoints_not_visited_.end());

    // add to path
    path_.push_back(nearest);

    publishPath();
  }

  // add last = first element to make path circular
  path_.push_back(path_[0]);

  path_costs_ = computePathCosts(path_);

  publishPath();

  return path_;
}

geometry_msgs::PoseStamped
GreedyTspSolver::getNearestNotVisitedWaypoint(const geometry_msgs::PoseStamped& current_waypoint)
{
  geometry_msgs::PoseStamped nearest;
  double lowest_costs = DBL_MAX;

  for (auto& wp: waypoints_not_visited_)
  {
    double costs = cost_map_.at({current_waypoint, wp});
    if (costs < lowest_costs)
    {
      nearest = wp;
      lowest_costs = costs;
    }
  }

  return nearest;
}
} // end namespace hector_waypoint_order

PLUGINLIB_EXPORT_CLASS(hector_waypoint_order::GreedyTspSolver,
                       hector_waypoint_order::WaypointOrderComputerBase)