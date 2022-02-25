#ifndef HECTOR_WAYPOINT_ORDER_PATH_COST_COMPUTER_H
#define HECTOR_WAYPOINT_ORDER_PATH_COST_COMPUTER_H


#include "hector_waypoint_order/cost_computation/cost_computer_base.h"
#include "hector_waypoint_order/cost_computation/path_planner/path_planner_base.h"

#include <memory>
#include <pluginlib/class_loader.h>

namespace hector_waypoint_order
{


class PathCostComputer : public CostComputerBase
{
public:

  PathCostComputer() = default;

  void initialize(ros::NodeHandle& nh) override;

  CostMap computeCosts(std::vector<geometry_msgs::PoseStamped> waypoints) override;

private:
  /**
   * Compute the length of a path given all poses of the path. Between the poses, the euclidean distance is used.
   * @param path ordered list of poses
   * @return length of path
   */
  static double computePathLength(const std::vector<geometry_msgs::PoseStamped>& path);


  std::unique_ptr<pluginlib::ClassLoader<hector_waypoint_order::PathPlannerBase>> path_planner_loader_ptr_;
  std::unique_ptr<PathPlannerBase> path_planner_;
};
} // end namespace hector_waypoint_order

#endif //HECTOR_WAYPOINT_ORDER_PATH_COST_COMPUTER_H
