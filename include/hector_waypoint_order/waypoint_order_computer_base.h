#ifndef HECTOR_WAYPOINT_ORDER_WAYPOINT_ORDER_COMPUTER_BASE_H
#define HECTOR_WAYPOINT_ORDER_WAYPOINT_ORDER_COMPUTER_BASE_H

#include "hector_waypoint_order/utils/utils.h"

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

namespace hector_waypoint_order
{


class WaypointOrderComputerBase
{
public:

  /**
   * Initialize the waypoint order computation.
   * @param nh
   * @param waypoints_unordered Unordered list containing all waypoints.
   * @param cost_map Cost map for the points in waypoints_unordered. See CostComputerBase::computeCosts for details.
   */
  virtual void
  initialize(ros::NodeHandle& nh, std::vector<geometry_msgs::PoseStamped> waypoints_unordered, CostMap cost_map);


  /**
   * Compute the waypoint order.
   * @return path in correct order.
   */
  virtual std::vector<geometry_msgs::PoseStamped> computeWaypointOrder() = 0;

  /**
   * Get path costs.
   * If the costs have not been set by the computeWaypointOrder implementation,
   * the are computed here for path_ using the costs from the cost_map_.
   * @return path costs, 0 if path is empty
   */
  double getPathCosts();

protected:

  /**
   * Compute path costs for given path using the costs from the cost_map_.
   * @param path
   * @param max_costs When the computed costs get greater than max_costs, the computation is aborted and DBL_MAX is returned.
   * @return path costs, 0 if path is empty, DBL_MAX if max_costs are exceeded.
   */
  double computePathCosts(std::vector<geometry_msgs::PoseStamped> path, double max_costs = DBL_MAX);

  void publishPath();

  std::vector<geometry_msgs::PoseStamped> waypoints_unordered_;

  CostMap cost_map_;

  std::vector<geometry_msgs::PoseStamped> path_;
  double path_costs_;

  ros::NodeHandle nh_;

  bool publish_path_;

  ros::Publisher path_pub_;
};
} // end namespace hector_waypoint_order

#endif //HECTOR_WAYPOINT_ORDER_WAYPOINT_ORDER_COMPUTER_BASE_H
